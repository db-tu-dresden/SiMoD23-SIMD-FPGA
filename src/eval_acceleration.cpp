/*
Copyright 2023 Database Research Group, TU Dresden

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <CL/sycl.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <sycl/ext/intel/fpga_extensions.hpp>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

// Template Vector Library (TVL) main header 
#include <tvlintrin.hpp>

// Time
#include <sys/time.h>
// Sleep
#include <unistd.h>

// Needed for oneAPI
using namespace sycl;
using namespace std::chrono;

using Type = float;  // type to use for the test


/**
 * \brief Kernel for DB-Filter Count.
 * \details This free function counts the number of elements within a given range (lower bound \a l, upper bound \a u). 
 * If a data element \a d is within [\a l, \a u], a counter is incremented.
 * \tparam SIMDT TVL specific SIMD type.
 * \tparam DataPtrT Pointer type to a chunk of memory (has to be dereferenceable).
 * \tparam DataT Data type which should be processed. For the sake of this submission we assume that DataT is of type float (and has a size of 4 byte).
 * \param data_ptr Template pointer type to a chunk of consecutive data.
 * \param element_count Number of elements within the chunk.
 * \param lower Lower bound (\a l).
 * \param upper Upper bound (\a u).
 * \returns Number of elements within the given range.
 *
 */
template<typename SIMDT, typename DataPtrT, typename DataT = Type>
uint32_t filter_kernel(DataPtrT data_ptr, size_t element_count, DataT lower, DataT upper) {
    using namespace tvl;
    // As the data is of type float we have to create a specific SIMD type which can carry out the simdified counting.
    using CoundSIMDT = tvl::simd<uint32_t, typename SIMDT::target_extension, SIMDT::vector_size_b()>;

    // Initialization of the result vector.
    auto result_vec = set1<CoundSIMDT>(0);
    // Creation of an increment vector which is used to increase the result vector whenever the corresponding value is in the given range.
    const auto increment_vec = set1<CoundSIMDT>(1);

    // Creation of the compare-vectors.
    const auto lower_vec = set1<SIMDT>(lower);
    const auto upper_vec = set1<SIMDT>(upper);
    for (size_t i = 0; i < element_count; i += SIMDT::vector_element_count()) {
        // Load (unaligned) data into a vector.
        const auto data_vec = loadu<SIMDT>(&data_ptr[i]);
        // Execution of the comparison.
        const auto result_mask = between_inclusive<SIMDT>(data_vec, lower_vec, upper_vec);

        // Creation of the increment vector of the current iteration (contains 0 for all elements which are not in the given range, 1 otherwise).
        const auto increment_result_vec = binary_and<CoundSIMDT>(reinterpret<SIMDT, CoundSIMDT>(to_vector<SIMDT>(result_mask)), increment_vec);
        // Increment of the result vector.
        result_vec = add<CoundSIMDT>(result_vec, increment_result_vec);
    }
    // Horizontally add all elements within the result vector to retrieve the final result.
    return hadd<CoundSIMDT>(result_vec);
}

/**
 * \brief Wrapper function which calls the DB-Filter Count kernel and measures the execution time on the cpu. 
 * \tparam SIMDT TVL specific SIMD type.
 * \tparam DataPtrT Pointer type to a chunk of memory (has to be dereferenceable).
 * \tparam DataT Data type which should be processed. For the sake of this submission we assume that DataT is of type float (and has a size of 4 byte).
 * \param ready_future Shared future to synchronize the start of the execution of multiple threads.
 * \param tid Thread-Id.
 * \param out_time Pointer to a memory location where the execution time should be stored to.
 * \param out_results Pointer to a memory location where the result should be stored to.
 * \param data_ptr Template pointer type to a chunk of consecutive data.
 * \param element_count Number of elements within the chunk.
 * \param lower Lower bound (\a l).
 * \param upper Upper bound (\a u).
 */
template<typename SIMDT, typename DataPtrT, typename DataT = Type>
void filter_cpu_wrapper(std::shared_future<void>* ready_future, size_t tid, long* out_time, long* out_results, DataPtrT data_ptr, size_t element_count, DataT lower, DataT upper) {
    ready_future->wait();
    auto start = std::chrono::high_resolution_clock::now();
    *out_results = filter_kernel<SIMDT>(data_ptr, element_count, lower, upper);
    auto end = std::chrono::high_resolution_clock::now();
    *out_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

void exception_handler(exception_list exceptions);

// Forward declaration to simplify demangling for icpx. 
class filterKernel_tvl;

/**
 * \brief Wrapper function which calls the DB-Filter Count kernel and measures the execution time on the FPGA. 
 * \tparam SIMDT TVL specific SIMD type.
 * \tparam DataPtrT Pointer type to a chunk of memory (has to be dereferenceable).
 * \tparam DataT Data type which should be processed. For the sake of this submission we assume that DataT is of type float (and has a size of 4 byte).
 * \param q Pointer to OneAPI queue.
 * \param ready_future Shared future to synchronize the start of the execution of multiple threads.
 * \param out_time Pointer to a memory location where the execution time should be stored to.
 * \param out_results Pointer to a memory location where the result should be stored to.
 * \param data_ptr_host Template pointer type to a chunk of consecutive data which resides on the host.
 * \param element_count Number of elements within the chunk.
 * \param lower Lower bound (\a l).
 * \param upper Upper bound (\a u).
 */
template<typename SIMDT, typename DataPtrT, typename DataT = Type>
void filter_fpga_wrapper(queue* q, std::shared_future<void>* ready_future, long* out_time, long* out_results, DataPtrT data_ptr_host, size_t element_count, DataT lower, DataT upper) {
    ready_future->wait();
    auto start = high_resolution_clock::now();
    q->submit([&](handler& h) {
         h.single_task<filterKernel_tvl>([=]() [[intel::kernel_args_restrict]] {
             host_ptr<DataT> data_ptr(data_ptr_host);
             host_ptr<long> out(out_results);
             out_results[0] = filter_kernel<SIMDT>(data_ptr, element_count, lower, upper);
         });
     }).wait();
    auto end = high_resolution_clock::now();
    *out_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

/**
 * \brief Benchmark function.
 * \details Execute the given Filter-Count operators and log the execution times.
 * \tparam WithFPGA Indicates, whether the execution should be include the FPGA.
 * \tparam SIMDT TVL specific SIMD type for the CPU.
 * \tparam SIMDTfpga TVL specific SIMD type for the FPGA.
 * \param q Reference to OneAPI queue.
 * \param total_elements Number of elements which should be processed.
 * \param core_count Reference to a vector containing different numbers of cores which should be used for benchmarking.
 * \param data_ptrs Reference to an array containing the pointers to the data buffers.
 * \param result_ptrs Reference to an array containing the pointers to the result buffers.
 * \param time_ptrs Reference to an array containing the pointers to the exeuction-time buffers.
 */
template<bool WithFPGA, typename SIMDT, typename SIMDTfpga, typename DataT = Type>
void benchmark(queue& q, size_t total_elements, std::vector<size_t>& core_count, std::array<DataT*, 9>& data_ptrs, std::array<long*, 9>& result_ptrs, std::array<long*, 9>& time_ptrs) {
    // Inclusive bounds for the filter kernel. 
    const DataT lower_bound = 5;
    const DataT upper_bound = 15;

    // Result-log preparation.
    std::ofstream outfile;
    std::string fname = std::to_string(total_elements) + "_" + tvl::type_name<typename SIMDT::target_extension>();
    if(WithFPGA) {
        outfile.open(fname + "_fpga.log");
        std::cout << "Adding FPGA as additional compute unit, starting dummy run." << std::endl;
    } else {
        outfile.open(fname + ".log");
    }
    outfile << "core_count\tinput_size_mb_per_thread\t";
    for(size_t i = 0; i < time_ptrs.size()-1; ++i) {
        outfile << "Core-" << std::to_string(i) << "-us\t";
    }
    outfile << "fpga-us\tsum_coretime\ttotal_results" << std::endl;

    std::cout << "core_count\tinput_size_mb_per_thread\t";
    for(size_t i = 0; i < time_ptrs.size()-1; ++i) {
        std::cout << "Core-" << std::to_string(i) << "-us\t";
    }
    std::cout << "fpga-us\tsum_coretime\ttotal_results" << std::endl;

    // Clean-up lambda for benchmark repetition.
    auto cleanup = [&]() -> void {
        for(size_t i = 0; i < time_ptrs.size(); ++i) {
            *time_ptrs[i] = 0;
            *result_ptrs[i] = 0;
        }
    };

    cleanup();

    std::vector<std::thread> pool;
    std::cout << "TCnt\tus\tMB/s\tResults" << std::endl;
    for(size_t i : core_count) {
        pool.reserve(i + 1);

        std::promise<void> p;
        std::shared_future<void> ready_future(p.get_future());

        // Add the current amount of threads for the CPU-Filter-Count to the thread-pool.
        for(size_t tid = 0; tid < i; tid++) {
            pool.emplace_back(filter_cpu_wrapper<SIMDT, DataT*, DataT>, &ready_future, tid, time_ptrs[tid], result_ptrs[tid], data_ptrs[tid], total_elements, lower_bound, upper_bound);
        }

        // If fpga should be used, we add another fpga kernel to the pool
        if(WithFPGA) {
            std::promise<void> fpga_promise;
            std::shared_future<void> fpga_ready_future(fpga_promise.get_future());
            fpga_promise.set_value();
            // We execute the fpga code once to properly load the FPGA program.
            filter_fpga_wrapper<SIMDTfpga>(&q, &fpga_ready_future, time_ptrs[i], result_ptrs[i], data_ptrs[i], total_elements, lower_bound, upper_bound);
            pool.emplace_back(filter_fpga_wrapper<SIMDTfpga,DataT*,DataT>, &q, &ready_future, time_ptrs[i], result_ptrs[i], data_ptrs[i], total_elements, lower_bound, upper_bound);
        }

        {
            using namespace std::chrono_literals;
            // Wating a second for every thread to reach the barrier...
            std::this_thread::sleep_for(500ms);
        }

        // Run the threads.
        p.set_value();

        for(auto& t : pool) {
            t.join();
        }

        double total_results = 0;
        double sum_coretime = 0.0;

        for(size_t tid = 0; tid < i; ++tid) {
            total_results += *result_ptrs[tid];
            sum_coretime += *time_ptrs[tid];
        }

        double time_fpga = WithFPGA ? *time_ptrs[i] : 0.0;
        double results_fpga = WithFPGA ? *result_ptrs[i] : 0.0;

        total_results = WithFPGA ? (total_results + results_fpga) / (i + 1) : total_results / i;
        double input_size_mb_per_thread = static_cast<double>(total_elements * sizeof(DataT)) / (1024 * 1024);

        auto log_to_stream = []<typename T>(std::ostream& stream, std::array<T, 9> arr, size_t idxs) -> void {
            for (size_t i = 0; i < idxs; ++i) {
                stream << *arr[i] << "\t";
            }
            for ( size_t i = idxs; i < arr.size()-1; ++i ) {
                stream << 0 << "\t";
            }
        };

        outfile << i << "\t"
                << input_size_mb_per_thread << "\t";
        log_to_stream(outfile, time_ptrs, i);
        outfile << time_fpga << "\t"
                << sum_coretime << "\t"
                << total_results
                << std::endl;

        std::cout << std::setw(2)
                  << i << "\t"
                  << input_size_mb_per_thread << "\t"
                  << std::fixed << std::setprecision(2);
        log_to_stream(std::cout, time_ptrs, i);
        std::cout << time_fpga << "\t"
                  << sum_coretime << "\t"
                  << std::setprecision(0) << total_results
                  << std::endl;

        cleanup();

        pool.clear();
    }
    outfile.close();
}

// main
int main(int argc, char* argv[]) {
    // the device selector
#ifdef FPGA_EMULATOR
    ext::intel::fpga_emulator_selector selector;
#else
    ext::intel::fpga_selector selector;
#endif

    // create the device queue
    auto props = property_list{property::queue::enable_profiling()};
    queue q(selector, exception_handler, props);

    // make sure the device supports USM device allocations
    device d = q.get_device();
    if (!d.get_info<info::device::usm_device_allocations>()) {
        std::cerr << "ERROR: The selected device does not support USM device"
                  << " allocations" << std::endl;
        std::terminate();
    }
    if (!d.get_info<info::device::usm_host_allocations>()) {
        std::cerr << "ERROR: The selected device does not support USM host"
                  << " allocations" << std::endl;
        std::terminate();
    }

    size_t size;
    if (argc != 2)  // argc should be 2 for correct execution
    {
        size = 1024;
    } else {
        size = atoll(argv[1]);
    }

    std::cout << "Element count: " << size << std::endl;

    size_t number_CL = 0;

    using SIMDTfpga = tvl::simd<Type, tvl::fpga, 512>;
    const size_t vec_elems = SIMDTfpga::vector_element_count();

    if (size % vec_elems == 0) {
        number_CL = size / vec_elems;
    } else {
        number_CL = size / vec_elems + 1;
    }

    double input_size_mb = static_cast<double>(size * sizeof(Type)) / (1024 * 1024);
    std::cout << "Number CLs: " << number_CL << " (" << input_size_mb << " MB)" << std::endl;

    // Allocate input/output data in pinned host memory
    // Used in both tests, for convenience

    std::vector<size_t> core_count{1, 2, 3, 4, 5, 6, 7, 8};
    std::array<Type*, 9> data_ptrs;
    std::array<long*, 9> result_ptrs;
    std::array<long*, 9> time_ptrs;

    for (size_t cc = 0; cc < data_ptrs.size(); ++cc) {
        if ((data_ptrs[cc] = malloc_host<Type>(number_CL * vec_elems, q)) == nullptr) {
            std::cerr << "ERROR: could not allocate space for 'in'" << std::endl;
            std::terminate();
        }
        if ((result_ptrs[cc] = malloc_host<long>(1, q)) == nullptr) {
            std::cerr << "ERROR: could not allocate space for 'in'" << std::endl;
            std::terminate();
        }
        if ((time_ptrs[cc] = malloc_host<long>(1, q)) == nullptr) {
            std::cerr << "ERROR: could not allocate space for 'in'" << std::endl;
            std::terminate();
        }
    }

    const size_t total_elements = number_CL * vec_elems;
    std::mt19937 engine(0xc01dbadc00ffee);
    std::uniform_real_distribution<float> dist(0, 20);

    // Init input buffer
    for (int i = 0; i < (number_CL * vec_elems); ++i) {
        if (i < size) {
            data_ptrs[0][i] = dist(engine);
        } else {
            data_ptrs[0][i] = 0;
        }
    }

    for (size_t i = 1; i < data_ptrs.size(); ++i) {
        memcpy(data_ptrs[i], data_ptrs[0], total_elements * sizeof(Type));
    }

    benchmark<false, tvl::simd<Type, tvl::sse>, SIMDTfpga>(q, total_elements, core_count, data_ptrs, result_ptrs, time_ptrs);
    benchmark<true, tvl::simd<Type, tvl::sse>, SIMDTfpga>(q, total_elements, core_count, data_ptrs, result_ptrs, time_ptrs);
    benchmark<false, tvl::simd<Type, tvl::avx2>, SIMDTfpga>(q, total_elements, core_count, data_ptrs, result_ptrs, time_ptrs);
    benchmark<true, tvl::simd<Type, tvl::avx2>, SIMDTfpga>(q, total_elements, core_count, data_ptrs, result_ptrs, time_ptrs);
    benchmark<false, tvl::simd<Type, tvl::avx512>, SIMDTfpga>(q, total_elements, core_count, data_ptrs, result_ptrs, time_ptrs);
    benchmark<true, tvl::simd<Type, tvl::avx512>, SIMDTfpga>(q, total_elements, core_count, data_ptrs, result_ptrs, time_ptrs);

    for (size_t cc = 0; cc < data_ptrs.size(); ++cc) {
        sycl::free(data_ptrs[cc], q);
        sycl::free(result_ptrs[cc], q);
        sycl::free(time_ptrs[cc], q);
    }
}

void exception_handler(exception_list exceptions) {
    for (std::exception_ptr const& e : exceptions) {
        try {
            std::rethrow_exception(e);
        } catch (exception const& e) {
            std::cout << "Caught asynchronous SYCL exception:\n"
                      << e.what() << std::endl;
        }
    }
}
