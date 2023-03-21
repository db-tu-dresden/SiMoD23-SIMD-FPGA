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

// Time
#include <sys/time.h>
// Sleep
#include <unistd.h>
#include <CL/sycl.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sycl/ext/intel/fpga_extensions.hpp>
#include <tuple>

// Template Vector Library (TVL) main header 
#include <tvlintrin.hpp>

#include <utility>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
using namespace sycl;
using namespace std::chrono;

using Type = float;  // type to use for the test

const Type lower_bound = 5.0;
const Type upper_bound = 15.0;
////////////////////////////////////////////////////////////////////////////////

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
 * \param data_ptr Template pointer type to a chunk of consecutive data.
 * \param element_count Number of elements within the chunk.
 */
template<typename SIMDT, typename DataPtrT, typename DataT = Type>
double filter_cpu_wrapper(DataPtrT data_ptr, size_t element_count) {
    double speed = 0;
    volatile size_t results = 0;
    for(size_t its = 0; its < 10; ++its) {
        auto start = std::chrono::high_resolution_clock::now();
        results += filter_kernel<SIMDT>(data_ptr, element_count, lower_bound, upper_bound);
        auto end = std::chrono::high_resolution_clock::now();
        speed += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
    std::cout << "Result for " << tvl::type_name<SIMDT>() << ": " << results / 10 << std::endl;
    return speed / 10;
}

/**
 * \brief Wrapper function which calls the DB-Filter Count kernel and measures the execution time on the FPGA. 
 * \tparam SIMDT TVL specific SIMD type.
 * \tparam DataPtrT Pointer type to a chunk of memory (has to be dereferenceable).
 * \tparam DataT Data type which should be processed. For the sake of this submission we assume that DataT is of type float (and has a size of 4 byte).
 * \param q Pointer to OneAPI queue.
 * \param data_ptr_host Template pointer type to a chunk of consecutive data which resides on the host.
 * \param out_results Pointer to a memory location where the result should be stored to.
 * \param element_count Number of elements within the chunk.
 */
template<typename SIMDT, typename DataPtrT, typename DataT = Type>
double filter_fpga_wrapper(queue* q, DataPtrT data_ptr_host, long* out_host, size_t element_count) {
    auto start = high_resolution_clock::now();
    q->submit([&](handler& h) {
         h.single_task([=]() [[intel::kernel_args_restrict]] {
             host_ptr<Type> in(data_ptr_host);
             host_ptr<long> out(out_host);
             out[0] = filter_kernel<SIMDT>(in, element_count, lower_bound, upper_bound);
         });
     }).wait();
    auto end = high_resolution_clock::now();
    return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
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

    // How many elements of type "Type" should we generate?
    size_t size = 0;
    if (argc != 2) {
        size = 1024;
    } else {
        size = atoll(argv[1]);
    }

    std::cout << "Element count: " << size << std::endl;

    // Define for Allocate input/output data in pinned host memory
    // Used in all three tests, for convenience
    Type* in;
    // int *out;
    long* out_aggr;
    long* out_aggr_2;

    std::cout << std::endl
              << "### filter-count ###" << std::endl
              << std::endl;

    size_t number_CL = 0;
    // Magic to make sure we can satisfy PCIe
    const size_t vec_elems = 2048 / 8 / sizeof(Type);
    if (size % vec_elems == 0) {
        number_CL = size / vec_elems;
    } else {
        number_CL = size / vec_elems + 1;
    }

    std::cout << "Number CLs: " << number_CL << std::endl;

    // Allocate input/output data in pinned host memory / USM
    if ((in = malloc_host<Type>(number_CL * vec_elems, q)) == nullptr) {
        std::cerr << "ERROR: could not allocate space for 'in'" << std::endl;
        std::terminate();
    }
    if ((out_aggr = malloc_host<long>(5, q)) == nullptr) {
        std::cerr << "ERROR: could not allocate space for 'out'" << std::endl;
        std::terminate();
    }

    std::mt19937 engine(0xc01dbadc00ffee);
    std::uniform_real_distribution<float> dist(0, 20);

    // Init input buffer
    out_aggr[0] = 0;
    for (int i = 0; i < (number_CL * vec_elems); ++i) {
        if (i < size) {
            in[i] = dist(engine);
            // Scalar test to verify correctness later
            if (in[i] >= lower_bound && in[i] <= upper_bound) out_aggr[0]++;
        } else {
            in[i] = 0;
        }
    }

    std::cout << "Buffer filled completely with '1'" << std::endl;
    std::cout << "in[0], in[1], in[2] ... : " << in[0] << " " << in[1] << " " << in[2] << std::endl;
    std::cout << "... in[size-1]: " << in[(number_CL * vec_elems) - 1] << std::endl;

    std::cout << "Serial filter: " << out_aggr[0] << std::endl;
    // Reset output buffer
    for (size_t idx = 0; idx < 5; ++idx) {
        out_aggr[idx] = 0;
    }

    double pcie_time[5];

    try {
        // Dummy run to program FPGA, we don't want to measure the FPGA image reconfiguring time.
        pcie_time[0] = filter_fpga_wrapper<tvl::simd<Type, tvl::fpga, 128>>(&q, in, out_aggr, vec_elems);

        // Actual measurement can start
        std::cout << "Running HOST-Filter test with " << size << " values" << std::endl;
        pcie_time[0] = filter_fpga_wrapper<tvl::simd<Type, tvl::fpga, 128>>(&q, in, out_aggr + 0, number_CL * vec_elems);
        pcie_time[1] = filter_fpga_wrapper<tvl::simd<Type, tvl::fpga, 256>>(&q, in, out_aggr + 1, number_CL * vec_elems);
        pcie_time[2] = filter_fpga_wrapper<tvl::simd<Type, tvl::fpga, 512>>(&q, in, out_aggr + 2, number_CL * vec_elems);
        pcie_time[3] = filter_fpga_wrapper<tvl::simd<Type, tvl::fpga, 1024>>(&q, in, out_aggr + 3, number_CL * vec_elems);
        pcie_time[4] = filter_fpga_wrapper<tvl::simd<Type, tvl::fpga, 2048>>(&q, in, out_aggr + 4, number_CL * vec_elems);
    } catch (exception const& e) {
        std::cout << "Caught a synchronous SYCL exception: " << e.what() << "\n";
        std::terminate();
    }

    for (size_t idx = 0; idx < 5; ++idx) {
        std::cout << "FPGA Result [" << idx << "]:\t" << out_aggr[idx] << std::endl;
    }

    std::cout << "pcie_time_128  [us]:\t" << pcie_time[0] << "us" << std::endl;
    std::cout << "pcie_time_256  [us]:\t" << pcie_time[1] << "us" << std::endl;
    std::cout << "pcie_time_512  [us]:\t" << pcie_time[2] << "us" << std::endl;
    std::cout << "pcie_time_1024 [us]:\t" << pcie_time[3] << "us" << std::endl;
    std::cout << "pcie_time_2048 [us]:\t" << pcie_time[4] << "us" << std::endl;

    // print result
    double input_size_mb = static_cast<double>(size * sizeof(Type)) / (1024 * 1024);

    std::cout << "Size of input data: " << input_size_mb << " MiB" << std::endl;
    std::cout << "HOST-DEVICE (128 ) Throughput: " << (input_size_mb / (pcie_time[0] * 1e-6)) << " MiB/s" << std::endl;
    std::cout << "HOST-DEVICE (256 ) Throughput: " << (input_size_mb / (pcie_time[1] * 1e-6)) << " MiB/s" << std::endl;
    std::cout << "HOST-DEVICE (512 ) Throughput: " << (input_size_mb / (pcie_time[2] * 1e-6)) << " MiB/s" << std::endl;
    std::cout << "HOST-DEVICE (1024) Throughput: " << (input_size_mb / (pcie_time[3] * 1e-6)) << " MiB/s" << std::endl;
    std::cout << "HOST-DEVICE (2048) Throughput: " << (input_size_mb / (pcie_time[4] * 1e-6)) << " MiB/s" << std::endl;

    size_t res1 = filter_cpu_wrapper<tvl::simd<Type, tvl::sse>>(in, (number_CL * vec_elems));
    size_t res2 = filter_cpu_wrapper<tvl::simd<Type, tvl::avx2>>(in, (number_CL * vec_elems));
    size_t res3 = filter_cpu_wrapper<tvl::simd<Type, tvl::avx512>>(in, (number_CL * vec_elems));

    std::cout << "## Simd Testing ##" << std::endl;
    std::cout << "SSE Tput: " << (input_size_mb / (res1 * 1e-6)) << " MiB/s "
              << "(" << res1 << " us)" << std::endl;
    std::cout << "AVX2 Tput: " << (input_size_mb / (res2 * 1e-6)) << " MiB/s "
              << "(" << res2 << " us)" << std::endl;
    std::cout << "AVX512 Tput: " << (input_size_mb / (res3 * 1e-6)) << " MiB/s "
              << "(" << res3 << " us)" << std::endl;

    std::string emu_prefix;
#ifdef FPGA_EMULATOR
    emu_prefix = "emu_";
#else
    emu_prefix = "fpga_";
#endif

    std::ofstream out(emu_prefix + "fpga_measures_filter_" + std::to_string(size) + ".txt");
    out << "processor\tsize in bytes\tregister width\truntime in us\n";
    out << "fpga\t" << size * sizeof(Type) << "\t128\t" << pcie_time[0] << std::endl;
    out << "fpga\t" << size * sizeof(Type) << "\t256\t" << pcie_time[1] << std::endl;
    out << "fpga\t" << size * sizeof(Type) << "\t512\t" << pcie_time[2] << std::endl;
    out << "fpga\t" << size * sizeof(Type) << "\t1024\t" << pcie_time[3] << std::endl;
    out << "fpga\t" << size * sizeof(Type) << "\t2048\t" << pcie_time[4] << std::endl;
    out << "cpu\t" << size * sizeof(Type) << "\t128\t" << res1 << std::endl;
    out << "cpu\t" << size * sizeof(Type) << "\t256\t" << res2 << std::endl;
    out << "cpu\t" << size * sizeof(Type) << "\t512\t" << res3 << std::endl;
    out.close();

    // free USM
    sycl::free(in, q);
    sycl::free(out_aggr, q);
}