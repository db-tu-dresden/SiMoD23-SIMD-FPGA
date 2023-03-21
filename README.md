# Simplicity done right for SIMDified query processing on CPU and FPGA

This is the corresponding prototype to our SiMoD 2023 submission showing a simple but effective solution idea to port SIMDified query processing code to Intel FPGA cards for acceleration. The main advantage of our approach is that it seamlessly integrates with existing SIMD abstraction libraries that are originally developed to overcome SIMD heterogeneity on x86-processors. Moreover, our approach can be straightforwardly implemented in C++ without the necessity of complex FPGA-specific programming. Our initial results are very promising, demonstrating a novel approach to comprehensively integrate Intel FPGAs into the prevailing SIMDified processing on the CPU with reasonable effort.

---
### Prototype description

build/ (folder for build files)

libs/ (folder contains SIMD abstraction library TVL with an FPGA backend)

src/ (folder contains evaluation code including SIMD kernels using TVL)

Makefile 

---
### SIMD abstraction library (TVL)

This prototype contains only the relevant primitives for the evaluated SIMD kernels.
The actual TVL is to be generated from https://github.com/db-tu-dresden/TVLGen.

For the full library, checkout the generator:
    > `git clone --recurse-submodules git@github.com:db-tu-dresden/TVL.git`

And run it with the following command:
    > `python3 main.py --targets fpga $(LANG=en;lscpu | grep -i flags | tr ' ' '\n' | egrep -v '^Flags:|^$' | sort -d | tr '\n' ' ') --no-concepts`
    
---    
### SIMD kernels
As described in our paper, we conducted two core evaluations: (i) FPGA performance and (ii) acceleration.

#### FPGA performance
To demonstrate the viability of our approach with templated C++ kernels, we developed two examples. One is a very basic aggregation SIMD-kernel (see `src/eval_agg_kernel.cpp`), i.e., streaming over the data and adding every element. The second is a filter-count SIMD-kernel (see `src/eval_filter_kernel.cpp`), which counts the amount of values in a given range.

#### Aceeleration evaluation
Our next experiment evaluates the achieved bandwidth for concurrent calculations on both the host CPU and the FPGA (see `eval/eval_acceleration.cpp`). Here, we vary the amount of concurrent threads on the host processor and additionally dispatch the same SIMD-kernel to the FPGA. The host code is executed using \SI{512}{\bit} sized AVX512 registers and the FPGA kernel is also parameterized to use \SI{512}{\bit} for its register width.

---
### Compile and Execute
Possible with the help of Intel DevCloud for oneAPI (https://devcloud.intel.com/oneapi/). Access is free of charge and corresponding `Makefile` is available. Environment variables may still need to be set. 
