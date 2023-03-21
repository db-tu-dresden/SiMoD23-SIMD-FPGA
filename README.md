# SiMoD23-SIMD-FPGA
Simplicity done right for SIMDified query processing on CPU and FPGA

### Prototype description

build/ (folder for build files)
libs/ (SIMD abstraction library TVL)
src/ (evaluation SIMD kernels)
Makefile 

### SIMD abstraction library (TVL)

This prototype contains only the relevant primitives for the evaluated SIMD kernels.
The actual TVL is to be generated from https://github.com/db-tu-dresden/TVLGen.

For the full library, checkout the generator:
    > git clone --recurse-submodules git@github.com:db-tu-dresden/TVL.git

And run it with the following command:
    > python3 main.py --targets fpga $(LANG=en;lscpu | grep -i flags | tr ' ' '\n' | egrep -v '^Flags:|^$' | sort -d | tr '\n' ' ') --no-concepts
    
    
### SIMD kernels
As described in our paper, we conducted two core evaluation: (i) FPGA performance and (ii) acceleration. 
