ALL_CXXFLAGS=-Wall -std=c++20 $(CXXFLAGS) -msse -msse2 -mssse3 -msse4.1 -msse4.2 -mavx -mavx2 -mavx512f -mavx512dq -mavx512bw

BOARD= #SET_ME

# Directories
INC_DIRS := ../common/inc
LIB_DIRS :=

# Files
INCS := $(wildcard )
LIBS := pthread

emu: 
	icpx -fsycl $(ALL_CXXFLAGS) -Ilibs/include -fintelfpga -DFPGA_EMULATOR src/eval_filter_kernel.cpp -fsycl-device-code-split=off -o build/eval_filter_kernel.fpga_emu $(foreach D,$(LIB_DIRS),-L$D) $(foreach L,$(LIBS),-l$L)
	icpx -fsycl $(ALL_CXXFLAGS) -Ilibs/include -fintelfpga -DFPGA_EMULATOR src/eval_agg_kernel.cpp -fsycl-device-code-split=off -o build/eval_agg_kernel.fpga_emu $(foreach D,$(LIB_DIRS),-L$D) $(foreach L,$(LIBS),-l$L)
	icpx -fsycl $(ALL_CXXFLAGS) -Ilibs/include -fintelfpga -DFPGA_EMULATOR src/eval_acceleration.cpp -fsycl-device-code-split=off -o build/eval_acceleration.cpp.fpga_emu $(foreach D,$(LIB_DIRS),-L$D) $(foreach L,$(LIBS),-l$L)

hw_agg:
	icpx -fsycl $(ALL_CXXFLAGS) -v -Ilibs/include -fintelfpga -Xshardware -fsycl-device-code-split=off -Xsprofile -Xssave-temps -reuse-exe=build/eval_agg_kernel.fpga -Xsoutput-report-folder=eval_agg_kernel.prj -Xsboard=$(BOARD) -o build/eval_agg_kernel.fpga src/eval_agg_kernel.cpp $(foreach D,$(LIB_DIRS),-L$D) $(foreach L,$(LIBS),-l$L)

hw_filter:
	icpx -fsycl $(ALL_CXXFLAGS) -v -Ilibs/include -fintelfpga -Xshardware -fsycl-device-code-split=off -Xsprofile -Xssave-temps -reuse-exe=build/eval_filter_kernel.fpga -Xsoutput-report-folder=eval_filter_kernel.prj -Xsboard=$(BOARD) -o build/eval_filter_kernel.fpga src/eval_filter_kernel.cpp $(foreach D,$(LIB_DIRS),-L$D) $(foreach L,$(LIBS),-l$L)

hw_mt:
	icpx -fsycl $(ALL_CXXFLAGS) -v -Ilibs/include -fintelfpga -Xshardware -fsycl-device-code-split=off -Xsprofile -Xssave-temps -reuse-exe=build/eval_acceleration.cpp.fpga -Xsoutput-report-folder=eval_acceleration.cpp.prj -Xsboard=$(BOARD) -o build/eval_acceleration.cpp.fpga src/eval_acceleration.cpp $(foreach D,$(LIB_DIRS),-L$D) $(foreach L,$(LIBS),-l$L)

.PHONY: emu report hw_agg hw_filter hw_non hw_mt clean clean_hw
