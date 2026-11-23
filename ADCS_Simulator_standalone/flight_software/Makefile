# Makefile -- ADCS_Software_v1 standalone package
#
# Needs only a C99 compiler and its libc/libm.  No MATLAB installation.
#
#   make lib                         x86-64 host, static library
#   make selftest                    x86-64 host, smoke-test executable
#   make lib CC=arm-none-eabi-gcc PORTABLE_SSE2=1 \
#        TARGET_CFLAGS="-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard"
#
# PORTABLE_SSE2=1 is required for every non-x86 target (see port/sse2).

ifeq ($(origin CC),default)
CC = gcc
endif
AR            ?= ar
TARGET_CFLAGS ?=
PORTABLE_SSE2 ?= 0
BUILD         ?= build

# Defines carried over from the generated ADCS_Software_v1.mk, minus the
# desktop-only ones (MAT_FILE logging, HAVESTDIO, rt_main's RT/USE_RTMODEL).
DEFINES = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DONESTEPFCN=1 -DTERMFCN=1 \
          -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0 -DNUMST=1 \
          -DNCSTATES=0 -DTID01EQ=0 -DMODEL=ADCS_Software_v1 -DMAT_FILE=0

# port/ comes first so its rt_logging.h is the one found.
INCLUDES = -Iport -Iinclude -Imatlab_rt
ifeq ($(PORTABLE_SSE2),1)
INCLUDES := -Iport/sse2 $(INCLUDES)
else
TARGET_CFLAGS += -msse2
endif

# -fno-predictive-commoning is from the generated makefile; keep it.
CFLAGS ?= -O2 -fno-predictive-commoning
CFLAGS += -std=c99 -Wall -Wno-unused-variable -Wno-unused-but-set-variable \
          -Wno-unused-function $(TARGET_CFLAGS) $(DEFINES) $(INCLUDES)

SRCS = src/ADCS_Software_v1.c src/ADCS_Software_v1_data.c \
       src/rt_nonfinite.c src/rtGetInf.c src/rtGetNaN.c
OBJS = $(patsubst src/%.c,$(BUILD)/%.o,$(SRCS))

.PHONY: all lib selftest dll clean
all: lib

lib: $(BUILD)/libadcs_software_v1.a

$(BUILD)/libadcs_software_v1.a: $(OBJS)
	$(AR) rcs $@ $^

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

selftest: $(BUILD)/host_selftest$(EXE)

$(BUILD)/host_selftest$(EXE): example/host_selftest.c $(BUILD)/libadcs_software_v1.a
	$(CC) $(CFLAGS) $< -L$(BUILD) -ladcs_software_v1 -lm -o $@

# Basilisk simulator DLL (x86-64 Windows host only, MinGW-w64 gcc).
# Same sources as the flight library plus optional/adcs_basilisk_wrapper.c.
# --export-all-symbols is required: the Python side reads the gains and the
# U / Y / B structs as exported DATA symbols through ctypes.
#   make dll
#   set ADCS_LIBRARY=<this folder>\build\ADCS_Software_v1.dll   (then run the scenario)
dll: $(BUILD)/ADCS_Software_v1.dll

$(BUILD)/adcs_basilisk_wrapper.o: optional/adcs_basilisk_wrapper.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/ADCS_Software_v1.dll: $(OBJS) $(BUILD)/adcs_basilisk_wrapper.o
	$(CC) -shared -m64 -static-libgcc $^ -o $@ -Wl,--export-all-symbols

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
