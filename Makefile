# Path to CUDA (override with: make CUDA_PATH=/path/to/cuda)
CUDA_PATH ?= /usr

# Compilers
HOST_COMPILER = g++
NVCC          = $(CUDA_PATH)/bin/nvcc -ccbin $(HOST_COMPILER)

# Debug flags (enable with: make DEBUG=1)
ifeq ($(DEBUG),1)
    NVCC_DBG = -g -G
else
    NVCC_DBG =
endif

# NVCC flags
NVCCFLAGS = $(NVCC_DBG) -m64

# GPU architecture (sm_75 for Turing / RTX 20xx)
GENCODE_FLAGS = -gencode arch=compute_75,code=sm_75

# Sources
SRCS = main.cu
INCS = vec3.h ray.h hitable.h hitable_list.h sphere.h camera.h material.h

# Targets
all: cudart

cudart: cudart.o
	$(NVCC) $(NVCCFLAGS) $(GENCODE_FLAGS) -o $@ $^

cudart.o: $(SRCS) $(INCS)
	$(NVCC) $(NVCCFLAGS) $(GENCODE_FLAGS) -c main.cu -o $@

out.ppm: cudart
	./cudart > out.ppm

out.jpg: out.ppm
	ppmtojpeg out.ppm > out.jpg

clean:
	rm -f cudart cudart.o out.ppm out.jpg

