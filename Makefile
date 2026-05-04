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
NVCCFLAGS = $(NVCC_DBG) -m64 -std=c++14

# GPU architecture (sm_75 for Turing / RTX 20xx)
GENCODE_FLAGS = -gencode arch=compute_75,code=sm_75

ifeq ($(USE_SFML),1)
    PKG_CONFIG := $(shell command -v pkg-config 2>/dev/null)
    ifeq ($(PKG_CONFIG),)
        $(error USE_SFML=1 requires pkg-config. Install pkg-config and SFML dev packages first)
    endif
    SFML_EXISTS := $(shell pkg-config --exists sfml-graphics && echo yes || echo no)
    ifneq ($(SFML_EXISTS),yes)
        $(error SFML headers/libs not found. Install libsfml-dev (Ubuntu/Debian) or sfml (Homebrew))
    endif
    SFML_CFLAGS := $(shell pkg-config --cflags sfml-graphics)
    SFML_LIBS   := $(shell pkg-config --libs sfml-graphics)
    NVCCFLAGS   += -DUSE_SFML $(SFML_CFLAGS)
endif

# Sources
SRCS = main.cu
INCS = vec3.h ray.h hitable.h hitable_list.h sphere.h camera.h material.h

# Targets
all: cudart

cudart: cudart.o
	$(NVCC) $(NVCCFLAGS) $(GENCODE_FLAGS) -o $@ $^ $(SFML_LIBS)

cudart.o: $(SRCS) $(INCS)
	$(NVCC) $(NVCCFLAGS) $(GENCODE_FLAGS) -c main.cu -o $@

out.ppm: cudart
	./cudart > out.ppm

out.jpg: out.ppm
	ppmtojpeg out.ppm > out.jpg

clean:
	rm -f cudart cudart.o out.ppm out.jpg

