# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

@page dwx_hello_world_sample Hello World

The Hello World sample shows how to initialize the Driveworks SDK context and
how to access GPU properties. This sample prints the DriveWorks version and
GPU properties.

The sample is called:
`./sample_hello_world`

## Example

The following shows typical output from the sample.

    *************************************************
    Welcome to Driveworks SDK
    Initialize DriveWorks SDK v0.3.131-experimental
    Experimental Debug build with GNU 4.8.4 from heads/dev/mxhani/core/gpu-0-gc470664
    SDK: no resources mounted
    SDK: GL not available. Running with restricted functionality.
    Context of Driveworks SDK successfully initialized.
    Version: 0.3.131
    GPU devices detected: 1
    ----------------------------------------------
    Device: 0, 
    CUDA Driver Version / Runtime Version : 8.0 / 8.0
    CUDA Capability Major/Minor version number: 3.0
    Total amount of global memory in MBytes:977.062
    Memory Clock rate Ghz: 891000
    Memory Bus Width bits: 128
    L2 Cache Size: 262144
    Maximum 1D Texture Dimension Size (x): 65536
    Maximum 2D Texture Dimension Size (x,y): 65536, 65536
    Maximum 3D Texture Dimension Size (x,y,z): 4096, 4096, 4096
    Maximum Layered 1D Texture Size, (x): 16384 num: 2048
    Maximum Layered 2D Texture Size, (x,y): 16384, 16384 num: 2048
    Total amount of constant memory bytes: 65536
    Total amount of shared memory per block bytes: 49152
    Total number of registers available per block: 65536
    Warp size: 32
    Maximum number of threads per multiprocessor: 2048
    Maximum number of threads per block: 1024
    Max dimension size of a thread block (x,y,z): 1024,1024,64
    Max dimension size of a grid size (x,y,z): 2147483647,65535,65535
    Maximum memory pitch bytes: 2147483647
    Texture alignment bytes: 512
    Concurrent copy and kernel execution: Yes, copy engines num: 1
    Run time limit on kernels: Yes
    Integrated GPU sharing Host Memory: No
    Support host page-locked memory mapping: Yes
    Alignment requirement for Surfaces: Yes
    Device has ECC support: Disabled
    Device supports Unified Addressing (UVA): Yes
    Device PCI Domain ID: 0, Device PCI Bus ID: 1, Device PCI location ID: 0
    Compute Mode: Default (multiple host threads can use ::cudaSetDevice() with device simultaneously)
    Driveworks SDK released
    Happy autonomous driving!