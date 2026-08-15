#!/bin/bash

# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.

# This script as an example of how a custom threading layer can be built for cuDSS
# so that multi-threaded computations can be used.
# It requires an NVCC compiler and OpenMP.
#
# Please check/set the following environment variables:
#  - $CUDA_PATH = Path to your CUDA installation directory.
#  - $CUDSS_PATH = Path to your CUDSS installation directory.
#  - $OPENMP_PATH = Path to your OpenMP library installation directory (optional).
#                 If your OpenMP library is not installed in system
#                 directories,
#                 ${OPENMP_PATH}/include is expected to contain the omp.h header.
#                 ${OPENMP_PATH}/lib64 or ${OPENMP_PATH}/lib is expected to contain the OpenMP runtime library.
#
# Run (inside this directory): source ./cudss_build_mtlayer.sh

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
  then
    echo "Usage example:"
    echo "chmod +x $0; CUDA_PATH=<CUDA CTK path> OPENMP_PATH=<OpenMP path with include/ and lib/ or lib64/> ./$0.sh <gomp>"
    exit 0
elif [ "$1" == "gomp" ]
  then
    echo "Building communication layer with ${1} backend"
else
    echo "Script input arguments (for help run with -h or --help):"
    echo "Args: $@"
    exit 1
fi

if [ -z "${CUDA_PATH}" ]
then
    echo "Environment variable CUDA_PATH is not set. Please set it to point to the CUDA root directory!"
    exit 2
fi

if [ -z "${CUDSS_PATH}" ]
then
    echo "Environment variable CUDSS_PATH is not set. Please set it to point to the CUDSS installation directory (for headers)!"
    exit 2
fi

backend=$1
#backend=openmpi
#backend=nccl

if [[ "$backend" == "gomp" ]]; then
    THR_LIB_PATH=${OPENMP_PATH}
    thr_lib_name="gomp"
    thr_src_name="omp"
    thr_omp_cflag="-fopenmp"
    if [ -z "${OPENMP_PATH}" ]
    then
        echo "Environment variable OPENMP_PATH is not set. Script will expect to find"
        echo "OpenMP includes and libraries in the default system paths."
        exit 3
    fi
fi

set -x
${CUDA_PATH}/bin/nvcc -forward-unknown-to-host-compiler -shared -fPIC \
    -Wunknown-pragmas -Wall -Werror all-warnings \
    -I${CUDA_PATH}/include -I${CUDSS_PATH}/include -I${THR_LIB_PATH}/include ${thr_omp_cflag} \
    cudss_mtlayer_${thr_src_name}.cu \
    -L${THR_LIB_PATH}/lib64 -L${THR_LIB_PATH}/lib -l${thr_lib_name} \
    -o libcudss_mtlayer_${backend}.so
set +x

# To enable usage of the threading layer in cuDSS routines, please set the environment
# variable below and follow the docs for the required source code changes
# export CUDSS_THREADING_LIB=${PWD}/libcudss_threading_layer_${backend}.so

ls libcudss_mtlayer_${backend}.so
exit $?

