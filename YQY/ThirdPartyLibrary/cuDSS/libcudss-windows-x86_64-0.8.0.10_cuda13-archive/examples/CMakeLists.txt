# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# cuDSS: Example CMake Project
#

cmake_minimum_required(VERSION 3.12)

project(cudss_examples LANGUAGES C CUDA CXX)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type: Release, Debug, etc")
endif()

set(CMAKE_C_STANDARD 99)

option(BUILD_STATIC "Building cuDSS examples with static linking" ON)
option(BUILD_STANDARD  "Building standard cuDSS examples (without MGMN mode)" ON)
option(BUILD_MGMN   "Building cuDSS examples for the MGMN mode" OFF)
option(BUILD_MT     "Building cuDSS examples for the MT mode" ON)
set(MGMN_NPROC 1 CACHE STRING  "Number of MPI processes the MGMN mode examples")
set(MPIRUN_EXECUTABLE "mpirun" CACHE STRING "MPI executable (mpirun) for launching example binaries")
set(MPIRUN_NUMPROC_FLAG "-np" CACHE STRING "MPI flag for setting the number of processes")
set(MPIRUN_EXTRA_FLAGS "--allow-run-as-root" CACHE STRING "Extra flags to be passed to the MPI executable")
option(BUILD_MGMN_WITH_OPENMPI "Enable OpenMPI backend for the MGMN mode examples" ON)
option(BUILD_MGMN_WITH_NCCL    "Enable NCCL backend for the MGMN mode examples" ON)
option(BUILD_MT_WITH_OPENMP "Enable OpenMP backend for the MT mode examples" ON)
set(MTLAYER_NAME CACHE STRING "Threading layer library name for cuDSS (if not set, read from env via CUDSS_THREADING_LIB)")
set (CUBLAS_LIB_PATH CACHE STRING "Path to cuBLAS library")
set (CUSOLVER_LIB_PATH CACHE STRING "Path to cuSOLVER library")
set (CUBLAS_INC_PATH CACHE STRING "Path to cuBLAS headers")
set (CUSOLVER_INC_PATH CACHE STRING "Path to cuSOLVER headers")

if(BUILD_MGMN AND BUILD_MGMN_WITH_OPENMPI AND
    (NOT DEFINED OPENMPI_INCLUDE_DIRECTORIES OR
     NOT DEFINED OPENMPI_LINK_DIRECTORIES))
    message(FATAL_ERROR "For building MGMN examples with OpenMPI, path to \
OpenMPI headers and libraries must be set in \
OPENMPI_INCLUDE_DIRECTORIES and OPENMPI_LINK_DIRECTORIES" )
endif()

if(BUILD_MGMN AND BUILD_MGMN_WITH_NCCL AND
    (NOT DEFINED NCCL_INCLUDE_DIRECTORIES OR
     NOT DEFINED NCCL_LINK_DIRECTORIES))
    message(FATAL_ERROR "For building MGMN examples with NCCL, path to \
NCCL headers and libraries must be set in \
NCCL_INCLUDE_DIRECTORIES and NCCL_LINK_DIRECTORIES" )
endif()

# Find cuDSS
find_package(cudss 0.8.0 REQUIRED) # COMPONENTS cudss

if (BUILD_MT AND BUILD_MT_WITH_OPENMP)
    find_package(OpenMP REQUIRED)
    if (NOT OpenMP_CXX_FOUND)
        message(FATAL_ERROR "For building MT examples with OpenMP, OpenMP \
package must be detectable by cmake")
    endif()
endif()

enable_testing()

# Building standard examples (not for MGMN mode)
if (BUILD_STANDARD)

    set(CUDSS_EXAMPLE_SOURCES
        cudss_simple.cpp
        cudss_batch.cpp
        cudss_simple_complex.cpp
        cudss_get_set.cpp
        cudss_hybrid_memory_mode.cpp
        cudss_dense_matrix_helpers.cpp
        cudss_sparse_matrix_helpers.cpp
        cudss_memory_handler.cpp
        cudss_batch_sparse_matrix_helpers.cpp
        cudss_hybrid_execute_mode.cpp
        cudss_reordering_phase.cpp
        cudss_uniform_batch.cpp
        cudss_schur_complement.cpp
        cudss_mg_mode.cpp
        cudss_residual.cpp
    )

    foreach(src ${CUDSS_EXAMPLE_SOURCES})
        set_source_files_properties(${src} PROPERTIES LANGUAGE CUDA)

        get_filename_component(name "${src}" NAME_WE)

        add_executable(${name} ${src})

        target_include_directories(${name} PUBLIC ${cudss_INCLUDE_DIR} ${CUBLAS_INC_PATH} ${CUSOLVER_INC_PATH})

        target_link_directories(${name} PUBLIC ${cudss_LIBRARY_DIR} ${CUBLAS_LIB_PATH} ${CUSOLVER_LIB_PATH})

        target_compile_options(${name} PRIVATE
            "$<$<CONFIG:Debug>:-lineinfo -g>"
            "$<$<CONFIG:RelWithDebInfo>:-lineinfo -g>"
        )

        target_link_libraries(${name} PUBLIC
            cudss
            cublas
            cusolver
        )

        add_test(${name} ${name})

        install(TARGETS ${name}
                RUNTIME DESTINATION examples
                COMPONENT Examples)

        if (BUILD_STATIC)
            add_executable(${name}_static ${src})

            target_link_directories(${name}_static PUBLIC ${cudss_LIBRARY_DIR} ${CUBLAS_LIB_PATH} ${CUSOLVER_LIB_PATH})
            target_include_directories(${name}_static PUBLIC ${cudss_INCLUDE_DIR} ${CUBLAS_INC_PATH} ${CUSOLVER_INC_PATH})
            if (WIN32)
                target_link_libraries(${name}_static PUBLIC
                    cudss
                    cublas
                    cusolver
                )
            else()
                target_link_libraries(${name}_static PUBLIC
                    cudss_static
                    cublas
                    cusolver
                )
            endif()


            add_test(${name}_static ${name}_static)

            install(TARGETS ${name}_static
                    RUNTIME DESTINATION example
                    COMPONENT Examples
            )

        endif()
    endforeach()
endif()

# Building examples for the MGMN mode

if(BUILD_MGMN)
    message(STATUS "CMake will use ${MPIRUN_EXECUTABLE} with ${MPIRUN_NUMPROC_FLAG} \
and extra flags ${MPIRUN_EXTRA_FLAGS} defined from MPIRUN_EXECUTABLE, MPIRUN_NUMPROC_FLAG \
and MPIRUN_EXTRA_FLAGS CMake variables to launch MGMN tests")
endif()

# Helper to make crun launch examples with mpirun
function(test_mpi_launcher target test Nproc)

    if(NOT (DEFINED MPIRUN_EXECUTABLE AND DEFINED MPIRUN_NUMPROC_FLAG))
        message(FATAL_ERROR "MPIRUN_EXECUTABLE and MPIRUN_NUMPROC_FLAG must be defined to use test_mpi_launcher")
    endif()

    if(NOT Nproc)
        message(FATAL_ERROR "Nproc must be defined to use test_mpi_launcher() function in cmake")
    endif()

    # This conversion is a workaround for the case when cmake passes multiple arguments to mpirin with extra ""
    string(REPLACE " " ";" MPIRUN_EXTRA_FLAGS_LIST ${MPIRUN_EXTRA_FLAGS})

    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.29)
        set_property(TARGET ${target} PROPERTY TEST_LAUNCHER ${MPIRUN_EXECUTABLE} ${MPIRUN_EXTRA_FLAGS_LIST} ${MPIRUN_NUMPROC_FLAG} ${Nproc})
    else()
        set_property(TARGET ${target} PROPERTY CROSSCOMPILING_EMULATOR ${MPIRUN_EXECUTABLE} ${MPIRUN_EXTRA_FLAGS_LIST} ${MPIRUN_NUMPROC_FLAG} ${Nproc})
    endif()

    set_property(TEST ${test} PROPERTY PROCESSORS ${Nproc})

endfunction()

if (BUILD_MGMN)
    set(CUDSS_MGMN_COMM_BACKENDS "")
    if (BUILD_MGMN_WITH_OPENMPI)
        set(CUDSS_MGMN_COMM_BACKENDS ${CUDSS_MGMN_COMM_BACKENDS} "openmpi")
    endif()
    if (BUILD_MGMN_WITH_NCCL)
        set(CUDSS_MGMN_COMM_BACKENDS ${CUDSS_MGMN_COMM_BACKENDS} "nccl")
    endif()
    message(STATUS "CUDSS_MGMN_COMM_BACKENDS = ${CUDSS_MGMN_COMM_BACKENDS}")

    if (BUILD_MGMN_WITH_OPENMPI OR BUILD_MGMN_WITH_NCCL)
        set(CUDSS_MGMN_EXAMPLE_SOURCES
            cudss_mgmn_mode.cpp
            cudss_mgmn_distributed_matrix.cpp
        )
    else()
        set(CUDSS_MGMN_EXAMPLE_SOURCES "")
    endif()

    if (BUILD_MGMN_WITH_OPENMPI OR BUILD_MGMN_WITH_NCCL)
        set(CUDSS_USE_MPI 1)
    else()
        set(CUDSS_USE_MPI 0)
    endif()

    foreach(src ${CUDSS_MGMN_EXAMPLE_SOURCES})
        set_source_files_properties(${src} PROPERTIES LANGUAGE CUDA)

        get_filename_component(name "${src}" NAME_WE)

        foreach(backend ${CUDSS_MGMN_COMM_BACKENDS})

            add_executable(${name}_${backend} ${src})

            target_include_directories(${name}_${backend} PUBLIC
                ${CUBLAS_INC_PATH} ${CUSOLVER_INC_PATH}
            )

            target_link_directories(${name}_${backend} PUBLIC
                ${cudss_LIBRARY_DIR}
                ${CUBLAS_LIB_PATH}
                ${CUSOLVER_LIB_PATH}
            )

            target_compile_options(${name}_${backend} PRIVATE
                "$<$<CONFIG:Debug>:-lineinfo -g>"
                "$<$<CONFIG:RelWithDebInfo>:-lineinfo -g>"
            )

            target_link_libraries(${name}_${backend} PUBLIC
                cudss
                cublas
                cusolver
            )

            if (CUDSS_USE_MPI)
                target_compile_options(${name}_${backend} PRIVATE -DUSE_MPI)
            endif()

            if ("${backend}" STREQUAL "openmpi")
                target_compile_options(${name}_${backend} PRIVATE -DUSE_OPENMPI)
            elseif ("${backend}" STREQUAL "nccl")
                target_compile_options(${name}_${backend} PRIVATE -DUSE_NCCL)
            endif()

            # Currently, we only support OpenMPI here for MPI but this might change
            if ("${backend}" STREQUAL "openmpi" OR CUDSS_USE_MPI)
                target_include_directories(${name}_${backend} PUBLIC
                                           ${OPENMPI_INCLUDE_DIRECTORIES}
                )
            endif()
            if ("${backend}" STREQUAL "nccl")
                target_include_directories(${name}_${backend} PUBLIC
                                           ${NCCL_INCLUDE_DIRECTORIES}
                )
            endif()

            # Note: for NCCL we still need the MPI header (for mpi.h) and library (for MPI_Init)
            if ("${backend}" STREQUAL "openmpi" OR CUDSS_USE_MPI)
                target_link_directories(${name}_${backend} PUBLIC
                                        ${OPENMPI_LINK_DIRECTORIES}
                )
                target_link_libraries(${name}_${backend} PUBLIC mpi)
            endif()
            if ("${backend}" STREQUAL "nccl")
                target_link_directories(${name}_${backend} PUBLIC
                                        ${NCCL_LINK_DIRECTORIES}
                )
                target_link_libraries(${name}_${backend} PUBLIC nccl)
            endif()

            add_test(NAME ${name}_${backend} COMMAND ${name}_${backend} ${backend} ${cudss_LIBRARY_DIR}/libcudss_commlayer_${backend}.so)

            test_mpi_launcher(${name}_${backend} ${name}_${backend} ${MGMN_NPROC})

            install(TARGETS ${name}_${backend}
                    RUNTIME
                        DESTINATION examples
                        COMPONENT ExampleTests
                    ARCHIVE
                        DESTINATION examples
                        COMPONENT ExampleTests
            )

            if (BUILD_STATIC)
                add_executable(${name}_${backend}_static ${src})

                target_include_directories(${name}_${backend}_static PUBLIC
                    ${CUBLAS_INC_PATH} ${CUSOLVER_INC_PATH}
                )

                target_link_directories(${name}_${backend}_static PUBLIC ${CUBLAS_LIB_PATH} ${CUSOLVER_LIB_PATH} )

                target_link_libraries(${name}_${backend}_static PUBLIC
                    cudss_static
                    cublas
                    cusolver
                )

                if (CUDSS_USE_MPI)
                    target_compile_options(${name}_${backend}_static PRIVATE -DUSE_MPI)
                endif()

                # Note: OpenMPI here has priority over NCCL ( we have to
                if ("${backend}" STREQUAL "openmpi")
                    target_compile_options(${name}_${backend}_static PRIVATE -DUSE_OPENMPI)
                elseif ("${backend}" STREQUAL "nccl")
                    target_compile_options(${name}_${backend}_static PRIVATE -DUSE_NCCL)
                endif()

                # Currently, we only support OpenMPI here for MPI but this might change
                if ("${backend}" STREQUAL "openmpi" OR CUDSS_USE_MPI)
                    target_include_directories(${name}_${backend}_static PUBLIC
                                               ${OPENMPI_INCLUDE_DIRECTORIES}
                    )
                endif()
                if ("${backend}" STREQUAL "nccl")
                    target_include_directories(${name}_${backend}_static PUBLIC
                                               ${NCCL_INCLUDE_DIRECTORIES}
                    )
                endif()

                # Note: for NCCL we still need the MPI header (for mpi.h) and library (for MPI_Init)
                if ("${backend}" STREQUAL "openmpi" OR CUDSS_USE_MPI)
                    target_link_directories(${name}_${backend}_static PUBLIC
                                            ${OPENMPI_LINK_DIRECTORIES}
                    )
                    target_link_libraries(${name}_${backend}_static PUBLIC mpi)
                endif()
                if ("${backend}" STREQUAL "nccl")
                    target_link_directories(${name}_${backend}_static PUBLIC
                                            ${NCCL_LINK_DIRECTORIES}
                    )
                    target_link_libraries(${name}_${backend}_static PUBLIC nccl)
                endif()

                #add_test(${name}_${backend}_static ${name}_${backend}_static)
                add_test(NAME ${name}_${backend}_static COMMAND ${name}_${backend}_static
                    ${backend} ${cudss_LIBRARY_DIR}/libcudss_commlayer_${backend}.so)

                test_mpi_launcher(${name}_${backend}_static ${name}_${backend}_static ${MGMN_NPROC})

                install(TARGETS ${name}_${backend}_static
                        RUNTIME
                            DESTINATION examples
                            COMPONENT ExampleTests
                        ARCHIVE
                            DESTINATION examples
                            COMPONENT ExampleTests
                )

            endif()

        endforeach()
    endforeach()

    # Communication layer standalone test (does not link libcudss; loads comm layer via dlopen).
    if (NOT WIN32 AND (BUILD_MGMN_WITH_OPENMPI OR BUILD_MGMN_WITH_NCCL))
        if (NOT DEFINED CUDSS_LAYER_INCLUDE_DIR)
            set(CUDSS_LAYER_INCLUDE_DIR ${cudss_INCLUDE_DIR})
        endif()
        set(COMM_LAYER_SRC test_communication_layer.cu)
        set_source_files_properties(${COMM_LAYER_SRC} PROPERTIES LANGUAGE CUDA)
        foreach(backend ${CUDSS_MGMN_COMM_BACKENDS})
            set(target_name test_communication_layer_mp_${backend})
            add_executable(${target_name} ${COMM_LAYER_SRC})
            target_compile_options(${target_name} PRIVATE -DUSE_MPI)
            if ("${backend}" STREQUAL "openmpi")
                target_compile_options(${target_name} PRIVATE -DUSE_OPENMPI)
            elseif ("${backend}" STREQUAL "nccl")
                target_compile_options(${target_name} PRIVATE -DUSE_NCCL)
            endif()
            target_include_directories(${target_name} PUBLIC
                ${CUDSS_LAYER_INCLUDE_DIR}
            )
            if ("${backend}" STREQUAL "openmpi" OR CUDSS_USE_MPI)
                target_include_directories(${target_name} PUBLIC
                    ${OPENMPI_INCLUDE_DIRECTORIES}
                )
            endif()
            if ("${backend}" STREQUAL "nccl")
                target_include_directories(${target_name} PUBLIC
                    ${NCCL_INCLUDE_DIRECTORIES}
                )
            endif()
            if ("${backend}" STREQUAL "openmpi" OR CUDSS_USE_MPI)
                target_link_directories(${target_name} PUBLIC
                    ${OPENMPI_LINK_DIRECTORIES}
                )
                target_link_libraries(${target_name} PUBLIC mpi)
            endif()
            if ("${backend}" STREQUAL "nccl")
                target_link_directories(${target_name} PUBLIC
                    ${NCCL_LINK_DIRECTORIES}
                )
                target_link_libraries(${target_name} PUBLIC nccl)
            endif()
            target_link_libraries(${target_name} PUBLIC cudart ${CMAKE_DL_LIBS})
            add_test(NAME ${target_name} COMMAND ${target_name} ${backend} ${cudss_LIBRARY_DIR}/libcudss_commlayer_${backend}${CMAKE_SHARED_LIBRARY_SUFFIX})
        endforeach()
    endif()
endif()

if (BUILD_MT)

    set(CUDSS_MT_EXAMPLE_SOURCES
        cudss_multithreaded_mode.cpp
    )

    set(CUDSS_MT_BACKENDS "")
    if (BUILD_MT_WITH_OPENMP)
        # Define the flavor of OpenMP picked up by find_package()
        if (OpenMP_CXX_LIBRARIES OR OpenMP_C_LIBRARIES) # should work on Linux
            if (OpenMP_CXX_LIBRARIES)
                string(REGEX MATCH ".*lib([A-Za-z0-9]*omp[A-Za-z0-9]*)[.](so|dll).*" dummy ${OpenMP_CXX_LIBRARIES})
            else()
                string(REGEX MATCH ".*lib([A-Za-z0-9]*omp[A-Za-z0-9]*)[.](so|dll).*" dummy ${OpenMP_C_LIBRARIES})
            endif()
            set(omp_backend_kind ${CMAKE_MATCH_1})
        else() # fallback for Windows
            set(omp_backend_kind "omp")
        endif()
        message(STATUS "omp_backend_kind = ${omp_backend_kind}")
        set(CUDSS_MT_BACKENDS ${CUDSS_MT_BACKENDS} ${omp_backend_kind})
    endif()

    message(STATUS "CUDSS_MT_BACKENDS = ${CUDSS_MT_BACKENDS}")

    foreach(src ${CUDSS_MT_EXAMPLE_SOURCES})
        set_source_files_properties(${src} PROPERTIES LANGUAGE CUDA)

        get_filename_component(name "${src}" NAME_WE)

        foreach(backend ${CUDSS_MT_BACKENDS})

            add_executable(${name}_${backend} ${src})

            target_include_directories(${name}_${backend} PUBLIC
                ${CUBLAS_INC_PATH} ${CUSOLVER_INC_PATH}
            )

            target_link_directories(${name}_${backend} PUBLIC
                ${cudss_LIBRARY_DIR}
                ${CUBLAS_LIB_PATH}
                ${CUSOLVER_LIB_PATH}
            )

            target_compile_options(${name}_${backend} PRIVATE
                "$<$<CONFIG:Debug>:-lineinfo -g>"
                "$<$<CONFIG:RelWithDebInfo>:-lineinfo -g>"
            )

            target_link_libraries(${name}_${backend} PUBLIC
                cudss
                cublas
                cusolver
            )

            if ("${backend}" MATCHES "omp")
                target_link_libraries(${name}_${backend} PUBLIC
                    OpenMP::OpenMP_CXX
                )
            endif()

            target_compile_options(${name}_${backend} PRIVATE -DUSE_OPENMP)

            # optional extra argument for the multithreaded examples, the threading layer library name
            set(extra_mt_input_arg ${cudss_LIBRARY_DIR}/libcudss_mtlayer_${backend}${CMAKE_SHARED_LIBRARY_SUFFIX})

            add_test(NAME ${name}_${backend} COMMAND ${name}_${backend} ${extra_mt_input_arg})

            install(TARGETS ${name}_${backend}
                    RUNTIME
                        DESTINATION examples
                        COMPONENT ExampleTests
                    ARCHIVE
                        DESTINATION examples
                        COMPONENT ExampleTests
            )

            if (BUILD_STATIC)
                add_executable(${name}_${backend}_static ${src})

                target_include_directories(${name}_${backend}_static PUBLIC
                    ${CUBLAS_INC_PATH} ${CUSOLVER_INC_PATH}
                )

                target_link_directories(${name}_${backend}_static PUBLIC ${CUBLAS_LIB_PATH} ${CUSOLVER_LIB_PATH})

                target_link_libraries(${name}_${backend}_static PUBLIC
                    cudss_static
                    cublas
                    cusolver
                )

                if ("${backend}" MATCHES "omp")
                    target_link_libraries(${name}_${backend}_static PUBLIC
                        OpenMP::OpenMP_CXX
                    )
                endif()

                target_compile_options(${name}_${backend}_static PRIVATE -DUSE_OPENMP)

                add_test(NAME ${name}_${backend}_static COMMAND ${name}_${backend}_static ${extra_mt_input_arg})

                install(TARGETS ${name}_${backend}_static
                        RUNTIME
                            DESTINATION examples
                            COMPONENT ExampleTests
                        ARCHIVE
                            DESTINATION examples
                            COMPONENT ExampleTests
                )

            endif()

        endforeach()
    endforeach()

    # Threading layer standalone test (does not link libcudss; loads threading layer via dlopen).
    if (NOT WIN32 AND BUILD_MT_WITH_OPENMP)
        if (NOT DEFINED CUDSS_LAYER_INCLUDE_DIR)
            set(CUDSS_LAYER_INCLUDE_DIR ${cudss_INCLUDE_DIR})
        endif()
        set(MT_LAYER_SRC test_threading_layer.cu)
        set_source_files_properties(${MT_LAYER_SRC} PROPERTIES LANGUAGE CUDA)
        foreach(backend ${CUDSS_MT_BACKENDS})
            set(target_name test_threading_layer_${backend})
            add_executable(${target_name} ${MT_LAYER_SRC})
            target_include_directories(${target_name} PUBLIC ${CUDSS_LAYER_INCLUDE_DIR})
            target_link_libraries(${target_name} PUBLIC OpenMP::OpenMP_CXX cudart ${CMAKE_DL_LIBS})
            add_test(NAME ${target_name} COMMAND ${target_name} ${backend} ${cudss_LIBRARY_DIR}/libcudss_mtlayer_${backend}${CMAKE_SHARED_LIBRARY_SUFFIX})
        endforeach()
    endif()
endif()
