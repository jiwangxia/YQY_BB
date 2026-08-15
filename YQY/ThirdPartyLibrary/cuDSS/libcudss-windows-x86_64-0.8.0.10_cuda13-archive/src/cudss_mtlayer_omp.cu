/*
 * Copyright (c) 2025-2026, NVIDIA CORPORATION & AFFILIATES
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "omp.h"
#include "cudss.h"
#include <library_types.h>

#include <stdio.h>

extern "C" {

int cudssGetMaxThreads()
{
    return omp_get_max_threads();
}

void cudssParallelFor(int nthr_requested, int ntasks, void *ctx, cudss_thr_func_t f)
{
    if (nthr_requested < 0) return;
    if (nthr_requested == 1) {
        for (int task = 0; task < ntasks; task++) {
            f(task, ctx);
        }
        return;
    }
    if (nthr_requested) {
        #pragma omp parallel for num_threads(nthr_requested)
        for (int task = 0; task < ntasks; task++) {
            f(task, ctx);
        }
    } else {
        #pragma omp parallel for
        for (int task = 0; task < ntasks; task++) {
            f(task, ctx);
        }
    }
}

int cudssThreadingGetProperty(libraryPropertyType propertyType, int *value) {

    if (!value) {
        printf("Invalid output value pointer");
        return 1;
    }

    // These are the publicly available properties.
    switch (propertyType) {
    case MAJOR_VERSION:
        *value = CUDSS_VERSION_MAJOR;
        break;
    case MINOR_VERSION:
        *value = CUDSS_VERSION_MINOR;
        break;
    case PATCH_LEVEL:
        *value = CUDSS_VERSION_PATCH;
        break;
    default:
        printf("Value of propertyType = %d is not supported", static_cast<int>(propertyType));
        return 2;
    }

    return 0;
}

/*
 * Parallel threading service API wrapper binding table (imported by cuDSS).
 * The exposed C symbol must be named as "cudssThreadingInterface".
 */
cudssThreadingInterface_t cudssThreadingInterface = {
    cudssGetMaxThreads,
    cudssParallelFor,
    cudssThreadingGetProperty
};

} // extern "C"
