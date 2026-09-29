// Vendored third-party code — TinyMPC (ADMM-based MPC solver)
//
// Source: https://github.com/TongjiSuperPower/sp_vision_25
//         path: tasks/auto_aim/planner/tinympc/
//         commit: 5b3eb17 (repository HEAD 604e119, 2025-09-29)
// Upstream: TinyMPC — https://github.com/TinyMPC/TinyMPC
//           (ICRA 2024 best paper; ADMM-based embedded MPC solver)
// License: MIT (see ../../LICENSE-TinyMPC)
//
// Vendored verbatim. Do not edit; apply patches upstream or in the build config.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

// #if defined(__linux__) || defined(__unix__)// Check if Linux
// #include <error.h>
// #define ERROR_MSG(exit_code, format, ...) error(exit_code, errno, format, ##__VA_ARGS__)

// #elif defined(__APPLE__) || defined(__MACH__) // Check if macOS
#define ERROR_MSG(exit_code, format, ...) \
        { \
        fprintf(stderr, format ": %s\n", ##__VA_ARGS__, strerror(errno)); \
        exit(exit_code); \
        }

// #else
// #error "Unsupported operating system"
// #endif

#ifdef __cplusplus
}
#endif