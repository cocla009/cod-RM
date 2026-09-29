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


// Default settings
#define TINY_DEFAULT_ABS_PRI_TOL        (1e-03)
#define TINY_DEFAULT_ABS_DUA_TOL        (1e-03)
#define TINY_DEFAULT_MAX_ITER           (1000)
#define TINY_DEFAULT_CHECK_TERMINATION  (1)
#define TINY_DEFAULT_EN_STATE_BOUND     (1)
#define TINY_DEFAULT_EN_INPUT_BOUND     (1)
#define TINY_DEFAULT_EN_STATE_SOC       (0)
#define TINY_DEFAULT_EN_INPUT_SOC       (0)
#define TINY_DEFAULT_EN_STATE_LINEAR    (0)
#define TINY_DEFAULT_EN_INPUT_LINEAR    (0)
