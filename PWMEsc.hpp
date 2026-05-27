#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: PWM ESC output module
constructor_args:
  - command_topic_name: "pwm_esc_cmd"
  - esc_count: 4
  - frequency: 400
  - min_pulse_us: 1000
  - max_pulse_us: 2000
  - disarmed_pulse_us: 1000
  - signal_timeout_ms: 1000
  - task_stack_depth: 1024
template_args: []
required_hardware:
  - pwm_motor_1
  - pwm_motor_2
  - pwm_motor_3
  - pwm_motor_4
  - pwm_motor_5
  - pwm_motor_6
  - ramfs
depends: []
=== END MANIFEST === */
// clang-format on

// Canonical include path after renaming the module to PWMESC.
#include "PWMEsc.hpp"
