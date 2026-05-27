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

#include "app_framework.hpp"
#include "message.hpp"
#include "pwm.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

class PWMESC : public LibXR::Application {
 public:
  static constexpr size_t MAX_ESC_COUNT_DEF = 6;

  struct Command {
    std::array<float, MAX_ESC_COUNT_DEF> normalized = {};
  };

  PWMESC(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
         const char* command_topic_name, uint32_t esc_count, uint32_t frequency,
         uint32_t min_pulse_us, uint32_t max_pulse_us,
         uint32_t disarmed_pulse_us, uint32_t signal_timeout_ms,
         size_t task_stack_depth)
      : esc_count_(std::clamp<uint32_t>(esc_count, 1u,
                                        static_cast<uint32_t>(MAX_ESC_COUNT_DEF))),
        frequency_hz_(frequency),
        min_pulse_us_(min_pulse_us),
        max_pulse_us_(max_pulse_us),
        disarmed_pulse_us_(disarmed_pulse_us),
        signal_timeout_ms_(signal_timeout_ms),
        command_topic_(command_topic_name, sizeof(command_), nullptr, true, true,
                       true),
        command_sub_(command_topic_, command_),
        cmd_file_(LibXR::RamFS::CreateFile("pwm_esc", CommandFunc, this)) {
    ASSERT(frequency_hz_ > 0);
    ASSERT(min_pulse_us_ <= max_pulse_us_);
    ASSERT(disarmed_pulse_us_ <= max_pulse_us_);
    ASSERT(signal_timeout_ms_ > 0);

    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    pwm_[0] = hw.template FindOrExit<LibXR::PWM>({"pwm_motor_1"});
    pwm_[1] = hw.template FindOrExit<LibXR::PWM>({"pwm_motor_2"});
    pwm_[2] = hw.template FindOrExit<LibXR::PWM>({"pwm_motor_3"});
    pwm_[3] = hw.template FindOrExit<LibXR::PWM>({"pwm_motor_4"});
    pwm_[4] = hw.template FindOrExit<LibXR::PWM>({"pwm_motor_5"});
    pwm_[5] = hw.template FindOrExit<LibXR::PWM>({"pwm_motor_6"});

    ConfigureOutputs();
    ApplyDisarmed();

    thread_.Create(this, ThreadFunc, "pwm_esc_thread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  void OnMonitor() override {}

 private:
  void ConfigureOutputs() {
    // TIM1 drives outputs 1-4, TIM5 drives outputs 5-6 on this board.
    ASSERT(pwm_[0]->SetConfig({frequency_hz_}) == LibXR::ErrorCode::OK);
    if (esc_count_ >= 5) {
      ASSERT(pwm_[4]->SetConfig({frequency_hz_}) == LibXR::ErrorCode::OK);
    }

    for (size_t i = 0; i < esc_count_; ++i) {
      ASSERT(pwm_[i]->Enable() == LibXR::ErrorCode::OK);
    }
  }

  float PulseToDuty(uint32_t pulse_us) const {
    return static_cast<float>(pulse_us) * static_cast<float>(frequency_hz_) /
           1000000.0f;
  }

  void SetPulse(size_t index, uint32_t pulse_us) {
    last_pulse_us_[index] = pulse_us;
    pwm_[index]->SetDutyCycle(PulseToDuty(pulse_us));
  }

  void ApplyThrottleCommand(const Command& command) {
    for (size_t i = 0; i < esc_count_; ++i) {
      float normalized = std::clamp(command.normalized[i], 0.0f, 1.0f);
      uint32_t pulse_us =
          min_pulse_us_ + static_cast<uint32_t>(
                              normalized *
                              static_cast<float>(max_pulse_us_ - min_pulse_us_));
      SetPulse(i, pulse_us);
    }
  }

  void ApplyDisarmed() {
    for (size_t i = 0; i < esc_count_; ++i) {
      SetPulse(i, disarmed_pulse_us_);
    }
  }

  static void ThreadFunc(PWMESC* pwm_esc) {
    while (true) {
      if (pwm_esc->command_sub_.Wait(pwm_esc->signal_timeout_ms_) ==
          LibXR::ErrorCode::OK) {
        pwm_esc->ApplyThrottleCommand(pwm_esc->command_);
      } else {
        pwm_esc->ApplyDisarmed();
      }
    }
  }

  static int CommandFunc(PWMESC* pwm_esc, int argc, char** argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show                         - Print current ESC pulses.\r\n">();
      LibXR::STDIO::Printf<
          "  set [index] [0.0-1.0]        - Set one ESC throttle for debug.\r\n">();
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "show") == 0) {
      for (size_t i = 0; i < pwm_esc->esc_count_; ++i) {
        LibXR::STDIO::Printf<"esc[%d] = %u us\r\n">(
            static_cast<int>(i + 1), pwm_esc->last_pulse_us_[i]);
      }
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "set") == 0) {
      int index = std::atoi(argv[2]);
      float value = static_cast<float>(std::atof(argv[3]));
      if (index < 1 || static_cast<size_t>(index) > pwm_esc->esc_count_) {
        LibXR::STDIO::Printf<"Error: Invalid ESC index.\r\n">();
        return -1;
      }

      pwm_esc->command_.normalized.fill(0.0f);
      pwm_esc->command_.normalized[static_cast<size_t>(index - 1)] =
          std::clamp(value, 0.0f, 1.0f);
      pwm_esc->ApplyThrottleCommand(pwm_esc->command_);
      return 0;
    }

    LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
    return -1;
  }

  size_t esc_count_ = 4;
  uint32_t frequency_hz_ = 400;
  uint32_t min_pulse_us_ = 1000;
  uint32_t max_pulse_us_ = 2000;
  uint32_t disarmed_pulse_us_ = 1000;
  uint32_t signal_timeout_ms_ = 1000;
  Command command_;
  std::array<uint32_t, MAX_ESC_COUNT_DEF> last_pulse_us_ = {
      1000, 1000, 1000, 1000, 1000, 1000};
  LibXR::Topic command_topic_;
  LibXR::Topic::SyncSubscriber<Command> command_sub_;
  std::array<LibXR::PWM*, MAX_ESC_COUNT_DEF> pwm_ = {};
  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
