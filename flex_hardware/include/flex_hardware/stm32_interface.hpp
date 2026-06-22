#ifndef FLEX_HARDWARE__STM32_INTERFACE_HPP_
#define FLEX_HARDWARE__STM32_INTERFACE_HPP_

#include "flex_hardware/encoder_interface.hpp"
#include "flex_hardware/imu_interface.hpp"
#include "flex_hardware/motor_interface.hpp"

namespace flex_hardware
{

/// Real-hardware boundary: composes IMU, encoder, and motor access over the future serial
/// bridge to the STM32 firmware (docs/architecture/deployment_architecture.md). The bridge
/// only taps the firmware's EXISTING sensor-in/PWM-out boundary — it does not intercept or
/// relay the balance loop's internal PID computation. Designed only — no real implementation
/// exists yet (no hardware access); see deployment_architecture.md for status and the
/// micro-ROS-vs-serial-bridge-vs-CAN decision.
class STM32Interface : public IMUInterface, public EncoderInterface, public MotorInterface
{
public:
  virtual ~STM32Interface() = default;

  /// Drives servogauche/servodroit. angle_rad is in the same range as the firmware's
  /// calibrated servo*posbas/servo*poshaute setpoints.
  virtual void set_leg_angles(double left_angle_rad, double right_angle_rad) = 0;

  /// Battery voltage in volts, read from the existing current-sense ADC averaging code
  /// (telemetry only — does not enforce any current limit).
  virtual double read_battery_voltage() = 0;
};

}  // namespace flex_hardware

#endif  // FLEX_HARDWARE__STM32_INTERFACE_HPP_
