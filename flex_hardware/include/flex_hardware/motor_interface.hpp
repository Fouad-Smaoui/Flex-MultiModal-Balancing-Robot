#ifndef FLEX_HARDWARE__MOTOR_INTERFACE_HPP_
#define FLEX_HARDWARE__MOTOR_INTERFACE_HPP_

namespace flex_hardware
{

/// Abstraction over the MC33926 H-bridge driver (real firmware, V1) or the simulated
/// velocity command interface (sim). Duty is a PWM duty fraction in [-0.8, 0.8], matching
/// firmware/src/main.cpp's outputPID clamp range exactly — this interface does not
/// reinterpret or rescale the existing control surface, see
/// docs/architecture/control_architecture.md.
///
/// Deliberately a separate interface from ODriveInterface, not a mode flag on one interface:
/// PWM duty and torque command are different control surfaces, not configuration variants of
/// the same one.
class MotorInterface
{
public:
  virtual ~MotorInterface() = default;

  virtual void set_duty(double left_duty, double right_duty) = 0;
};

}  // namespace flex_hardware

#endif  // FLEX_HARDWARE__MOTOR_INTERFACE_HPP_
