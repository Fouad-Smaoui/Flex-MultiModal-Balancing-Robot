#ifndef FLEX_HARDWARE__ODRIVE_INTERFACE_HPP_
#define FLEX_HARDWARE__ODRIVE_INTERFACE_HPP_

namespace flex_hardware
{

/// Abstraction over the unbuilt V2 ODrive motor stage, torque-control mode — matches the
/// existing odrive/ODrive_Configuration_Script.txt CONTROL_MODE_TORQUE_CONTROL setting and
/// avoids stacking ODrive's internal velocity loop under the existing balance PID. See
/// docs/architecture/control_architecture.md for the full rationale. Designed only — no
/// real implementation exists yet (no V2 hardware access); see
/// docs/architecture/deployment_architecture.md for status.
class ODriveInterface
{
public:
  virtual ~ODriveInterface() = default;

  virtual void set_torque(double left_torque_nm, double right_torque_nm) = 0;
};

}  // namespace flex_hardware

#endif  // FLEX_HARDWARE__ODRIVE_INTERFACE_HPP_
