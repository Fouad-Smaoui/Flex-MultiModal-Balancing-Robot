#ifndef FLEX_HARDWARE__IMU_INTERFACE_HPP_
#define FLEX_HARDWARE__IMU_INTERFACE_HPP_

namespace flex_hardware
{

struct Quaternion
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Vector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// Abstraction over the MPU6050's on-chip DMP fusion output (real firmware) or Gazebo's IMU
/// sensor plugin (sim) — see docs/architecture/control_architecture.md. No filtering happens
/// here; this interface only exposes whatever orientation/rate the underlying source provides.
class IMUInterface
{
public:
  virtual ~IMUInterface() = default;

  virtual Quaternion read_orientation() = 0;
  virtual Vector3 read_angular_velocity() = 0;
};

}  // namespace flex_hardware

#endif  // FLEX_HARDWARE__IMU_INTERFACE_HPP_
