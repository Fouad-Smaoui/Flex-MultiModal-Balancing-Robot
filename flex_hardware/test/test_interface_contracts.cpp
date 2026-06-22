// Contract test: verifies each flex_hardware interface is a usable abstract base — i.e. a
// minimal mock can implement it and be driven polymorphically through the base pointer. This
// is what guarantees a future sim implementation and a future real_stm32_interface.cpp
// implementation are interchangeable behind the same interface (the actual point of the HAL,
// see docs/architecture/simulation_architecture.md "Topic Parity").

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

#include "flex_hardware/encoder_interface.hpp"
#include "flex_hardware/imu_interface.hpp"
#include "flex_hardware/motor_interface.hpp"
#include "flex_hardware/odrive_interface.hpp"
#include "flex_hardware/stm32_interface.hpp"

namespace
{

class MockIMU : public flex_hardware::IMUInterface
{
public:
  flex_hardware::Quaternion read_orientation() override { return {}; }
  flex_hardware::Vector3 read_angular_velocity() override { return {}; }
};

class MockEncoder : public flex_hardware::EncoderInterface
{
public:
  int32_t read_left_ticks() override { return left_ticks_; }
  int32_t read_right_ticks() override { return right_ticks_; }
  int32_t left_ticks_ = 0;
  int32_t right_ticks_ = 0;
};

class MockMotor : public flex_hardware::MotorInterface
{
public:
  void set_duty(double left_duty, double right_duty) override
  {
    last_left_duty_ = left_duty;
    last_right_duty_ = right_duty;
  }
  double last_left_duty_ = 0.0;
  double last_right_duty_ = 0.0;
};

class MockODrive : public flex_hardware::ODriveInterface
{
public:
  void set_torque(double left_torque_nm, double right_torque_nm) override
  {
    last_left_torque_nm_ = left_torque_nm;
    last_right_torque_nm_ = right_torque_nm;
  }
  double last_left_torque_nm_ = 0.0;
  double last_right_torque_nm_ = 0.0;
};

class MockSTM32 : public flex_hardware::STM32Interface
{
public:
  flex_hardware::Quaternion read_orientation() override { return {}; }
  flex_hardware::Vector3 read_angular_velocity() override { return {}; }
  int32_t read_left_ticks() override { return 0; }
  int32_t read_right_ticks() override { return 0; }
  void set_duty(double, double) override {}
  void set_leg_angles(double left_angle_rad, double right_angle_rad) override
  {
    last_left_angle_rad_ = left_angle_rad;
    last_right_angle_rad_ = right_angle_rad;
  }
  double read_battery_voltage() override { return 11.1; }
  double last_left_angle_rad_ = 0.0;
  double last_right_angle_rad_ = 0.0;
};

}  // namespace

TEST(FlexHardwareContracts, IMUInterfaceIsPolymorphic)
{
  std::unique_ptr<flex_hardware::IMUInterface> imu = std::make_unique<MockIMU>();
  EXPECT_NO_THROW(imu->read_orientation());
  EXPECT_NO_THROW(imu->read_angular_velocity());
}

TEST(FlexHardwareContracts, EncoderInterfaceReportsTicksIndependently)
{
  MockEncoder mock;
  mock.left_ticks_ = 100;
  mock.right_ticks_ = -42;
  std::unique_ptr<flex_hardware::EncoderInterface> encoder =
    std::make_unique<MockEncoder>(mock);
  EXPECT_EQ(encoder->read_left_ticks(), 100);
  EXPECT_EQ(encoder->read_right_ticks(), -42);
}

TEST(FlexHardwareContracts, MotorInterfaceAcceptsDutyRange)
{
  auto mock = std::make_unique<MockMotor>();
  flex_hardware::MotorInterface * motor = mock.get();
  motor->set_duty(0.8, -0.8);
  EXPECT_DOUBLE_EQ(mock->last_left_duty_, 0.8);
  EXPECT_DOUBLE_EQ(mock->last_right_duty_, -0.8);
}

TEST(FlexHardwareContracts, ODriveInterfaceIsDistinctFromMotorInterface)
{
  // MotorInterface (PWM duty) and ODriveInterface (torque) are deliberately separate
  // interfaces, not a mode flag on one — see odrive_interface.hpp. Confirm neither is
  // assignable to the other's pointer type (this is a compile-time guarantee; the
  // static_assert documents the intent explicitly).
  static_assert(
    !std::is_base_of<flex_hardware::MotorInterface, flex_hardware::ODriveInterface>::value,
    "ODriveInterface must not derive from MotorInterface");
  static_assert(
    !std::is_base_of<flex_hardware::ODriveInterface, flex_hardware::MotorInterface>::value,
    "MotorInterface must not derive from ODriveInterface");

  auto mock = std::make_unique<MockODrive>();
  mock->set_torque(1.5, -1.5);
  EXPECT_DOUBLE_EQ(mock->last_left_torque_nm_, 1.5);
  EXPECT_DOUBLE_EQ(mock->last_right_torque_nm_, -1.5);
}

TEST(FlexHardwareContracts, STM32InterfaceComposesImuEncoderMotor)
{
  std::unique_ptr<flex_hardware::STM32Interface> stm32 = std::make_unique<MockSTM32>();

  // Usable as each composed interface independently — this is what lets flex_control select
  // "sim" or "stm32" at launch time behind the same controller code.
  flex_hardware::IMUInterface * as_imu = stm32.get();
  flex_hardware::EncoderInterface * as_encoder = stm32.get();
  flex_hardware::MotorInterface * as_motor = stm32.get();
  EXPECT_NO_THROW(as_imu->read_orientation());
  EXPECT_NO_THROW(as_encoder->read_left_ticks());
  EXPECT_NO_THROW(as_motor->set_duty(0.1, 0.1));

  stm32->set_leg_angles(0.2, -0.2);
  EXPECT_GT(stm32->read_battery_voltage(), 0.0);
}
