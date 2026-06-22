#ifndef FLEX_HARDWARE__ENCODER_INTERFACE_HPP_
#define FLEX_HARDWARE__ENCODER_INTERFACE_HPP_

#include <cstdint>

namespace flex_hardware
{

/// Abstraction over the two QEI wheel encoders (real firmware) or the simulated joint state
/// position interfaces (sim). Ticks are intentionally raw/pre-CPR-scaling — see
/// docs/architecture/deployment_architecture.md "Known Issues" for why the left/right CPR
/// values are not assumed equal here.
class EncoderInterface
{
public:
  virtual ~EncoderInterface() = default;

  virtual int32_t read_left_ticks() = 0;
  virtual int32_t read_right_ticks() = 0;
};

}  // namespace flex_hardware

#endif  // FLEX_HARDWARE__ENCODER_INTERFACE_HPP_
