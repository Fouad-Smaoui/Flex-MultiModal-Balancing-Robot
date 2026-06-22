# Raw QEI tick counters from the STM32 firmware, intentionally pre-CPR-scaling so the known
# left/right CPR inconsistency (firmware/src/main.cpp) is visible rather than silently baked
# in. See docs/architecture/deployment_architecture.md.
int32 left_ticks
int32 right_ticks
