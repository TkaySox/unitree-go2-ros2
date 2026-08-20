# Vendored Feetech SDK

Python package `scservo_sdk/` is from the official Feetech Python SDK
(https://github.com/Adam-Software/Feetech-Servo-SDK), which matches the register
map and protocol used by **SCServo_Linux** C++ SMS/STS examples
(https://github.com/adityakamath/SCServo_Linux).

Protocol notes from SMS_STS.h:
- Present position @ 56 (with speed in same 4-byte block)
- Goal position @ 42, speed @ 46, acc @ 41
- 4096 ticks / revolution for STS/SMS
