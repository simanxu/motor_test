# Beigo test

# Derive const parameters releated to motor ratio
```shell
const float kGearRatio = 8.f;
const float kTorqueCoefficient = 0.15f;                     // Nm/A
const float kPosDri2Jnt = 2.f * M_PI / kGearRatio;          // 1 r -> 2pi rad -> 2pi/8 rad
const float kVelDri2Jnt = 2.f * M_PI / kGearRatio;          // 1 rps -> 2pi rad/s -> 2pi/8 rad/s
const float kTauDri2Jnt = kGearRatio * kTorqueCoefficient;  // 1 A -> 8 A -> 8*ke Nm
```

