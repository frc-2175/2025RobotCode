import math
import wpimath.units

kWheelDiameter = wpimath.units.inchesToMeters(3)
kDriveMotorReduction = (45 * 22) /(13*15)
kDriveMotorFreeSpeed = 5676 / 60 #94.6 rev/s

kMaxSpeedTheoretical = math.pi * kWheelDiameter * kDriveMotorFreeSpeed / kDriveMotorReduction #4.46 m/s
kMaxSpeed = 0.8 * kMaxSpeedTheoretical