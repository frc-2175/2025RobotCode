import math
import wpimath.units

# Drivetrain
kWheelDiameter = wpimath.units.inchesToMeters(3)
kDriveMotorReduction = (45 * 22) /(13*15)
kDriveMotorFreeSpeed = 5676 / 60 #94.6 rev/s

kMaxSpeedTheoretical = math.pi * kWheelDiameter * kDriveMotorFreeSpeed / kDriveMotorReduction #4.46 m/s
kMaxSpeed = 0.8 * kMaxSpeedTheoretical

# Elevator
kMaxElevatorSpeed = wpimath.units.inchesToMeters(9) # m/s
kMinElevatorHeight = 0
kMaxElevatorHeight = wpimath.units.inchesToMeters(4 * 12) # m

# Arm
kMaxArmSpeed = wpimath.units.degreesToRadians(45) # rad/s
kMinWristAngle = wpimath.units.degreesToRadians(-90)
kMaxWristAngle = wpimath.units.degreesToRadians(0)
kWristMotorReduction = 1 / 20

# Source intake
kSquishyWheelCoralSpeed = 0.4
kSquishyWheelAlgaeSpeed = 0.6
