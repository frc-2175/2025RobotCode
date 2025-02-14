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
kMinElevatorHeight = wpimath.units.inchesToMeters(1) # m
kMaxElevatorHeight = wpimath.units.inchesToMeters(4 * 12 - 0.2) # m
# Pitch diameter for the WCP 18 tooth sprocket is 1.432 inches
kElevatorSprocketDiameter = wpimath.units.inchesToMeters(1.432)
kElevatorMotorReduction = (1 / 20)

# Arm safety constraints, specified in range allowed when elevator is above n meters
# (constraintMinElevHeight, constraintMaxElevHeight): (minAngle, maxAngle)
kArmSafetyConstraints = {
    # Prevent from hitting bellypan
    (float("-inf"), 0.5): (wpimath.units.degreesToRadians(-90), wpimath.units.degreesToRadians(0)),
    # Unrestricted middle range
    (0.5, 1.0): (wpimath.units.degreesToRadians(-180), wpimath.units.degreesToRadians(0)),
    # Prevent from hitting top elevator bar
    (1.0, float("inf")): (wpimath.units.degreesToRadians(-180), wpimath.units.degreesToRadians(-20)),
}

kArmAlwaysSafeAngle = wpimath.units.degreesToRadians(-20) # rad

# Arm
kMaxArmSpeed = wpimath.units.degreesToRadians(45) # rad/s
kWristAngleOffset = 3 * math.pi / 2
kMinWristAngle = -2.36 #rad
kMaxWristAngle = -0.7 #rad
kWristMotorReduction = (1 / 20) * (18 / 22)
kSquishyWheelCoralSpeed = 0.2
kSquishyWheelAlgaeSpeed = 0.4

# Arm FF constants
# https://www.reca.lc/arm?armMass=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A5.97%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A38%2C%22u%22%3A%22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A25%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-90%2C%22u%22%3A%22deg%22%7D
kArmKS = 0.00 #
kArmKG = 0.27 #V
kArmKF = 0.39 #V*s/rad
kArmKA = 0.00 #V*s^2/rad

#Source Intake
kSourceIntakeSpeed = 0.15
