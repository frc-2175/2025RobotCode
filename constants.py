import math

import wpimath.geometry
import wpimath.units

kCoralMode = 0
kAlgaeMode = 1

# Drivetrain
kWheelDiameter = wpimath.units.inchesToMeters(2.96) # empirically measured with calipers
kDriveMotorReduction = 4.71 # From REV MAXSwerve docs
kDriveMotorFreeSpeed = 5676 / 60 #94.6 rev/s
kWheelDistanceFromCenter = wpimath.units.inchesToMeters(12.375)

kMaxSpeedTheoretical = math.pi * kWheelDiameter * kDriveMotorFreeSpeed / kDriveMotorReduction #4.46 m/s
kMaxSpeed = 0.9 * kMaxSpeedTheoretical
# kMaxSpeed = 0.5 * kMaxSpeedTheoretical

kMaxTurnSpeed = 2 * math.pi #rad/s

kDirectionSlewRate = 1.2 * kMaxTurnSpeed # rad/s
kSpeedSlewRate = 1.8 * kMaxSpeed # m/s
kRotationSlewRate = 2.0 * kMaxTurnSpeed # m/s

kHeadingControllerP = 1 / wpimath.units.degreesToRadians(15)
kHeadingControllerI = 0
kHeadingControllerD = 0

kChoreoTranslationP = 1 / wpimath.units.inchesToMeters(20)
kChoreoTranslationI = 0
kChoreoTranslationD = 1 / wpimath.units.feetToMeters(20)
kChoreoRotationP = 1 / wpimath.units.degreesToRadians(30)
kChoreoRotationI = 0
kChoreoRotationD = 1 / wpimath.units.degreesToRadians(75)

# Elevator
kMinElevatorHeight = wpimath.units.inchesToMeters(0.1) # m
kMaxElevatorHeight = wpimath.units.inchesToMeters((4 * 12) + 2) # m
# Pitch diameter for the WCP 18 tooth sprocket is 1.432 inches
kElevatorSprocketDiameter = wpimath.units.inchesToMeters(1.432)
kElevatorMotorReduction = (1 / 20)

kElevatorCurrentLimit = 40

kElevatorP = 7.5
kElevatorI = 0
kElevatorD = 0

# Elevator FF constants
# https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A12.184%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A20%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.432%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A46%2C%22u%22%3A%22in%22%7D
# Do we subtract the spring force from the elevator FF? Or just half of it because of pullies?
# We subtract half the spring force for calculation of the kG, but not the kV or kA
kElevatorKS = 0.00 #V
kElevatorKG = 0.09 #V
kElevatorKV = 10.72 #V*m/s
kElevatorKA = 0.01 #V*m^2/s^2

#Elevator Presets
kElevatorL1 = wpimath.units.inchesToMeters(28)
kElevatorL2 = wpimath.units.inchesToMeters(32.5)
kElevatorL3 = wpimath.units.inchesToMeters(48.5)
kElevatorL4 = wpimath.units.inchesToMeters(73)

kElevatorAlgaeGround = wpimath.units.inchesToMeters(14)
kElevatorAlgaeLow = wpimath.units.inchesToMeters(42)
kElevatorAlgaeHigh = wpimath.units.inchesToMeters(46)
kElevatorBaseHeight = wpimath.units.inchesToMeters(6)
# Arm safety constraints, specified in range allowed when elevator is above n meters
# (constraintMinElevHeight, constraintMaxElevHeight): (minAngle, maxAngle)
kArmSafetyConstraints = {
    # Prevent from hitting bellypan
    (float("-inf"), 0.2): (wpimath.units.degreesToRadians(-110), wpimath.units.degreesToRadians(0)),
    
    (0.2, 0.4): (wpimath.units.degreesToRadians(-130), wpimath.units.degreesToRadians(-20)),
    # Unrestricted middle range
    (0.4, float("inf")): (wpimath.units.degreesToRadians(-135), wpimath.units.degreesToRadians(-20)),
    # Prevent from hitting top elevator bar
    #(1.0, float("inf")): (wpimath.units.degreesToRadians(-180), wpimath.units.degreesToRadians(-20)),
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
kArmHeightInCarriage = wpimath.units.inchesToMeters(5.5)
kArmCoralRadius = wpimath.units.inchesToMeters(16)
kArmAlgaeRadius = wpimath.units.inchesToMeters(21.5)
kWristNudgeAmount = wpimath.units.degreesToRadians(45)
kArmSafeToDoCoralPositionAngle = wpimath.units.degreesToRadians(-20)

#Arm Angle Presets
kWristUprightAngle = wpimath.units.degreesToRadians(-5) #rad
kWristCoralScoreAngle = wpimath.units.degreesToRadians(-30) #rad
kWristHighCoralScoreAngle = wpimath.units.degreesToRadians(-35)
kWristAlgaeDereef = wpimath.units.degreesToRadians(-110) #rad
kWristAlgaeGround = wpimath.units.degreesToRadians(-130) #rad

# Arm wheel constants
kArmWheelReduction = 5.0
kArmWheelDiameter = wpimath.units.inchesToMeters(4)
kCoralPositionWhenStoppedDetecting = wpimath.units.inchesToMeters(7)

# Arm FF constants
# https://www.reca.lc/arm?armMass=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A5.97%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A38%2C%22u%22%3A%22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A25%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-90%2C%22u%22%3A%22deg%22%7D
kArmKS = 0.00 #
kArmKG = 0.47 #V
kArmKV = 0.49 #V*s/rad
kArmKA = 0.01 #V*s^2/rad

kArmP = 0.3
kArmI = 0.0004
kArmD = 0

#Source Intake
kSourceIntakeSpeed = 0.15
kSourceIntakeElevatorMaxHeight = wpimath.units.feetToMeters(1) #m
kSourceIntakeWristMinAngle = wpimath.units.degreesToRadians(-15) #rad

# Hanger
kHangerMinAngle = -math.pi/2 #rad
kHangerMaxAngle = math.pi/2 #rad
kHangerMotorReduction = (1 / 3) * (1 / 9) * (1 / 9)

# Camera coordinates on robot
kRobotToCam = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(
      wpimath.units.inchesToMeters(9),
      wpimath.units.inchesToMeters(-9.25),
      wpimath.units.inchesToMeters(22.75),
    ),
    wpimath.geometry.Rotation3d.fromDegrees(0.0, 0.0, 0.0),
)
