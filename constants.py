import math
import wpimath.units

# Drivetrain
kWheelDiameter = wpimath.units.inchesToMeters(3)
kDriveMotorReduction = (45 * 22) /(13*15)
kDriveMotorFreeSpeed = 5676 / 60 #94.6 rev/s

kMaxSpeedTheoretical = math.pi * kWheelDiameter * kDriveMotorFreeSpeed / kDriveMotorReduction #4.46 m/s
kMaxSpeed = 0.8 * kMaxSpeedTheoretical

kMaxTurnSpeed = 3 #rad/s

# Elevator
kMaxElevatorSpeed = wpimath.units.inchesToMeters(9) # m/s
kMinElevatorHeight = wpimath.units.inchesToMeters(1) # m
kMaxElevatorHeight = wpimath.units.inchesToMeters((4 * 12) + 2) # m
# Pitch diameter for the WCP 18 tooth sprocket is 1.432 inches
kElevatorSprocketDiameter = wpimath.units.inchesToMeters(1.432)
kElevatorMotorReduction = (1 / 20)

kElevatorP = 0
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

# Arm safety constraints, specified in range allowed when elevator is above n meters
# (constraintMinElevHeight, constraintMaxElevHeight): (minAngle, maxAngle)
kArmSafetyConstraints = {
    # Prevent from hitting bellypan
    (float("-inf"), 0.2): (wpimath.units.degreesToRadians(-110), wpimath.units.degreesToRadians(0)),
    
    (0.2, 0.4): (wpimath.units.degreesToRadians(-90), wpimath.units.degreesToRadians(-20)),
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

#Arm Angle Presets
kWristUprightAngle = wpimath.units.degreesToRadians(0) #rad
kWristCoralScoreAngle = wpimath.units.degreesToRadians(-30) #rad
kWristAlgaeDereef = wpimath.units.degreestoRadians(-110) #rad
kWristAlgaeGround = wpimath.units.degreesToRadian(-124) #rad

# Arm FF constants
# https://www.reca.lc/arm?armMass=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A5.97%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A38%2C%22u%22%3A%22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A25%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-90%2C%22u%22%3A%22deg%22%7D
kArmKS = 0.00 #
kArmKG = 0.47 #V
kArmKV = 0.49 #V*s/rad
kArmKA = 0.01 #V*s^2/rad

kArmP = 0.133
kArmI = 0.0002
kArmD = 0

#Source Intake
kSourceIntakeSpeed = 0.15
kSourceIntakeElevatorMaxHeight = wpimath.units.feetToMeters(1) #m
kSourceIntakeWristMinAngle = wpimath.units.degreesToRadians(-15) #rad
