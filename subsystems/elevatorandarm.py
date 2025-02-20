import rev
import math
from wpimath.geometry import Translation2d
import wpilib
import wpimath
import wpimath.geometry
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.controller import ArmFeedforward
import ntcore
import wpimath.units
import constants
import utils

def wristEncoderToAngle(encoderPosition: float) -> float:
    return encoderPosition - constants.kWristAngleOffset

def wristAngleToEncoder(angleRadians: float) -> float:
    return angleRadians + constants.kWristAngleOffset

elevatorMotor1Config = rev.SparkMaxConfig()
elevatorMotor1Config.encoder.positionConversionFactor(
    constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2
).velocityConversionFactor(
    constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2 / 60
)
elevatorMotor1Config.softLimit.forwardSoftLimit(constants.kMaxElevatorHeight).reverseSoftLimit(constants.kMinElevatorHeight).forwardSoftLimitEnabled(True).reverseSoftLimitEnabled(True)

elevatorMotor2Config = rev.SparkMaxConfig()
elevatorMotor2Config.follow(31, True)


armOuterWheelMotorConfig = rev.SparkMaxConfig()
armInnerWheelMotorConfig = rev.SparkMaxConfig()
armOuterWheelMotorConfig.inverted(True)
armOuterWheelMotorConfig.smartCurrentLimit(20)
armInnerWheelMotorConfig.smartCurrentLimit(20)

wristMotorConfig = rev.SparkMaxConfig()
wristMotorConfig.inverted(True)
wristMotorConfig.absoluteEncoder.positionConversionFactor(2*math.pi).velocityConversionFactor(2*math.pi/60).inverted(True).zeroOffset(0.0424)
#wristMotorConfig.softLimit.forwardSoftLimit(wristAngleToEncoder(constants.kMaxWristAngle)).reverseSoftLimit(wristAngleToEncoder(constants.kMinWristAngle)).forwardSoftLimitEnabled(True).reverseSoftLimitEnabled(True)

wristMotorConfig.IdleMode.kCoast

nt = ntcore.NetworkTableInstance.getDefault()
elevatorHeightTopic = nt.getFloatTopic("/ElevatorHeight").publish()
wristAngleTopic = nt.getFloatTopic("/WristAngle").publish()
wristAngleRawTopic = nt.getFloatTopic("/WristAngleRawTopic").publish()
wristAngleSetpointTopic = nt.getFloatTopic("/WristAngleSetpoint").publish()
wristSafePositionTopic = nt.getFloatTopic("/WristSafePosition").publish()
wristFFPositionTopic = nt.getFloatTopic("/WristFFTopic").publish()

# nt.getFloatTopic("/WristP").publish().set(0)
# nt.getFloatTopic("/WristI").publish().set(0)
# nt.getFloatTopic("/WristD").publish().set(0)
# wristPTopic = nt.getFloatTopic("/WristP").publish()
# wristITopic = nt.getFloatTopic("/WristI").publish()
# wristDTopic = nt.getFloatTopic("/WristD").publish()

elevatorPTopic = nt.getFloatTopic("/ElevatorP").publish()
elevatorITopic = nt.getFloatTopic("/ElevatorI").publish()
elevatorDTopic = nt.getFloatTopic("/ElevatorD").publish()

elevatorSetpointTopic = nt.getFloatTopic("/ElevatorSetpoint").publish()

CalebIsProTopic = nt.getStringTopic("/CalebIsTheGoat").publish()

class ElevatorAndArm:
    # Elevator hardware
    elevatorMotor1 = rev.SparkMax(31, rev.SparkLowLevel.MotorType.kBrushless) #Left motor from robot POV
    elevatorMotor2 = rev.SparkMax(32, rev.SparkLowLevel.MotorType.kBrushless) #Right motor from robot POV
    elevatorMotor2.configure(elevatorMotor2Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    elevatorMotor1.configure(elevatorMotor1Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    
    elevatorEncoder = elevatorMotor1.getEncoder()

    elevatorController = elevatorMotor1.getClosedLoopController()
    elevatorMotor1Config.closedLoop.pid(constants.kElevatorP, constants.kElevatorI, constants.kElevatorD)
    elevatorMotor1Config.closedLoop.outputRange(-0.6, 0.6)
    elevatorMotor1Config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    elevatorMotor1Config.closedLoop.IZone(wpimath.units.inchesToMeters(3))

    elevatorMotor1.configure(elevatorMotor1Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    # Arm hardware
    wristMotor = rev.SparkMax(41, rev.SparkLowLevel.MotorType.kBrushless)
    armOuterWheelMotor = rev.SparkMax(42, rev.SparkLowLevel.MotorType.kBrushless)
    armInnerWheelMotor = rev.SparkMax(43, rev.SparkLowLevel.MotorType.kBrushless)
    wristEncoder = wristMotor.getAbsoluteEncoder()

    # Arm PID
    wristController = wristMotor.getClosedLoopController()
    # Set PID from constants
    wristMotorConfig.closedLoop.pid(constants.kArmP, constants.kArmI, constants.kArmD)

    wristMotorConfig.closedLoop.outputRange(-0.1, 0.1)
    wristMotorConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    wristMotorConfig.closedLoop.IZone(wpimath.units.degreesToRadians(15))

    wristMotor.configure(wristMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    
    armOuterWheelMotor.configure(armOuterWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    armInnerWheelMotor.configure(armInnerWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    wristPositionSetpoint = 0.0

    def periodic(self):
        elevatorHeightTopic.set(self.elevatorEncoder.getPosition())
        wristAngleTopic.set(wristEncoderToAngle(self.wristEncoder.getPosition()))
        wristAngleRawTopic.set(self.wristEncoder.getPosition())
        wristAngleSetpointTopic.set(self.wristPositionSetpoint)

        # # wristPTopic.set(self.newP)
        # # wristITopic.set(self.newI)
        # # wristDTopic.set(self.newD)

        # # newPIDConfig = rev.SparkMaxConfig()
        # # newPIDConfig.closedLoop.pid(self.newP, self.newI, self.newD)

        # self.wristMotor.configure(newPIDConfig, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        # Recalculate the new safe wrist position based on the current elevator height
        safeWristPosition = self.get_safe_wrist_position(self.wristPositionSetpoint)
        wristSafePositionTopic.set(safeWristPosition)

        armFF = self.calculate_arm_ff()

        wristFFPositionTopic.set(armFF)

        self.wristController.setReference(wristAngleToEncoder(safeWristPosition), rev.SparkMax.ControlType.kPosition, arbFeedforward=armFF)

        pass

    # def move_elevator(self, speed: float):
    #     """
    #     Manually control the elevator speed. Positive means up, negative means down.
    #     """
    #     self.elevatorMotor1.set(speed * 0.3)
    #     # The other elevator motor is set as a follower.

    def set_elevator_pid(self, p: float, i: float, d: float):
        """
        Set the elevator PID constants.
        """
        
        elevatorMotor1Config.closedLoop.pid(p, i, d)
        self.elevatorMotor1.configure(elevatorMotor1Config, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        elevatorPTopic.set(p)
        elevatorITopic.set(i)
        elevatorDTopic.set(d)

    def set_elevator_position(self, setpoint: float):
        """
        Set the elevator position in meters. The
        setpoint is clamped to the range of heights allowed.
        """

        self.elevatorController.setReference(setpoint, rev.SparkMax.ControlType.kPosition, arbFeedforward=0)
        
        elevatorSetpointTopic.set(setpoint)
        pass


    def move_coral(self, speed: float):
        """
        Spins the squishy wheels in opposite directions to move coral. Speeds range from
        -1 to 1, where positive means intake -> reef. Speed will be limited by the top
        speed in constants.py.
        """
        self.armOuterWheelMotor.set(speed * constants.kSquishyWheelCoralSpeed)
        self.armInnerWheelMotor.set(-speed * constants.kSquishyWheelCoralSpeed)
        pass

    def move_algae(self, speed: float):
        """
        Spins the squishy wheels in the same direction to move algae. Speeds range from
        -1 to 1, where positive means robot -> processor. Speed will be limited by the
        top speed in constants.py.
        """
        self.armOuterWheelMotor.set(speed * constants.kSquishyWheelAlgaeSpeed)
        self.armInnerWheelMotor.set(speed * constants.kSquishyWheelAlgaeSpeed)
        pass

    def get_wrist_position(self) -> float:
        """
        Get the current wrist position in radians.
        """
        return wristEncoderToAngle(self.wristEncoder.getPosition())
    
    def get_elevator_position(self) -> float:
        """
        Get the current elevator position in meters.
        """
        return self.elevatorEncoder.getPosition()
    
    def get_safe_wrist_position(self, setpoint) -> float:
        """
        Takes a setpoint in radians and returns a safe position based on the 
        current elevator height. The setpoint is clamped to the range of angles allowed
        for the current elevator height. If the elevator height is outside all ranges,
        the always safe angle is returned.
        """
        # First, find what constraints apply to the current elevator height
        elevatorHeight = self.get_elevator_position()
        for heightRange, (minAngle, maxAngle) in constants.kArmSafetyConstraints.items():
            if heightRange[0] <= elevatorHeight < heightRange[1]:
                return utils.clamp(setpoint, minAngle, maxAngle)
        else:
            # If the elevator height is outside all constraint ranges, return the always safe angle
            # TODO: Replace this with a proper logging solution that goes to AdvantageScope
            print("ERROR! We should not have a gap in our arm angle limits!")
            return constants.kArmAlwaysSafeAngle
    
    def set_wrist_position(self, setpoint: float):
        """
        Set the wrist position in radians. The setpoint is clamped to the range of angles
        allowed for the current elevator height. If the elevator height is outside all ranges,
        the always safe angle is used.
        """
        self.wristPositionSetpoint = setpoint

    def calculate_arm_ff(self):
        """
        Calculates the arm feedforward based on the current wrist angle
        """

        wristAngle = self.get_wrist_position()

        # Calculate the arm feedforward
        armFF = ArmFeedforward(constants.kArmKS, constants.kArmKG, constants.kArmKV, constants.kArmKA)

        ffVoltage = armFF.calculate(wristAngle + wpimath.units.degreesToRadians(90), self.wristEncoder.getVelocity())

        return ffVoltage


    
