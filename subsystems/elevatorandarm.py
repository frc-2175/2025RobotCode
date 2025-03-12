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
from wpimath.filter import SlewRateLimiter
import ntutil

def wristEncoderToAngle(encoderPosition: float) -> float:
    return encoderPosition - constants.kWristAngleOffset

def wristAngleToEncoder(angleRadians: float) -> float:
    return angleRadians + constants.kWristAngleOffset

class ElevatorAndArm:
    # Elevator hardware
    elevatorMotor1 = rev.SparkMax(31, rev.SparkLowLevel.MotorType.kBrushless) #Left motor from robot POV
    elevatorMotor2 = rev.SparkMax(32, rev.SparkLowLevel.MotorType.kBrushless) #Right motor from robot POV

    elevatorEncoder = elevatorMotor1.getEncoder()
    elevatorController = elevatorMotor1.getClosedLoopController()

    elevatorMotor1Config = rev.SparkMaxConfig()
    (
        elevatorMotor1Config
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(constants.kElevatorCurrentLimit)
    )
    (
        elevatorMotor1Config
            .encoder
                .positionConversionFactor(constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2)
                .velocityConversionFactor(constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2 / 60)
    )
    (
        elevatorMotor1Config
            .softLimit
                .forwardSoftLimit(constants.kMaxElevatorHeight)
                .forwardSoftLimitEnabled(True)
                .reverseSoftLimit(constants.kMinElevatorHeight)
                .reverseSoftLimitEnabled(True)
    )
    (
        elevatorMotor1Config
            .closedLoop
                .pid(constants.kElevatorP, constants.kElevatorI, constants.kElevatorD)
                .outputRange(-0.8, 0.8)
                .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .IZone(wpimath.units.inchesToMeters(3))
    )
    elevatorMotor1.configure(elevatorMotor1Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    
    elevatorMotor2Config = rev.SparkMaxConfig()
    (
        elevatorMotor2Config
            .smartCurrentLimit(constants.kElevatorCurrentLimit)
            .follow(31, True)
    )
    elevatorMotor2.configure(elevatorMotor2Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    # Elevator telemetry
    elevatorHeightTopic = ntutil.getFloatTopic("/Elevator/Height")
    elevatorPTopic = ntutil.getFloatTopic("/Elevator/P")
    elevatorITopic = ntutil.getFloatTopic("/Elevator/I")
    elevatorDTopic = ntutil.getFloatTopic("/Elevator/D")
    elevatorSetpointTopic = ntutil.getFloatTopic("/Elevator/Setpoint/Actual")
    elevatorSlewedSetpointTopic = ntutil.getFloatTopic("/Elevator/Setpoint/Slewed")

    # Arm hardware
    wristMotor = rev.SparkMax(41, rev.SparkLowLevel.MotorType.kBrushless)
    armOuterWheelMotor = rev.SparkMax(42, rev.SparkLowLevel.MotorType.kBrushless)
    armInnerWheelMotor = rev.SparkMax(43, rev.SparkLowLevel.MotorType.kBrushless)

    wristEncoder = wristMotor.getAbsoluteEncoder()
    wristController = wristMotor.getClosedLoopController()

    wristMotorConfig = rev.SparkMaxConfig()
    (
        wristMotorConfig
            .inverted(False)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)
    )
    (
        wristMotorConfig
            .absoluteEncoder
                .positionConversionFactor(2*math.pi)
                .velocityConversionFactor(2*math.pi/60)
                .inverted(True)
                .zeroOffset(0.0891859)
    )
    (
        wristMotorConfig
            .closedLoop
                .pid(constants.kArmP, constants.kArmI, constants.kArmD)
                .outputRange(-0.175, 0.175)
                .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .IZone(wpimath.units.degreesToRadians(10))
    )
    wristMotor.configure(wristMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    armOuterWheelMotorConfig = rev.SparkMaxConfig()
    armInnerWheelMotorConfig = rev.SparkMaxConfig()
    (
        armOuterWheelMotorConfig
            .inverted(True)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)
    )
    (
        armInnerWheelMotorConfig
            .inverted(False)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)
    )
    armOuterWheelMotor.configure(armOuterWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    armInnerWheelMotor.configure(armInnerWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    # Arm telemetry
    wristAngleTopic = ntutil.getFloatTopic("/Wrist/Angle")
    wristAngleRawTopic = ntutil.getFloatTopic("/Wrist/AngleRaw")
    wristAngleSetpointTopic = ntutil.getFloatTopic("/Wrist/AngleSetpoint")
    wristSafePositionTopic = ntutil.getFloatTopic("/Wrist/SafePosition")
    wristFFTopic = ntutil.getFloatTopic("/Wrist/FF")

    # do not delete
    CalebIsProTopic = ntutil.getStringTopic("/CalebIsTheGoat")

    # Control variables
    elevatorSetpoint = 0
    elevatorSetpointLimiter = SlewRateLimiter(1) #m/s
    wristPositionSetpoint = 0.0

    def periodic(self):
        slewedElevatorSetpoint = self.elevatorSetpointLimiter.calculate(self.elevatorSetpoint)
        self.elevatorController.setReference(slewedElevatorSetpoint, rev.SparkMax.ControlType.kPosition, arbFeedforward=0)
        
        self.elevatorSetpointTopic.set(self.elevatorSetpoint)
        self.elevatorSlewedSetpointTopic.set(slewedElevatorSetpoint)
        self.elevatorHeightTopic.set(self.elevatorEncoder.getPosition())

        self.wristAngleTopic.set(wristEncoderToAngle(self.wristEncoder.getPosition()))
        self.wristAngleRawTopic.set(self.wristEncoder.getPosition())
        self.wristAngleSetpointTopic.set(self.wristPositionSetpoint)

        # Recalculate the new safe wrist position based on the current elevator height
        safeWristPosition = self.get_safe_wrist_position(self.wristPositionSetpoint)
        armFF = self.calculate_arm_ff()
        self.wristController.setReference(wristAngleToEncoder(safeWristPosition), rev.SparkMax.ControlType.kPosition, arbFeedforward=armFF)

        self.wristSafePositionTopic.set(safeWristPosition)
        self.wristFFTopic.set(armFF)

    def set_elevator_pid(self, p: float, i: float, d: float):
        """
        Set the elevator PID constants.
        """
        
        self.elevatorMotor1Config.closedLoop.pid(p, i, d)
        self.elevatorMotor1.configure(self.elevatorMotor1Config, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

        self.elevatorPTopic.set(p)
        self.elevatorITopic.set(i)
        self.elevatorDTopic.set(d)

    def set_arm_position(self, height: float, angle: float):
        self.wristPositionSetpoint = angle
        self.elevatorSetpoint = self.compute_elevator_height(height,angle)

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

    def calculate_arm_ff(self):
        """
        Calculates the arm feedforward based on the current wrist angle
        """

        wristAngle = self.get_wrist_position()

        # Calculate the arm feedforward
        armFF = ArmFeedforward(constants.kArmKS, constants.kArmKG, constants.kArmKV, constants.kArmKA)

        ffVoltage = armFF.calculate(wristAngle + wpimath.units.degreesToRadians(90), self.wristEncoder.getVelocity())

        return ffVoltage

    def compute_elevator_height(self, armHeight, angle):
        return armHeight - constants.kArmHeightInCarriage - constants.kElevatorBaseHeight - constants.kArmCoralRadius * math.cos(angle)