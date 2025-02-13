import rev
import math
from wpimath.geometry import Translation2d
import wpilib
import wpimath
import wpimath.geometry
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import ntcore
import wpimath.units
import constants

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
# wristMotorConfig.encoder.velocityConversionFactor(
#     constants.kWristMotorReduction * 2 * math.pi / 60.0
# ).positionConversionFactor(
#     constants.kWristMotorReduction * 2 * math.pi
# )
wristMotorConfig.inverted(True)
wristMotorConfig.absoluteEncoder.positionConversionFactor(2*math.pi).velocityConversionFactor(2*math.pi/60).inverted(True).zeroOffset(0.0424)
#wristMotorConfig.softLimit.forwardSoftLimit(wristAngleToEncoder(constants.kMaxWristAngle)).reverseSoftLimit(wristAngleToEncoder(constants.kMinWristAngle)).forwardSoftLimitEnabled(True).reverseSoftLimitEnabled(True)

nt = ntcore.NetworkTableInstance.getDefault()
elevatorHeightTopic = nt.getFloatTopic("/ElevatorHeight").publish()
wristAngleTopic = nt.getFloatTopic("/WristAngle").publish()
wristAngleRawTopic = nt.getFloatTopic("/WristAngleRawTopic").publish()
CalebIsProTopic = nt.getStringTopic("/CalebIsTheGoat").publish()

class ElevatorAndArm:
    # Elevator hardware
    elevatorMotor1 = rev.SparkMax(31, rev.SparkLowLevel.MotorType.kBrushless) #Left motor from robot POV
    elevatorMotor2 = rev.SparkMax(32, rev.SparkLowLevel.MotorType.kBrushless) #Right motor from robot POV
    elevatorMotor2.configure(elevatorMotor2Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    elevatorMotor1.configure(elevatorMotor1Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    
    elevatorEncoder = elevatorMotor1.getEncoder()

    # Arm hardware
    wristMotor = rev.SparkMax(41, rev.SparkLowLevel.MotorType.kBrushless)
    armOuterWheelMotor = rev.SparkMax(42, rev.SparkLowLevel.MotorType.kBrushless)
    armInnerWheelMotor = rev.SparkMax(43, rev.SparkLowLevel.MotorType.kBrushless)
    wristEncoder = wristMotor.getAbsoluteEncoder()

    wristMotor.configure(wristMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    armOuterWheelMotor.configure(armOuterWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    armInnerWheelMotor.configure(armInnerWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    def periodic(self):
        elevatorHeightTopic.set(self.elevatorEncoder.getPosition())
        wristAngleTopic.set(wristEncoderToAngle(self.wristEncoder.getPosition()))
        wristAngleRawTopic.set(self.wristEncoder.getPosition())
        CalebIsProTopic.set("Caleb is sigma ")
        pass

    def move_elevator(self, speed: float):
        """
        Manually control the elevator speed. Positive means up, negative means down.
        """
        self.elevatorMotor1.set(speed * 0.2)
        # The other elevator motor is set as a follower.

    def move_arm(self, speed: float):
        """
        Manually control the arm (wrist) speed. Negative means moving the arm out/down,
        positive means moving the arm up/in.
        """
        self.wristMotor.set(speed * 0.1)
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

    
