import rev
import math
from wpimath.geometry import Translation2d
import wpilib
import rev
import wpimath
import wpimath.geometry
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import ntcore
import wpimath.units
import constants

elevatorMotor1Config = rev.SparkMaxConfig()
elevatorMotor1Config.encoder.positionConversionFactor(
    constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2
).velocityConversionFactor(
    constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2 / 60
)
elevatorMotor1Config.softLimit.forwardSoftLimit(constants.kMaxElevatorHeight).reverseSoftLimit(constants.kMinElevatorHeight).forwardSoftLimitEnabled(True).reverseSoftLimitEnabled(True)
elevatorMotor2Config = rev.SparkMaxConfig()

# Should this even be inverted?
elevatorMotor2Config.follow(31, True)

wristMotorConfig = rev.SparkMaxConfig()
# wristMotorConfig.encoder.velocityConversionFactor(
#     constants.kWristMotorReduction * 2 * math.pi / 60.0
# ).positionConversionFactor(
#     constants.kWristMotorReduction * 2 * math.pi
# )
wristMotorConfig.absoluteEncoder.positionConversionFactor(2*math.pi).velocityConversionFactor(2*math.pi/60)
wristMotorConfig.softLimit.forwardSoftLimit(constants.kMaxWristAngle).reverseSoftLimit(constants.kMinWristAngle).forwardSoftLimitEnabled(True).reverseSoftLimitEnabled(True)

nt = ntcore.NetworkTableInstance.getDefault()
elevatorHeightTopic = nt.getFloatTopic("/ElevatorHeight").publish()
wristAngleTopic = nt.getFloatTopic("/WristAngle").publish()
CalebIsProTopic = nt.getStringTopic("/CalebIsTheGoat").publish()


class ElevatorAndArm:
    # Elevator hardware
    elevatorMotor1 = rev.SparkMax(31, rev.SparkLowLevel.MotorType.kBrushless)
    elevatorMotor2 = rev.SparkMax(32, rev.SparkLowLevel.MotorType.kBrushless)
    elevatorMotor2.configure(elevatorMotor2Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    elevatorMotor1.configure(elevatorMotor1Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    
    elevatorEncoder = elevatorMotor1.getEncoder()

    # Arm hardware
    wristMotor = rev.SparkMax(41, rev.SparkLowLevel.MotorType.kBrushless)
    armOuterWheelMotor = rev.SparkMax(42, rev.SparkLowLevel.MotorType.kBrushless)
    armInnerWheelMotor = rev.SparkMax(43, rev.SparkLowLevel.MotorType.kBrushless)
    wristEncoder = wristMotor.getAbsoluteEncoder()

    wristMotor.configure(wristMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    def periodic(self):
        elevatorHeightTopic.set(self.elevatorEncoder.getPosition())
        wristAngleTopic.set(self.wristEncoder.getPosition())
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
        self.wristMotor.set(speed * 0.2)
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

    
