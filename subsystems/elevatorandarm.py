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
elevatorMotor2Config = rev.SparkMaxConfig( )
elevatorMotor2Config.follow(31)

wristMotorConfig = rev.SparkMaxConfig()
wristMotorConfig.encoder.velocityConversionFactor(
    (math.pi *constants.kWheelDiameter / constants.kDriveMotorReduction) / 60.0
).positionConversionFactor(
    constants.kWristMotorReduction * 2 * math.pi
)

class ElevatorAndArm:

    # Elevator hardware
    
     
    # TODO: Elevator motor 1 (ID 31)
    elevatorMotor1 = rev.SparkMax(31, rev.SparkLowLevel.MotorType.kBrushless)
    # TODO: Elevator motor 2 (ID 32)
    elevatorMotor2 = rev.SparkMax(32, rev.SparkLowLevel.MotorType.kBrushless)
    # TODO: Set elevator motor 2 to follow motor 1
    elevatorMotor2.configure(elevatorMotor2Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    # TODO: Set elevator position/velocity conversion factors


    # TODO: Set elevator spark max soft limits to enforce safety


    # Arm hardware
    # TODO: Wrist motor (ID 41)
    wristMotor = rev.SparkMax(41, rev.SparkLowLevel.MotorType.kBrushless)
    # TODO: Outer wheel motor (ID 42)
    armOuterWheelMotor = rev.SparkMax(42, rev.SparkLowLevel.MotorType.kBrushless)
    # TODO: Inner wheel motor (ID 43)
    armInnerWheelMotor = rev.SparkMax(43, rev.SparkLowLevel.MotorType.kBrushless)

    # TODO: Absolute encoder for the wrist?

    # TODO: Set wrist position/velocity conversion factors
    wristMotor.configure(wristMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    # TODO: Set wrist spark max soft limits for safety


    def periodic(self):
        # TODO: Report encoder positions (and anything else) to NetworkTables
        pass

    def move_elevator(self, speed: float):
        """
        Manually control the elevator speed. Positive means up, negative means down.
        """
        # TODO: Implement this

        self.elevatorMotor1.set(speed)

        pass

    def move_arm(self, speed: float):
        """
        Manually control the arm (wrist) speed. Negative means moving the arm out/down,
        positive means moving the arm up/in.
        """
        # TODO: Implement this
        pass

    def move_coral(self, speed: float):
        """
        Spins the squishy wheels in opposite directions to move coral. Speeds range from
        -1 to 1, where positive means intake -> reef. Speed will be limited by the top
        speed in constants.py.
        """
        # TODO: Implement this
        pass

    def move_algae(self, speed: float):
        """
        Spins the squishy wheels in the same direction to move algae. Speeds range from
        -1 to 1, where positive means robot -> processor. Speed will be limited by the
        top speed in constants.py.
        """
        # TODO: Implement this
        pass

    
