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
# TODO: Ensure that this motor is inverted! The above should look like `follow(31, True)`, as there
# is a second optional parameter to `follow` that controls whether the follower is inverted or not.

wristMotorConfig = rev.SparkMaxConfig()
wristMotorConfig.encoder.velocityConversionFactor(
    # TODO: This factor is incorrect. These constants are for the drive system, not for the arm.
    # The position conversion factor below, however, should (in principle) be correct. This means
    # that the correct factor here should instead be:
    #
    #  constants.kWristMotorReduction * 2 * math.pi / 60.0
    #
    # This will convert from RPM (rev/min) to radians/second, which is a more useful unit for us.
    (math.pi *constants.kWheelDiameter / constants.kDriveMotorReduction) / 60.0
).positionConversionFactor(
    constants.kWristMotorReduction * 2 * math.pi
)

class ElevatorAndArm:
    # Elevator hardware
    elevatorMotor1 = rev.SparkMax(31, rev.SparkLowLevel.MotorType.kBrushless)
    elevatorMotor2 = rev.SparkMax(32, rev.SparkLowLevel.MotorType.kBrushless)
    elevatorMotor2.configure(elevatorMotor2Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    # TODO: Set elevator position/velocity conversion factors. These should be in meters and meters per second respectively.

    # TODO: Set elevator spark max soft limits to enforce safety.
    # Example:
    #
    #  elevatorMotor1Config.softLimit.forwardSoftLimit(constants.kMaxElevatorHeight).reverseSoftLimit(constants.kMinElevatorHeight)
    #
    # You may also have to do .forwardSoftLimitEnabled(True).reverseSoftLimitEnabled(True). I'm not sure.

    # Arm hardware
    wristMotor = rev.SparkMax(41, rev.SparkLowLevel.MotorType.kBrushless)
    armOuterWheelMotor = rev.SparkMax(42, rev.SparkLowLevel.MotorType.kBrushless)
    armInnerWheelMotor = rev.SparkMax(43, rev.SparkLowLevel.MotorType.kBrushless)

    # TODO: Absolute encoder for the wrist?

    wristMotor.configure(wristMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    # TODO: Set wrist spark max soft limits for safety. See elevator example above. The soft limits on the arm should be in
    # radians, and note that we defined negative angle to mean out/down and positive to mean up/in.

    def periodic(self):
        # TODO: Report encoder positions (and anything else) to NetworkTables
        pass

    def move_elevator(self, speed: float):
        """
        Manually control the elevator speed. Positive means up, negative means down.
        """
        self.elevatorMotor1.set(speed)
        # The other elevator motor is set as a follower.

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

    
