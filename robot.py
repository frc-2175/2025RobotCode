# TODO: insert robot code here
from wpimath.geometry import Translation2d
import wpilib
import rev
import wpimath
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import ntcore
import wpimath.units

wheelDistanceFromCenter = wpimath.units.inchesToMeters(12.375)
frontLeftLocation = Translation2d(wheelDistanceFromCenter, wheelDistanceFromCenter)
frontRightLocation = Translation2d(wheelDistanceFromCenter, -wheelDistanceFromCenter)
backLeftLocation = Translation2d(-wheelDistanceFromCenter, wheelDistanceFromCenter)
backRightLocation = Translation2d(-wheelDistanceFromCenter, -wheelDistanceFromCenter)

kinematics = SwerveDrive4Kinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation)
nt= ntcore.NetworkTableInstance.getDefault()
topic=nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
pub= topic.publish()

class MyRobot(wpilib.TimedRobot):
    leftStick = wpilib.Joystick(0)
    rightStick = wpilib.Joystick(1)
    gamePad = wpilib.XboxController(2)
    
    def robotInit(self) -> None:
        # self.testMotor = rev.CANSparkMax(testMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
        # self.testMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        pass

    def teleopPeriodic(self) -> None:
        speeds= ChassisSpeeds(-self.leftStick.getY(), -self.leftStick.getX(), -self.rightStick.getX())
        frontLeft, frontRight,backLeft,backRight= kinematics.toSwerveModuleStates(speeds)
        pub.set([frontLeft,frontRight,backLeft,backRight])
        pass

