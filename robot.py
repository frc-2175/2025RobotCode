from wpimath.geometry import Translation2d
import wpilib
import math
import rev
import wpimath
import wpimath.geometry
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import ntcore
import wpimath.units
import constants

from subsystems.drivetrain import Drivetrain
from subsystems.sourceintake import SourceIntake
from subsystems.elevatorandarm import ElevatorAndArm

class MyRobot(wpilib.TimedRobot):
    leftStick = wpilib.Joystick(0)
    rightStick = wpilib.Joystick(1)
    gamePad = wpilib.XboxController(2)

    drivetrain = Drivetrain()
    sourceintake = SourceIntake()
    elevatorandarm = ElevatorAndArm()
    
    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.sourceintake.periodic()
        self.elevatorandarm.periodic()

    def teleopPeriodic(self) -> None:
        xSpeed = wpimath.applyDeadband(-self.leftStick.getY(), 0.2)
        ySpeed = wpimath.applyDeadband(-self.leftStick.getX(), 0.2)
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.2)
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed)

        # DO NOT DO THE FOLLOWING UNTIL ALL SAFETY CONSTRAINTS HAVE BEEN PROGRAMMED IN EACH SUBSYSTEM!
        # TODO: Elevator controls (e.g. left gamepad stick up/down, calling self.drivetrain.move_elevator()
        # TODO: Arm controls (e.g. right gamepad stick up/down, calling self.drivetrain.move_elevator()
        # TODO: Source intake controls (e.g. right trigger to pull coral into arm, right bumper to push coral back to source)
        # TODO: Arm coral controls (e.g. A button to score, B button to reverse)
        # TODO: Arm algae controls (e.g. left trigger to grab/de-reef, left bumper to score into processor)
