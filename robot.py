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
# from urcl import URCL

from subsystems.drivetrain import Drivetrain
from subsystems.sourceintake import SourceIntake
from subsystems.elevatorandarm import ElevatorAndArm
import utils

class MyRobot(wpilib.TimedRobot):
    leftStick = wpilib.Joystick(0)
    rightStick = wpilib.Joystick(1)
    gamePad = wpilib.XboxController(2)

    drivetrain = Drivetrain()
    sourceintake = SourceIntake()
    elevatorandarm = ElevatorAndArm()

    def robotInit(self):
        # wpilib.DataLogManager.start()
        # URCL.start()
        pass
    
    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.sourceintake.periodic()
        self.elevatorandarm.periodic()
    
    def testPeriodic(self):
        armAngle = utils.remap(self.rightStick.getRawAxis(2), (-1, 1), (0, wpimath.units.degreesToRadians(-180)))
        self.elevatorandarm.set_wrist_position(armAngle)

    def teleopPeriodic(self) -> None:
        xSpeed = wpimath.applyDeadband(-self.leftStick.getY(), 0.2)
        ySpeed = wpimath.applyDeadband(-self.leftStick.getX(), 0.2)
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.2)
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed)

        elevatorSpeed = wpimath.applyDeadband(-self.gamePad.getLeftY(), 0.1)
        self.elevatorandarm.move_elevator(elevatorSpeed)

        if self.gamePad.getRightTriggerAxis() > 0.5:
            self.sourceintake.run_intake(1)
        elif self.gamePad.getRightBumperButton():
            self.sourceintake.run_intake(-1)
        else:
            self.sourceintake.run_intake(0)

        if self.gamePad.getAButton():
            self.elevatorandarm.move_coral(-1)
        elif self.gamePad.getBButton():
            self.elevatorandarm.move_coral(1)
        elif self.gamePad.getLeftTriggerAxis() > 0.5:
            self.elevatorandarm.move_algae(1)
        elif self.gamePad.getLeftBumperButton():
            self.elevatorandarm.move_algae(-1)
        else:
            self.elevatorandarm.move_coral(0)