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
        pass


    def teleopPeriodic(self) -> None:
        xSpeed = wpimath.applyDeadband(-self.leftStick.getY(), 0.2)
        ySpeed = wpimath.applyDeadband(-self.leftStick.getX(), 0.2)
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.2)
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed * constants.kMaxTurnSpeed)

        elevatorSpeed = wpimath.applyDeadband(-self.gamePad.getLeftY(), 0.1)
        self.elevatorandarm.move_elevator(elevatorSpeed)

        if self.gamePad.getAButton():
            self.elevatorandarm.set_wrist_position(wpimath.units.degreesToRadians(-140))
        elif self.gamePad.getBButton():
            self.elevatorandarm.set_wrist_position(wpimath.units.degreesToRadians(-90))
        elif self.gamePad.getXButton():
            self.elevatorandarm.set_wrist_position(wpimath.units.degreesToRadians(-30))
        elif self.gamePad.getYButton():
            self.elevatorandarm.set_wrist_position(wpimath.units.degreesToRadians(0))   

        coralSpeed = self.gamePad.getRightTriggerAxis()-self.gamePad.getLeftTriggerAxis()

        if self.gamePad.getLeftBumper():
            self.elevatorandarm.move_algae(1)
        elif self.gamePad.getRightBumper():
            self.elevatorandarm.move_algae(-1)
        elif abs(coralSpeed) > 0.2:
            self.sourceintake.run_intake(coralSpeed)
            self.elevatorandarm.move_coral(coralSpeed)
        else:
            self.elevatorandarm.move_algae(0)
            self.sourceintake.run_intake(0)
    
        if self.leftStick.getRawButtonPressed(8):
            self.drivetrain.reset_heading()
