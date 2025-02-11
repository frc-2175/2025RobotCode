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

        elevatorSpeed = wpimath.applyDeadband(-self.gamePad.getLeftY(), 0.1)
        self.elevatorandarm.move_elevator(elevatorSpeed)
        
        wristSpeed = wpimath.applyDeadband(-self.gamePad.getRightY(), 0.1)
        self.elevatorandarm.move_arm(wristSpeed)

        if self.gamePad.getRightTriggerAxis() > 0.5:
            self.sourceintake.run_intake(constants.kSourceIntakeSpeed)
        elif self.gamePad.getRightBumperButton():
            self.sourceintake.run_intake(-constants.kSourceIntakeSpeed)
        else:
            self.sourceintake.run_intake(0)

        if self.gamePad.getAButton():
            self.elevatorandarm.move_coral(constants.kSquishyWheelCoralSpeed)
        elif self.gamePad.getBButton():
            self.elevatorandarm.move_coral(-constants.kSquishyWheelCoralSpeed)
        elif self.gamePad.getLeftTriggerAxis() > 0.5:
            self.elevatorandarm.move_algae(constants.kSquishyWheelAlgaeSpeed)
        elif self.gamePad.getLeftBumperButton():
            self.elevatorandarm.move_algae(-constants.kSquishyWheelAlgaeSpeed)
        else:
            self.elevatorandarm.move_coral(0)