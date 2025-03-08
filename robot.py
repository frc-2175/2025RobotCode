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
from subsystems.hanger import Hanger

class MyRobot(wpilib.TimedRobot):
    leftStick = wpilib.Joystick(0)
    rightStick = wpilib.Joystick(1)
    gamePad = wpilib.XboxController(2)

    drivetrain = Drivetrain()
    sourceintake = SourceIntake()
    elevatorandarm = ElevatorAndArm()
    hanger = Hanger()

    scoringMode = 0
    algaeReverse = False

    def robotInit(self):
        # wpilib.DataLogManager.start()
        # URCL.start()
        pass
    
    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.sourceintake.periodic()
        self.elevatorandarm.periodic()
        self.hanger.periodic()
    
    def testPeriodic(self):
        # self.elevatorandarm.set_elevator_pid(utils.remap(self.leftStick.getRawAxis(2), (-1, 1), (3, 0)), 0, 0)
        # self.elevatorandarm.set_elevator_position(utils.remap(self.rightStick.getRawAxis(2), (-1, 1), (1, 0)))
        self.hanger.set_position(utils.remap(self.leftStick.getRawAxis(2), (-1, 1), (math.pi / 2, -math.pi / 2)))
        pass


    def teleopPeriodic(self) -> None:
        xSpeed = constants.kMaxSpeed * wpimath.applyDeadband(-self.leftStick.getY(), 0.1)
        ySpeed = constants.kMaxSpeed * wpimath.applyDeadband(-self.leftStick.getX(), 0.1)
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.1)
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed * constants.kMaxTurnSpeed)
        if self.leftStick.getRawButtonPressed(8):
            self.drivetrain.reset_heading()

        gamePieceSpeed = self.gamePad.getRightTriggerAxis()-self.gamePad.getLeftTriggerAxis()

        if self.gamePad.getRightBumper():
            self.scoringMode = 0

        if self.gamePad.getLeftBumper():
            self.scoringMode = 1
        
        if self.scoringMode == 0:
            if self.gamePad.getAButton():
                self.elevatorandarm.set_wrist_position(constants.kWristUprightAngle)
                #Set Elevator to L1/Handoff Height
                self.elevatorandarm.set_elevator_position(constants.kElevatorL1)
            elif self.gamePad.getBButton():
                self.elevatorandarm.set_wrist_position(constants.kWristCoralScoreAngle)
                #Set Elevator to L2 Height
                self.elevatorandarm.set_elevator_position(constants.kElevatorL2)
            elif self.gamePad.getXButton():
                self.elevatorandarm.set_wrist_position(constants.kWristCoralScoreAngle)
                #TODO: Set Elevator to L3 Height
                self.elevatorandarm.set_elevator_position(constants.kElevatorL3)
            elif self.gamePad.getYButton():
                self.elevatorandarm.set_wrist_position(constants.kWristCoralScoreAngle)
                #TODO" Set Elevator to L4 Height
                self.elevatorandarm.set_elevator_position(constants.kElevatorL4)

            coralSpeed = wpimath.applyDeadband(gamePieceSpeed, 0.2)
            self.elevatorandarm.move_coral(coralSpeed)

        elif self.scoringMode == 1:
            if self.gamePad.getAButton():
                self.elevatorandarm.set_wrist_position(constants.kWristAlgaeGround)
                self.algaeReverse = False
                #TODO: Set Elevator to Algae Ground Height
                self.elevatorandarm.set_elevator_position(constants.kElevatorAlgaeGround)
            elif self.gamePad.getXButton() or self.gamePad.getBButton():
                self.elevatorandarm.set_wrist_position(constants.kWristAlgaeDereef)
                self.algaeReverse = False
                #TODO Set Elevator To Algae Low DeReef Height
                self.elevatorandarm.set_elevator_position(constants.kElevatorAlgaeLow)
            elif self.gamePad.getYButton():
                self.elevatorandarm.set_wrist_position(constants.kWristAlgaeDereef)
                self.algaeReverse = True
                #TODO Set Elevator To Algae High DeReef Height
                self.elevatorandarm.set_elevator_position(constants.kElevatorAlgaeHigh)
            
            if self.algaeReverse == True:
                self.elevatorandarm.move_algae(gamePieceSpeed)
            elif self.algaeReverse == False:
                self.elevatorandarm.move_algae(-gamePieceSpeed)
            else:
                print(f"Variable algaeReverse improper value: {self.algaeReverse}; expected True or False")
            
            self.sourceintake.run_intake(0)
        else:
            print(f"Variable scoringMode improper value: {self.scoringMode}; expected Coral or Algae")