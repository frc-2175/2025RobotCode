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
    def robotInit(self):
        # wpilib.DataLogManager.start()
        # URCL.start()

        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.gamePad = wpilib.XboxController(2)

        self.drivetrain = Drivetrain()
        self.sourceintake = SourceIntake()
        self.elevatorandarm = ElevatorAndArm()
        self.hanger = Hanger()

        self.scoringMode = constants.kCoralMode
        self.algaeReverse = False

        self.elevatorandarm.set_arm_position(constants.kElevatorL1, constants.kWristUprightAngle, constants.kCoralMode)

    
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
            self.scoringMode = constants.kCoralMode

        if self.gamePad.getLeftBumper():
            self.scoringMode = constants.kAlgaeMode
        
        if self.scoringMode == constants.kCoralMode:
            if self.gamePad.getAButton():
                #Set Elevator to L1/Handoff Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL1, constants.kWristUprightAngle, constants.kCoralMode)
            elif self.gamePad.getBButton():
                #Set Elevator to L2 Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL2, constants.kWristCoralScoreAngle, constants.kCoralMode)
            elif self.gamePad.getXButton():
                #Set Elevator to L3 Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL3, constants.kWristCoralScoreAngle, constants.kCoralMode)
            elif self.gamePad.getYButton():
                #Set Elevator to L4 Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL4, constants.kWristHighCoralScoreAngle, constants.kCoralMode)

            coralSpeed = wpimath.applyDeadband(gamePieceSpeed, 0.2)
            self.elevatorandarm.move_coral(coralSpeed)

        elif self.scoringMode == constants.kAlgaeMode:
            if self.gamePad.getAButton():
                self.elevatorandarm.set_arm_position(constants.kElevatorAlgaeGround, constants.kWristAlgaeGround, constants.kAlgaeMode)
                self.algaeReverse = False
            elif self.gamePad.getXButton() or self.gamePad.getBButton():
                self.algaeReverse = False
                self.elevatorandarm.set_arm_position(constants.kElevatorAlgaeLow, constants.kWristAlgaeDereef, constants.kAlgaeMode)
            elif self.gamePad.getYButton():
                self.elevatorandarm.set_arm_position(constants.kElevatorAlgaeHigh, constants.kWristAlgaeDereef, constants.kAlgaeMode)
                self.algaeReverse = True
            
            if self.algaeReverse == True:
                self.elevatorandarm.move_algae(gamePieceSpeed)
            elif self.algaeReverse == False:
                self.elevatorandarm.move_algae(-gamePieceSpeed)
            else:
                print(f"Variable algaeReverse improper value: {self.algaeReverse}; expected True or False")

        else:
            print(f"Variable scoringMode improper value: {self.scoringMode}; expected Coral or Algae")