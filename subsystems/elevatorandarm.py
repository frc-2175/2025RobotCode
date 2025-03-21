import math
from typing import Literal

import ntcore
import rev
import wpilib
import wpimath
import wpimath.geometry
import wpimath.units
from wpilib import SmartDashboard
from wpimath.controller import ArmFeedforward
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d
from wpimath.kinematics import (ChassisSpeeds, SwerveDrive4Kinematics,
                                SwerveModuleState)

import constants
import ntutil
import utils


def wristEncoderToAngle(encoderPosition: float) -> float:
    return encoderPosition - constants.kWristAngleOffset

def wristAngleToEncoder(angleRadians: float) -> float:
    return angleRadians + constants.kWristAngleOffset

class ElevatorAndArm:
    def __init__(self):
        # Elevator hardware
        self.elevatorMotor1 = rev.SparkMax(31, rev.SparkLowLevel.MotorType.kBrushless) #Left motor from robot POV
        self.elevatorMotor2 = rev.SparkMax(32, rev.SparkLowLevel.MotorType.kBrushless) #Right motor from robot POV

        self.elevatorEncoder = self.elevatorMotor1.getEncoder()
        self.elevatorController = self.elevatorMotor1.getClosedLoopController()

        elevatorMotor1Config = rev.SparkMaxConfig()
        (
            elevatorMotor1Config
                .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(constants.kElevatorCurrentLimit)
        )
        (
            elevatorMotor1Config
                .encoder
                    .positionConversionFactor(constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2)
                    .velocityConversionFactor(constants.kElevatorMotorReduction * constants.kElevatorSprocketDiameter * math.pi * 2 / 60)
        )
        (
            elevatorMotor1Config
                .softLimit
                    .forwardSoftLimit(constants.kMaxElevatorHeight)
                    .forwardSoftLimitEnabled(True)
                    .reverseSoftLimit(constants.kMinElevatorHeight)
                    .reverseSoftLimitEnabled(True)
        )
        (
            elevatorMotor1Config
                .closedLoop
                    .pid(constants.kElevatorP, constants.kElevatorI, constants.kElevatorD)
                    .outputRange(-0.8, 0.8)
                    .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                    .IZone(wpimath.units.inchesToMeters(3))
        )
        self.elevatorMotor1.configure(elevatorMotor1Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        
        elevatorMotor2Config = rev.SparkMaxConfig()
        (
            elevatorMotor2Config
                .smartCurrentLimit(constants.kElevatorCurrentLimit)
                .follow(31, True)
        )
        self.elevatorMotor2.configure(elevatorMotor2Config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        # Elevator telemetry
        self.elevatorHeightTopic = ntutil.getFloatTopic("/Elevator/Height")
        self.elevatorPTopic = ntutil.getFloatTopic("/Elevator/P")
        self.elevatorITopic = ntutil.getFloatTopic("/Elevator/I")
        self.elevatorDTopic = ntutil.getFloatTopic("/Elevator/D")
        self.elevatorSetpointTopic = ntutil.getFloatTopic("/Elevator/Setpoint/Actual")
        self.elevatorSlewedSetpointTopic = ntutil.getFloatTopic("/Elevator/Setpoint/Slewed")

        # Alerts
        self.badCoralLevelAlert = wpilib.Alert("Bad level commanded for coral; ignoring", wpilib.Alert.AlertType.kWarning)

        # Arm hardware
        self.wristMotor = rev.SparkMax(41, rev.SparkLowLevel.MotorType.kBrushless)
        self.armOuterWheelMotor = rev.SparkMax(42, rev.SparkLowLevel.MotorType.kBrushless)
        self.armInnerWheelMotor = rev.SparkMax(43, rev.SparkLowLevel.MotorType.kBrushless)
        self.intakeSensor = wpilib.AnalogInput(3)
        self.intakeSensor.setAverageBits(2)

        self.wristEncoder = self.wristMotor.getAbsoluteEncoder()
        self.wristController = self.wristMotor.getClosedLoopController()
        self.armOuterWheelEncoder = self.armOuterWheelMotor.getEncoder()
        self.armInnerWheelEncoder = self.armInnerWheelMotor.getEncoder()

        wristMotorConfig = rev.SparkMaxConfig()
        (
            wristMotorConfig
                .inverted(False)
                .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(40)
        )
        (
            wristMotorConfig
                .absoluteEncoder
                    .positionConversionFactor(2*math.pi)
                    .velocityConversionFactor(2*math.pi/60)
                    .inverted(True)
                    .zeroOffset(0.0891859)
        )
        (
            wristMotorConfig
                .closedLoop
                    .pid(constants.kArmP, constants.kArmI, constants.kArmD)
                    .outputRange(-0.175, 0.175)
                    .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                    .IZone(wpimath.units.degreesToRadians(10))
        )
        self.wristMotor.configure(wristMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        armOuterWheelMotorConfig = rev.SparkMaxConfig()
        armInnerWheelMotorConfig = rev.SparkMaxConfig()
        (
            armOuterWheelMotorConfig
                .inverted(True)
                .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(40)
        )
        (
            armInnerWheelMotorConfig
                .inverted(False)
                .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(40)
        )
        self.armOuterWheelMotor.configure(armOuterWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.armInnerWheelMotor.configure(armInnerWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        # Arm telemetry
        self.wristAngleTopic = ntutil.getFloatTopic("/Wrist/Angle")
        self.wristAngleRawTopic = ntutil.getFloatTopic("/Wrist/AngleRaw")
        self.wristAngleSetpointTopic = ntutil.getFloatTopic("/Wrist/AngleSetpoint")
        self.wristSafePositionTopic = ntutil.getFloatTopic("/Wrist/SafePosition")
        self.wristFFTopic = ntutil.getFloatTopic("/Wrist/FF")
        self.intakeSensorTopic = ntutil.getFloatTopic("/Intake/Sensor")
        self.coralDetectedTopic = ntutil.getBooleanTopic("/Intake/Detected")
        self.armInnerWheelSpeedTopic = ntutil.getFloatTopic("/Intake/InnerWheelVelocity")
        self.armOuterWheelSpeedTopic = ntutil.getFloatTopic("/Intake/OuterWheelVelocity")

        # Mechanism2d telemetry
        self.mechActual = self.Mechanism("ElevatorMechanism/Actual", wpilib.Color.kRed)
        self.mechDesired = self.Mechanism("ElevatorMechanism/Desired", wpilib.Color.kBlue)

        # do not delete
        self.CalebIsProTopic = ntutil.getStringTopic("/CalebIsTheGoat")

        # Control variables
        self.elevatorSetpoint = 0
        self.elevatorSetpointLimiter = SlewRateLimiter(1) #m/s
        self.wristPositionSetpoint = 0.0
        self.previousCoralDetected = False
        self.intakeState: Literal["initial"] | Literal["stopped"] = "initial"
        self.intakeSpeed = 0
        self.intakeMode: Literal["coral"] | Literal["algae"] = "coral"

    def periodic(self):
        # Elevator
        slewedElevatorSetpoint = self.elevatorSetpointLimiter.calculate(self.elevatorSetpoint)
        self.elevatorController.setReference(slewedElevatorSetpoint, rev.SparkMax.ControlType.kPosition, arbFeedforward=0)
        
        self.elevatorSetpointTopic.set(self.elevatorSetpoint)
        self.elevatorSlewedSetpointTopic.set(slewedElevatorSetpoint)
        self.elevatorHeightTopic.set(self.elevatorEncoder.getPosition())

        # Wrist
        self.wristAngleTopic.set(wristEncoderToAngle(self.wristEncoder.getPosition()))
        self.wristAngleRawTopic.set(self.wristEncoder.getPosition())
        self.wristAngleSetpointTopic.set(self.wristPositionSetpoint)

        # Recalculate the new safe wrist position based on the current elevator height
        safeWristPosition = self.get_safe_wrist_position(self.wristPositionSetpoint)
        armFF = self.calculate_arm_ff()
        self.wristController.setReference(wristAngleToEncoder(safeWristPosition), rev.SparkMax.ControlType.kPosition, arbFeedforward=armFF)

        self.wristSafePositionTopic.set(safeWristPosition)
        self.wristFFTopic.set(armFF)

        # Intake wheels
        if self.intakeState == "initial":
            if self.intakeMode == "coral":
                self.armOuterWheelMotor.set(self.intakeSpeed * constants.kSquishyWheelCoralSpeed)
                self.armInnerWheelMotor.set(-self.intakeSpeed * constants.kSquishyWheelCoralSpeed)
            elif self.intakeMode == "algae":
                self.armOuterWheelMotor.set(self.intakeSpeed * constants.kSquishyWheelAlgaeSpeed)
                self.armInnerWheelMotor.set(self.intakeSpeed * constants.kSquishyWheelAlgaeSpeed)
            else:
                ntutil.log(f"ERROR! Bad intake mode: {self.intakeMode}")
                self.armOuterWheelMotor.set(0)
                self.armInnerWheelMotor.set(0)

            if self.previousCoralDetected and not self.coral_detected():
                self.intakeState = "stopped"
        elif self.intakeState == "stopped":
            self.armOuterWheelMotor.set(0)
            self.armInnerWheelMotor.set(0)
            
            if self.intakeSpeed == 0:
                self.intakeState = "initial"
        self.previousCoralDetected = self.coral_detected()

        self.intakeSensorTopic.set(self.intakeSensor.getAverageVoltage())
        self.coralDetectedTopic.set(self.coral_detected())
        self.armInnerWheelSpeedTopic.set(self.armInnerWheelEncoder.getVelocity())
        self.armOuterWheelSpeedTopic.set(self.armOuterWheelEncoder.getVelocity())

        # Update Mechanism2d telemetry
        self.mechActual.update(elevatorHeight=self.get_elevator_position(), armAngle=self.get_wrist_position())
        self.mechDesired.update(elevatorHeight=slewedElevatorSetpoint, armAngle=safeWristPosition)

    def set_arm_position(self, height: float, angle: float, mode: int):
        """
        Directly set the target height and angle of the arm. The mode controls
        what part of the arm will be moved to the desired height - either the
        center of the wheels for coral, or the end of the arm for algae.

        Consider using `go_to_coral_preset`, `go_to_algae_floor_preset`, or
        `go_to_algae_dereef_preset` instead.
        """
        self.wristPositionSetpoint = angle
        radius = None
        if mode == constants.kCoralMode:
            radius = constants.kArmCoralRadius
        elif mode == constants.kAlgaeMode:
            radius = constants.kArmAlgaeRadius
        else:
            print("ERROR! Unknown mode!")
        self.elevatorSetpoint = self.compute_elevator_height(height, angle, radius)

    def go_to_coral_preset(self, level: int):
        coralPresets = {
            1: (constants.kElevatorL1, constants.kWristUprightAngle),
            2: (constants.kElevatorL2, constants.kWristCoralScoreAngle),
            3: (constants.kElevatorL3, constants.kWristCoralScoreAngle),
            4: (constants.kElevatorL4, constants.kWristHighCoralScoreAngle),
        }

        if level not in coralPresets:
            ntutil.logAlert(self.badCoralLevelAlert, level)
            return
        elevatorHeight, armAngle = coralPresets[level]
        self.set_arm_position(elevatorHeight, armAngle, constants.kCoralMode)

    def go_to_algae_floor_preset(self):
        self.set_arm_position(constants.kElevatorAlgaeGround, constants.kWristAlgaeGround, constants.kAlgaeMode)

    def go_to_algae_dereef_preset(self, high: bool):
        if high:
            elevatorHeight = constants.kElevatorAlgaeHigh
        else:
            elevatorHeight = constants.kElevatorAlgaeLow
        self.set_arm_position(elevatorHeight, constants.kWristAlgaeDereef, constants.kAlgaeMode)

    def move_coral(self, speed: float):
        """
        Spins the squishy wheels in opposite directions to move coral. Speeds range from
        -1 to 1, where positive means intake -> reef. Speed will be limited by the top
        speed in constants.py.
        """
        self.intakeSpeed = speed
        self.intakeMode = "coral"
        pass

    def move_algae(self, speed: float):
        """
        Spins the squishy wheels in the same direction to move algae. Speeds range from
        -1 to 1, where positive means robot -> processor. Speed will be limited by the
        top speed in constants.py.
        """
        self.intakeSpeed = speed
        self.intakeMode = "algae"
        pass

    def get_wrist_position(self) -> float:
        """
        Get the current wrist position in radians.
        """
        return wristEncoderToAngle(self.wristEncoder.getPosition())
    
    def get_elevator_position(self) -> float:
        """
        Get the current elevator position in meters.
        """
        return self.elevatorEncoder.getPosition()
    
    def get_safe_wrist_position(self, setpoint) -> float:
        """
        Takes a setpoint in radians and returns a safe position based on the 
        current elevator height. The setpoint is clamped to the range of angles allowed
        for the current elevator height. If the elevator height is outside all ranges,
        the always safe angle is returned.
        """
        # First, find what constraints apply to the current elevator height
        elevatorHeight = self.get_elevator_position()
        for heightRange, (minAngle, maxAngle) in constants.kArmSafetyConstraints.items():
            if heightRange[0] <= elevatorHeight < heightRange[1]:
                return utils.clamp(setpoint, minAngle, maxAngle)
        else:
            # If the elevator height is outside all constraint ranges, return the always safe angle
            # TODO: Replace this with a proper logging solution that goes to AdvantageScope
            print("ERROR! We should not have a gap in our arm angle limits!")
            return constants.kArmAlwaysSafeAngle

    def calculate_arm_ff(self):
        """
        Calculates the arm feedforward based on the current wrist angle
        """

        wristAngle = self.get_wrist_position()

        # Calculate the arm feedforward
        armFF = ArmFeedforward(constants.kArmKS, constants.kArmKG, constants.kArmKV, constants.kArmKA)

        ffVoltage = armFF.calculate(wristAngle + wpimath.units.degreesToRadians(90), self.wristEncoder.getVelocity())

        return ffVoltage

    def compute_elevator_height(self, armHeight, angle, radius):
        return armHeight - constants.kArmHeightInCarriage - constants.kElevatorBaseHeight - radius * math.cos(angle)
    
    def coral_detected(self):
        if 1.5 < self.intakeSensor.getAverageVoltage() < 2.5:
            return True
        else:
            return False

    class Mechanism:
        """
        A utility class that creates a Mechanism2d for display in AdvantageScope.
        This makes it easy to display both the desired and actual mechanisms.
        """
        def __init__(self, name: str, color: wpilib.Color):
            canvasWidth = 2 # m
            canvasHeight = 2.5 # m
            self.mech = wpilib.Mechanism2d(width=canvasWidth, height=canvasHeight)
            self.root = self.mech.getRoot("Scorer",
                x=canvasWidth/2 + wpimath.units.inchesToMeters(7),
                y=0,
            )
            self.elevator = self.root.appendLigament("Elevator",
                length=constants.kElevatorBaseHeight + constants.kArmHeightInCarriage,
                angle=90, # Mechanism2d uses degrees for reasons unknown
                color=wpilib.Color8Bit(color),
            )
            self.wrist = self.elevator.appendLigament("Wrist",
                length=constants.kArmAlgaeRadius,
                angle=0,
                color=wpilib.Color8Bit(color),
            )
            SmartDashboard.putData(name, self.mech)

        def update(self, elevatorHeight, armAngle):
            self.elevator.setLength(constants.kElevatorBaseHeight + elevatorHeight + constants.kArmHeightInCarriage)
            self.wrist.setAngle(wpimath.units.radiansToDegrees(armAngle))
