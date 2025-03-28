import math
import random

import hal.simulation
import wpimath
import wpimath.units
from rev import SparkMaxSim
from wpilib import RobotController
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.system.plant import DCMotor

import ntutil
from subsystems.drivetrain import Drivetrain
from swervemodule import SwerveModule

# Taken from:
# https://github.com/mythgarr/robotpy-rev-examples/blob/main/SPARK/MAXSwerve/sim/swerve_drive_sim.py

class SwerveModuleSim:
    def __init__(self, module: SwerveModule):
        self.driveMotorSim = SparkMaxSim(module.driveMotor, DCMotor.NEO(1))
        self.steerMotorSim = SparkMaxSim(module.steerMotor, DCMotor.NEO(1))
        self.angleOffset = module.angleOffset

        # Calculates the velocity of the drive motor, roughly simulating
        # inertia and friction. Note that this limits the SPEED of the motors.
        # Linear change to velocity means constant acceleration, so the units
        # here check out.
        self.driveSpeedLimiter = SlewRateLimiter(9.8 * 5) # 5 G's (m/s^2)

        # Use a PID controller to simulate velocity of the steer motor
        self.steerController = PIDController(100, 0, 0) # magic numbers!
        self.steerController.enableContinuousInput(-math.pi, math.pi)

        # Randomize the starting rotation to simulate what we have when we take
        # the field
        self.steerMotorSim.setPosition(random.uniform(-math.pi, math.pi))

    def simulationPeriodic(self, vbus: float, dt: float):
        targetSpeed = self.driveMotorSim.getSetpoint()
        driveSpeed = self.driveSpeedLimiter.calculate(targetSpeed)
        self.driveMotorSim.iterate(driveSpeed, vbus, dt)

        targetAngle = self.steerMotorSim.getSetpoint()
        currentAngle = wpimath.angleModulus(self.steerMotorSim.getPosition())
        self.steerController.setSetpoint(targetAngle)
        steerSpeed = self.steerController.calculate(currentAngle)
        self.steerMotorSim.iterate(steerSpeed, vbus, dt)
    
    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            wpimath.units.meters_per_second(self.driveMotorSim.getVelocity()),
            Rotation2d(self.steerMotorSim.getPosition() - self.angleOffset)
        )


class SwerveDriveSim:
    def __init__(self, drivetrain: Drivetrain):
        self.kinematics = drivetrain.kinematics
        self.modules = (
            SwerveModuleSim(drivetrain.frontLeftSwerveModule),
            SwerveModuleSim(drivetrain.frontRightSwerveModule),
            SwerveModuleSim(drivetrain.backLeftSwerveModule),
            SwerveModuleSim(drivetrain.backRightSwerveModule),
        )

        # Per the navX docs, the recommended way to use the navX as a sim
        # device is to just use the real device but set the simulator variable
        # for Yaw directly.
        #
        # https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/c/
        navXDevice = hal.simulation.getSimDeviceHandle(f"navX-Sensor[{drivetrain.gyro.getPort()}]")
        self.navXAngleSim = hal.SimDouble(hal.simulation.getSimValueHandle(navXDevice, "Yaw"))

        self.pose = Pose2d()
        self.poseTopic = ntutil.getStructTopic("/Sim/Pose", Pose2d)
    
    def simulationPeriodic(self, dt: float):
        vbus = RobotController.getBatteryVoltage()
        for module in self.modules:
            module.simulationPeriodic(vbus, dt)
        moduleStates = (
            self.modules[0].getState(),
            self.modules[1].getState(),
            self.modules[2].getState(),
            self.modules[3].getState(),
        )
        chassisSpeeds = self.kinematics.toChassisSpeeds(moduleStates)
        self.pose = self.pose.exp(chassisSpeeds.toTwist2d(dt))
        self.poseTopic.set(self.pose)

        # Update the simulated gyro. For reasons unknown, the navX uses
        # clockwise degrees.
        self.navXAngleSim.set(-self.pose.rotation().degrees())

    def getPose(self):
        return self.pose

    def setPose(self, pose: Pose2d):
        self.pose = pose
