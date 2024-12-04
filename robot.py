# TODO: insert robot code here

import wpilib
import rev

testMotorID = 21

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.gamePad = wpilib.XboxController(0)

        self.testMotor = rev.CANSparkMax(testMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.testMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

    def teleopPeriodic(self) -> None:
        self.testMotor.set(self.gamePad.getLeftY)