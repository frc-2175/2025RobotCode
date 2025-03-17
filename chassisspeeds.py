import math
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds

class ChassisSpeeds2175:
    """
    A replacement for WPILib's ChassisSpeeds that stores angle/speed instead
    of vx/vy. This prevents us from forgetting what direction we were traveling
    when we set our speed to zero.
    """

    def __init__(self, direction: float, speed: float, omega: float):
        self.direction: float = direction # rad
        self.speed: float = speed # m/s
        self.omega: float = omega # rad/s
    
    def fromWPILibChassisSpeeds(speeds: ChassisSpeeds):
        direction = math.atan2(speeds.vy, speeds.vx)
        speed = math.sqrt(speeds.vx**2 + speeds.vy**2)
        return ChassisSpeeds2175(direction, speed, speeds.omega)
    
    def fromFieldRelativeSpeeds(vx: float, vy: float, omega: float, robotAngle: Rotation2d):
        wpilibSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotAngle)
        return ChassisSpeeds2175.fromWPILibChassisSpeeds(wpilibSpeeds)

    @property
    def vx(self) -> float:
        return self.speed * math.cos(self.direction)

    @property
    def vy(self) -> float:
        return self.speed * math.sin(self.direction)

    def toWPILibChassisSpeeds(self) -> ChassisSpeeds:
        return ChassisSpeeds(self.vx, self.vy, self.omega)
