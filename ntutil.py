import typing
from typing import Any, Dict, Generic, List, Type, TypeVar

import ntcore
import wpilib
import wpiutil.log

nt = ntcore.NetworkTableInstance.getDefault()

# Plain NetworkTables topics

T = TypeVar("T")

class _NTTopic(Generic[T]):
    def __init__(self, topic, default: T):
        self.publisher = topic.publish()
        self.subscriber = topic.subscribe(default)

    def get(self, defaultValue: T | None = None) -> T:
        if defaultValue is None:
            return self.subscriber.get()
        else:
            return self.subscriber.get(defaultValue)

    def set(self, value: T) -> None:
        self.publisher.set(value)

    def setDefault(self, value: T) -> None:
        self.publisher.setDefault(value)

def getBooleanArrayTopic(name: str, defaultValue: List[bool] = []) -> _NTTopic[List[bool]]:
    return _NTTopic(nt.getBooleanArrayTopic(name), defaultValue)

def getBooleanTopic(name: str, defaultValue: bool = False) -> _NTTopic[bool]:
    return _NTTopic(nt.getBooleanTopic(name), defaultValue)

def getFloatArrayTopic(name: str, defaultValue: List[float] = []) -> _NTTopic[List[float]]:
    return _NTTopic(nt.getFloatArrayTopic(name), defaultValue)

def getFloatTopic(name: str, defaultValue: float = 0) -> _NTTopic[float]:
    return _NTTopic(nt.getFloatTopic(name), defaultValue)

def getIntegerArrayTopic(name: str, defaultValue: List[int] = []) -> _NTTopic[List[int]]:
    return _NTTopic(nt.getIntegerArrayTopic(name), defaultValue)

def getIntegerTopic(name: str, defaultValue: int = 0) -> _NTTopic[int]:
    return _NTTopic(nt.getIntegerTopic(name), defaultValue)

def getStringArrayTopic(name: str, defaultValue: List[str] = []) -> _NTTopic[List[str]]:
    return _NTTopic(nt.getStringArrayTopic(name), defaultValue)

def getStringTopic(name: str, defaultValue: str = "") -> _NTTopic[str]:
    return _NTTopic(nt.getStringTopic(name), defaultValue)

def getStructArrayTopic(name: str, type: Type[T], defaultValue: List[T] = []) -> _NTTopic[List[T]]:
    return _NTTopic(nt.getStructArrayTopic(name, type), defaultValue)

def getStructTopic(name: str, type: Type[T], defaultValue: T | None = None) -> _NTTopic[T]:
    if defaultValue is None:
        defaultValue = type()
    return _NTTopic(nt.getStructTopic(name, type), defaultValue)

# Log entries

def log(msg: Any):
    """
    Logs a message to both stdout and the /messages entry of NetworkTables
    (visible when viewing log files).
    """
    wpilib.DataLogManager.log(f"{msg}")

originalAlertText: Dict[wpilib.Alert, str] = {}

def logAlert(alert: wpilib.Alert, msg: Any):
    """
    Enables an Alert and also logs an additional message. Useful for indicating
    a specific value that caused an error, or to provide more context. Consider
    using this whenever you would instead use a simple `ntutil.log`.

    To avoid spam, this method will only log when the message changes.
    """
    global originalAlertText
    
    if alert not in originalAlertText:
        originalAlertText[alert] = alert.getText()
    
    fullMsg = f"{originalAlertText[alert]}: {msg}"
    if alert.getText() != fullMsg:
        log(fullMsg)
        alert.setText(fullMsg)
    alert.set(True)

def getBooleanArrayLog(name: str):
    return wpiutil.log.BooleanArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getBooleanLog(name: str):
    return wpiutil.log.BooleanLogEntry(wpilib.DataLogManager.getLog(), name)

def getDoubleArrayLog(name: str):
    return wpiutil.log.DoubleArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getDoubleLog(name: str):
    return wpiutil.log.DoubleLogEntry(wpilib.DataLogManager.getLog(), name)

def getFloatArrayLog(name: str):
    return wpiutil.log.FloatArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getFloatLog(name: str):
    return wpiutil.log.FloatLogEntry(wpilib.DataLogManager.getLog(), name)

def getIntegerArrayLog(name: str):
    return wpiutil.log.IntegerArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getIntegerLog(name: str):
    return wpiutil.log.IntegerLogEntry(wpilib.DataLogManager.getLog(), name)

def getStringArrayLog(name: str):
    return wpiutil.log.StringArrayLogEntry(wpilib.DataLogManager.getLog(), name)

def getStringLog(name: str):
    return wpiutil.log.StringLogEntry(wpilib.DataLogManager.getLog(), name)

def getStructArrayLog(name: str, type: type):
    return wpiutil.log.StructArrayLogEntry(wpilib.DataLogManager.getLog(), name, type)

def getStructLog(name: str, type: type):
    return wpiutil.log.StructLogEntry(wpilib.DataLogManager.getLog(), name, type)
