import ntcore
import wpilib
import wpiutil.log

nt = ntcore.NetworkTableInstance.getDefault()
log = wpilib.DataLogManager.getLog()

# Plain NetworkTables topics

class _NTTopic:
    def __init__(self, topic, default):
        self.publisher = topic.publish()
        self.subscriber = topic.subscribe(default)

    def get(self, defaultValue=None):
        if defaultValue is None:
            return self.subscriber.get()
        else:
            return self.subscriber.get(defaultValue)

    def set(self, value):
        self.publisher.set(value)

    def setDefault(self, value):
        self.publisher.setDefault(value)

def getBooleanArrayTopic(name: str, defaultValue=[]) -> _NTTopic:
    return _NTTopic(nt.getBooleanArrayTopic(name), defaultValue)

def getBooleanTopic(name: str, defaultValue=False) -> _NTTopic:
    return _NTTopic(nt.getBooleanTopic(name), defaultValue)

def getDoubleArrayTopic(name: str, defaultValue=[]) -> _NTTopic:
    return _NTTopic(nt.getDoubleArrayTopic(name), defaultValue)

def getDoubleTopic(name: str, defaultValue=0) -> _NTTopic:
    return _NTTopic(nt.getDoubleTopic(name), defaultValue)

def getFloatArrayTopic(name: str, defaultValue=[]) -> _NTTopic:
    return _NTTopic(nt.getFloatArrayTopic(name), defaultValue)

def getFloatTopic(name: str, defaultValue=0) -> _NTTopic:
    return _NTTopic(nt.getFloatTopic(name), defaultValue)

def getIntegerArrayTopic(name: str, defaultValue=[]) -> _NTTopic:
    return _NTTopic(nt.getIntegerArrayTopic(name), defaultValue)

def getIntegerTopic(name: str, defaultValue=0) -> _NTTopic:
    return _NTTopic(nt.getIntegerTopic(name), defaultValue)

def getStringArrayTopic(name: str, defaultValue=[]) -> _NTTopic:
    return _NTTopic(nt.getStringArrayTopic(name), defaultValue)

def getStringTopic(name: str, defaultValue="") -> _NTTopic:
    return _NTTopic(nt.getStringTopic(name), defaultValue)

def getStructArrayTopic(name: str, type: type, defaultValue=[]) -> _NTTopic:
    return _NTTopic(nt.getStructArrayTopic(name, type), defaultValue)

def getStructTopic(name: str, type: type, defaultValue=None) -> _NTTopic:
    if defaultValue is None:
        defaultValue = type()
    return _NTTopic(nt.getStructTopic(name, type), defaultValue)

# Log entries

def getBooleanArrayLog(name: str):
    return wpiutil.log.BooleanArrayLogEntry(log, name)

def getBooleanLog(name: str):
    return wpiutil.log.BooleanLogEntry(log, name)

def getDoubleArrayLog(name: str):
    return wpiutil.log.DoubleArrayLogEntry(log, name)

def getDoubleLog(name: str):
    return wpiutil.log.DoubleLogEntry(log, name)

def getFloatArrayLog(name: str):
    return wpiutil.log.FloatArrayLogEntry(log, name)

def getFloatLog(name: str):
    return wpiutil.log.FloatLogEntry(log, name)

def getIntegerArrayLog(name: str):
    return wpiutil.log.IntegerArrayLogEntry(log, name)

def getIntegerLog(name: str):
    return wpiutil.log.IntegerLogEntry(log, name)

def getStringArrayLog(name: str):
    return wpiutil.log.StringArrayLogEntry(log, name)

def getStringLog(name: str):
    return wpiutil.log.StringLogEntry(log, name)

def getStructArrayLog(name: str, type: type):
    return wpiutil.log.StructArrayLogEntry(log, name, type)

def getStructLog(name: str, type: type):
    return wpiutil.log.StructLogEntry(log, name, type)
