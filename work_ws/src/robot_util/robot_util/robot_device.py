import os
from dataclasses import dataclass
from typing import Optional, Dict, List
from abc import ABC, abstractmethod


@dataclass
class DeviceStatus:
    """Device status data class"""
    device_name: str
    device_path: str
    connected: bool
    message: str = ""
    details: dict = None

    def __post_init__(self):
        if self.details is None:
            self.details = {}


class DeviceDetector(ABC):
    """Base class for device detection"""

    def __init__(self, device_name: str, device_path: str):
        self.device_name = device_name
        self.device_path = device_path

    @abstractmethod
    def check(self) -> DeviceStatus:
        """Check if device is connected"""
        pass

    def _check_path_exists(self) -> bool:
        """Check if device path exists"""
        return os.path.exists(self.device_path)


class SpeechDeviceDetector(DeviceDetector):
    """Detect speech module (/dev/myspeech)"""

    def __init__(self):
        super().__init__('Speech Module', '/dev/myspeech')

    def check(self) -> DeviceStatus:
        connected = self._check_path_exists()
        
        if connected:
            # Get device info if available
            try:
                stat = os.stat(self.device_path)
                details = {
                    'mode': oct(stat.st_mode),
                    'owner_uid': stat.st_uid,
                    'group_gid': stat.st_gid
                }
                message = "Speech module connected"
            except Exception as e:
                details = {'error': str(e)}
                message = f"Speech module connected but cannot access: {e}"
        else:
            details = {}
            message = "Speech module not found"
        
        return DeviceStatus(
            device_name=self.device_name,
            device_path=self.device_path,
            connected=connected,
            message=message,
            details=details
        )


class LidarDeviceDetector(DeviceDetector):
    """Detect lidar device (/dev/ydlidar)"""

    def __init__(self):
        super().__init__('YDLidar', '/dev/ydlidar')

    def check(self) -> DeviceStatus:
        connected = self._check_path_exists()
        
        if connected:
            try:
                stat = os.stat(self.device_path)
                details = {
                    'mode': oct(stat.st_mode),
                    'owner_uid': stat.st_uid,
                    'group_gid': stat.st_gid
                }
                message = "YDLidar connected"
            except Exception as e:
                details = {'error': str(e)}
                message = f"YDLidar connected but cannot access: {e}"
        else:
            details = {}
            message = "YDLidar not found"
        
        return DeviceStatus(
            device_name=self.device_name,
            device_path=self.device_path,
            connected=connected,
            message=message,
            details=details
        )


# Device detector registry
DEVICE_DETECTORS = {
    'speech': SpeechDeviceDetector,
    'lidar': LidarDeviceDetector
}


def print_status(status: DeviceStatus, verbose: bool = False):
    """Print device status"""
    if status.connected:
        print(f"✅ {status.device_name} ({status.device_path}): {status.message}")
    else:
        print(f"❌ {status.device_name} ({status.device_path}): {status.message}")
    
    if verbose and status.details:
        for key, value in status.details.items():
            print(f"   {key}: {value}")


def check_device(device_name: str, verbose: bool = False) -> DeviceStatus:
    """
    Check specified device
    
    Args:
        device_name: Device name (speech, lidar)
        verbose: Print detailed info
    
    Returns:
        DeviceStatus: Device status
    """
    if device_name not in DEVICE_DETECTORS:
        return DeviceStatus(
            device_name=device_name,
            device_path='unknown',
            connected=False,
            message=f"Unknown device: {device_name}, supported: {list(DEVICE_DETECTORS.keys())}"
        )
    
    detector = DEVICE_DETECTORS[device_name]()
    status = detector.check()
    return status


def check_speech(verbose: bool = False) -> DeviceStatus:
    """Check speech module"""
    return check_device('speech', verbose)


def check_lidar(verbose: bool = False) -> DeviceStatus:
    """Check lidar device"""
    return check_device('lidar', verbose)


def check_all_devices(verbose: bool = False) -> Dict[str, DeviceStatus]:
    """
    Check all registered devices
    
    Returns:
        dict: Device name -> DeviceStatus
    """
    results = {}
    print("Checking all devices...")
    print("-" * 50)
    
    for name in DEVICE_DETECTORS:
        results[name] = check_device(name, verbose)
    
    print("-" * 50)
    
    # Summary
    connected = sum(1 for s in results.values() if s.connected)
    total = len(results)
    print(f"Summary: {connected}/{total} devices connected")
    
    return results



