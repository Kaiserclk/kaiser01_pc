import os
from ament_index_python.packages import get_package_share_directory


def GetCameraType():
    try:
        return os.getenv("CAMERA_TYPE")
    except:
        raise Exception("CAMERA_TYPE environment variable is not set. ")

def GetLidarType():
    try:
        return os.getenv("RPLIDAR_TYPE")
    except:
        raise Exception("RPLIDAR_TYPE environment variable is not set. ")

def GetMachineType():
    try:
        machine_type = os.getenv("MACHINE")
        if machine_type not in ["OrinNx", "OrinNano", "RaspberryPi5","RDKX5"]:
            print("Invalid MACHINE_TYPE environment variable. ")
            return None
        return machine_type
    except:
        print("MACHINE_TYPE environment variable is not set.")
        return None   # Default machine type

def GetMapDirectory():
    return os.path.join(os.path.expanduser('~'), 'map')



def GetCameraRgbTopic()->str|None:
    camera_type=GetCameraType()
    if camera_type is None:
        return None
    if camera_type=="nuwa":
        return "/ascamera_hp60c/camera_publisher/rgb0/image"
    elif camera_type=="usb":
        return "/camera/color/image_raw"
    else:
        return None

def GetCameraDepthTopic()->str|None:
    camera_type=GetCameraType()
    if camera_type is None:
        return None
    if camera_type=="nuwa":
        return "/ascamera_hp60c/camera_publisher/depth0/image_raw"
    elif camera_type=="usb":
        return "/camera/depth/image_raw"
    else:
        return None


def GetSmartTrackModelPath()->tuple[bool, str]:
    '''Get the path of the smart track model'''
    machine=GetMachineType()
    if machine is None:
        return (False, "MACHINE environment variable is not set")
    if machine in ["OrinNx", "OrinNano"]:
        model_path=os.path.join(os.path.expanduser('~'),'MODELS','tracker','super_track.engine')
    elif machine=="RaspberryPi5":
        model_path=os.path.join(os.path.expanduser('~'),'MODELS','tracker','super_track.onnx')
    elif machine=="RDKX5":
        model_path=os.path.join(os.path.expanduser('~'),'MODELS','tracker','super_track.bin')
    else:
        return (False, f"Invalid machine type: {machine}")

    if not os.path.isfile(model_path):
        return (False, f"Model file not found: {model_path}")
    return (True, model_path)

def GetSmartTrackInferDevice()->tuple[bool, str]:
    '''Get the device for SmartTrack inference'''
    machine=GetMachineType()
    if machine is None:
        return (False, "MACHINE environment variable is not set")
    if machine in ["OrinNx", "OrinNano"]:
        return (True, "gpu")
    elif machine=="RaspberryPi5":
        return (True, "cpu")
    elif machine=="RDKX5":
        return (True, "npu")
    else:
        print(f"Invalid machine type: {machine}, use cpu as default")
        return (False, "cpu")





def GetYolo26ModelPath()->tuple[bool, str]:
    '''Get the path of the waste_classify yolo26 OBB model'''
    machine=GetMachineType()
    if machine is None:
        return (False, "MACHINE environment variable is not set")
    
    if machine=="OrinNx" or machine=="OrinNano":
        model_path=os.path.join(os.path.expanduser('~'),'MODELS','waste_classify','WasteSort-v2.engine')
    elif machine=="RaspberryPi5":
        model_path=os.path.join(os.path.expanduser('~'),'MODELS','waste_classify','WasteSort-v2.onnx')
    elif machine=="RDKX5":
        model_path=os.path.join(os.path.expanduser('~'),'MODELS','waste_classify','WasteSort-v2.bin')
    else:
        return (False, f"Invalid machine type: {machine}")

    if not os.path.isfile(model_path):
        return (False, f"Model file not found: {model_path}")
    return (True, model_path)

def GetYoloInferDevice()->tuple[bool, str]:
    '''Get the device for yolo inference'''
    machine=GetMachineType()
    if machine is None:
        return (False, "MACHINE environment variable is not set")
    if machine=="OrinNx" or machine=="OrinNano":
        return (True, "cuda:0")
    elif machine=="RaspberryPi5":
        return (True, "cpu")
    elif machine=="RDKX5":
        return (True, "npu")
    else:
        print(f"Invalid machine type: {machine}, use cpu as default")
        return (False, "cpu")
    
    
def GetCameraLink()->str:
    '''Get the camera link name for tf'''
    camera_type=GetCameraType()
    if camera_type is None:
        raise Exception("CAMERA_TYPE environment variable is not set. ")
    if camera_type=="nuwa":
        return "ascamera_hp60c_camera_link_0"
    elif camera_type=="usb":
        return "camera_link"
    else:
        raise Exception("Invalid camera type. Please set CAMERA_TYPE environment variable to 'nuwa' or 'usb'")
    
    
def GetDepthCameraLink()->str:
    '''Get the depth camera link name for tf'''
    camera_type=GetCameraType()
    if camera_type is None:
        raise Exception("CAMERA_TYPE environment variable is not set. ")
    if camera_type=="nuwa":
        return "ascamera_hp60c_color_0"
    elif camera_type=="usb":
        return "camera_link"
    else:
        raise Exception("Invalid camera type. Please set CAMERA_TYPE environment variable to 'nuwa' or 'usb'")
    
    
def GenerateRobotDescription(robot_urdf: str)->str:
    '''Generate the robot description '''
    from launch.substitutions import Command
    laser_type = GetLidarType()
    camera_type = GetCameraType()
    robot_description = Command(
        [
            "xacro",
            " ",
            robot_urdf,
            " ",
            "laser_type:=",
            laser_type,
            " ",
            "camera_type:=",
            camera_type,
        ]
    )
    return robot_description
    