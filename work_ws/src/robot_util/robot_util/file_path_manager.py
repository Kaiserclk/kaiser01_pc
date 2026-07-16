
import os
import sys
from ament_index_python.packages import get_package_share_directory
class FilePathManager:
    '''A class to manage file paths for the project.'''
    def __init__(self):
        self.file_path_dict = {

            "robot_urdf": os.path.join(get_package_share_directory("kai01_description"),"urdf","kai01.urdf.xacro"), # robot urdf file
        }
        
    def GetFilePath(self, file_name):
        return self.file_path_dict.get(file_name)
    
    def GetFileAndCheckExist(self, file_name):
        '''Get the file path and check if it exists.'''
        file_path = self.GetFilePath(file_name)
        if file_path is None:
            raise ValueError(f"File name '{file_name}' not found in file path manager.")
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"File '{file_path}' does not exist.")
        return file_path