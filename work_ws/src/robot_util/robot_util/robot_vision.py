import re
from robot_util.file_path_manager import FilePathManager


def SetHSV(color: str, hsv: list[int]) -> tuple[bool, str]:
    '''set HSV value to param file'''
    valid_colors = {'blue', 'green', 'red', 'yellow'}
    if color not in valid_colors:
        return False, f"Invalid color '{color}', supported: {sorted(valid_colors)}"
    if len(hsv) != 6:
        return False, f"HSV must contain 6 integers, got {len(hsv)}"
    for i, v in enumerate(hsv):
        if not isinstance(v, int):
            return False, f"HSV value #{i+1} must be int, got {type(v).__name__}"

    try:
        file_manager = FilePathManager()
        common_param_file = file_manager.GetFileAndCheckExist("config_file")
    except Exception as e:
        return False, f"Config file error: {e}"

    with open(common_param_file, 'r') as f:
        content = f.read()

    hsv_str = '[' + ','.join(str(v) for v in hsv) + ']'
    pattern = re.compile(rf'(^    {color}_hsv:\s*)\[.*\]', re.MULTILINE)
    new_content = pattern.sub(rf'\g<1>{hsv_str}', content)

    if new_content == content:
        return False, f"'{color}_hsv' field not found in {common_param_file}"

    with open(common_param_file, 'w') as f:
        f.write(new_content)
    return True, ""


def SetLAB(color: str, lab: list[int]) -> tuple[bool, str]:
    '''set LAB value to param file'''
    valid_colors = {'blue', 'green', 'red', 'yellow'}
    if color not in valid_colors:
        return False, f"Invalid color '{color}', supported: {sorted(valid_colors)}"
    if len(lab) != 6:
        return False, f"LAB must contain 6 integers, got {len(lab)}"
    for i, v in enumerate(lab):
        if not isinstance(v, int):
            return False, f"LAB value #{i+1} must be int, got {type(v).__name__}"

    try:
        file_manager = FilePathManager()
        common_param_file = file_manager.GetFileAndCheckExist("config_file")
    except Exception as e:
        return False, f"Config file error: {e}"

    with open(common_param_file, 'r') as f:
        content = f.read()

    lab_str = '[' + ','.join(str(v) for v in lab) + ']'
    pattern = re.compile(rf'(^    {color}_lab:\s*)\[.*\]', re.MULTILINE)
    new_content = pattern.sub(rf'\g<1>{lab_str}', content)

    if new_content == content:
        return False, f"'{color}_lab' field not found in {common_param_file}"

    with open(common_param_file, 'w') as f:
        f.write(new_content)
    return True, ""