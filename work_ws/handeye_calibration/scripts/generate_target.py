#!/usr/bin/env python3
"""
生成 ArUco 手眼标定板图像
使用方法: python3 generate_target.py
"""

import cv2
import numpy as np
import os

def generate_aruco_target():
    """生成 ArUco 标定板图像"""
    
    # 配置参数(与程序中的配置一致)
    markers_x = 4          # X方向标记数
    markers_y = 3          # Y方向标记数
    marker_size = 200      # 标记大小(像素)
    separation = 20        # 间距(像素)
    dictionary_id = cv2.aruco.DICT_4X4_250
    
    print("===========================================")
    print("生成 ArUco 手眼标定板")
    print("===========================================")
    print(f"标记数量: {markers_x} x {markers_y}")
    print(f"标记大小: {marker_size} px")
    print(f"标记间距: {separation} px")
    print(f"字典类型: DICT_4X4_250")
    print()
    
    # 创建字典和标定板
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    
    # OpenCV 4.x 使用不同的 API
    try:
        # OpenCV 4.7+
        board = cv2.aruco.GridBoard(
            size=(markers_x, markers_y),
            markerLength=marker_size,
            markerSeparation=separation,
            dictionary=dictionary
        )
    except AttributeError:
        # OpenCV 4.6 及以下版本
        board = cv2.aruco.GridBoard_create(
            markers_x, markers_y, marker_size, separation, dictionary
        )
    
    # 计算图像尺寸
    img_width = markers_x * (marker_size + separation) - separation + 2 * separation
    img_height = markers_y * (marker_size + separation) - separation + 2 * separation
    
    # 生成图像
    img = board.generateImage((img_width, img_height), marginSize=separation, borderBits=1)
    
    # 保存图像
    output_file = "/tmp/handeye_calibration_target.png"
    cv2.imwrite(output_file, img)
    
    print(f"✓ 标定板图像已保存到: {output_file}")
    print(f"✓ 图像尺寸: {img_width} x {img_height} 像素")
    print()
    print("下一步:")
    print("1. 打印标定板图像(使用高质量打印机)")
    print("2. 精确测量实际标记大小和间距(单位:米)")
    print("3. 修改程序中的参数以匹配实际尺寸:")
    print("   - measured marker size (m): 实际标记大小")
    print("   - measured separation (m): 实际间距")
    print()
    print("===========================================")
    
    # 显示图像
    cv2.imshow("ArUco Calibration Target", img)
    print("按任意键关闭窗口...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    generate_aruco_target()
