#!/usr/bin/env python3
"""
测试 OpenCV 在 Docker 容器中是否能打开窗口显示图片
"""

import cv2
import sys
import os

def test_opencv_window():
    # 图片路径
    image_path = os.path.join(os.path.dirname(__file__), "11.png")
    
    print("=" * 50)
    print("OpenCV 窗口显示测试")
    print("=" * 50)
    print(f"图片路径: {image_path}")
    print()
    
    # 检查图片是否存在
    if not os.path.exists(image_path):
        print(f"❌ 错误: 找不到图片文件 {image_path}")
        return False
    
    # 读取图片
    print("正在读取图片...")
    img = cv2.imread(image_path)
    
    if img is None:
        print("❌ 错误: 无法读取图片")
        return False
    
    print(f"✓ 图片读取成功，尺寸: {img.shape[1]}x{img.shape[0]}")
    print()
    
    # 尝试创建窗口并显示
    print("正在创建窗口...")
    try:
        cv2.namedWindow("Test Window", cv2.WINDOW_AUTOSIZE)
        print("✓ 窗口创建成功")
        print()
        print("正在显示图片...")
        print()
        
        cv2.imshow("Test Window", img)
        
        # 等待 2 秒，看窗口是否真的能显示
        print("等待 2 秒，观察窗口是否出现...")
        key = cv2.waitKey(2000) & 0xFF
        
        if key != 0xFF:  # 有按键
            if key == ord('q') or key == 27:
                print("用户按 'q' 或 ESC 退出")
            else:
                print(f"用户按键: {chr(key) if 32 <= key <= 126 else key}")
        else:
            print("(超时，自动继续)")
        
        cv2.destroyAllWindows()
        print("✓ 窗口已关闭")
        print()
        print("=" * 50)
        print("✅ 测试成功! OpenCV 窗口显示功能正常")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"❌ 错误: 无法创建窗口或显示图片")
        print(f"错误信息: {type(e).__name__}: {e}")
        print()
        print("=" * 50)
        print("❌ 测试失败! Docker 容器不支持 GUI 显示")
        print("=" * 50)
        print()
        print("建议:")
        print("1. 检查 DISPLAY 环境变量是否设置")
        print("2. 检查 X11 转发是否启用")
        print("3. 使用 ROS2 话题发布图像，在宿主机查看")
        return False

if __name__ == "__main__":
    success = test_opencv_window()
    sys.exit(0 if success else 1)
