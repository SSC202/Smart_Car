"""颜色识别工程
    @Time   2023.10.24
    @Author SSC
"""

import numpy as np
import cv2
import serial
import time
import struct


# 全局定义段
# 1. 饱和度增强定义
# 调节通道强度
lutEqual = np.array([i for i in range(256)]).astype("uint8")
lutRaisen = np.array([int(102+0.6*i) for i in range(256)]).astype("uint8")
# 调节饱和度
lutSRaisen = np.dstack((lutEqual, lutRaisen, lutEqual))  # Saturation raisen
# 2. 掩膜阈值定义
lower_ball = np.array([104, 247, 173])
upper_ball = np.array([129, 255, 255])
# 3. 结构元素定义
kernel = np.ones((3, 3), np.uint8)
# 4. Serial Port Definition
serial_port = serial.Serial("/dev/ttyACM0",115200,timeout=0.5)
serial_port_state = serial_port.is_open
# 5. Capture Definition
cap = cv2.VideoCapture(0)
cap.set(10, -2)
# 6. Ball Definition
ball_x = 0.0
ball_y = 0.0
ball_r = 0.0

#############################
# 运行段
#############################
while cap.isOpened() and serial_port_state:
    ##############################
    # 接收颜色图像数据
    ##############################
    ret, color_image = cap.read()

    if ret == True:
        """
            图像处理段
        """
        color_image = cv2.flip(color_image, 0)
        """
            1. 饱和度增强
        """
        hsv = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)  # 色彩空间转换, RGB->HSV

        # cv2.imshow('hsv', hsv)
        blendSRaisen = cv2.LUT(hsv, lutSRaisen)             # 饱和度增大
        # img_enhance_saturation = cv2.cvtColor(blendSRaisen, cv2.COLOR_HSV2RGB)
        # cv2.imshow('img_enhance_saturation',img_enhance_saturation)
        """
            2. 掩膜创建
        """
        # cv2.imshow('result',purple_img)
        ball_mask = cv2.inRange(blendSRaisen, lower_ball, upper_ball)
        ball_img = cv2.bitwise_and(color_image, color_image, mask=ball_mask)
        """
            3. 滤波
        """
        # purple_img = cv2.GaussianBlur(purple_img, (3, 3), 0)
        # ball_img = cv2.GaussianBlur(ball_img, (3, 3), 0)
        ball_img = cv2.medianBlur(ball_img, 5)
        """
            4. 二值化
        """
        ball_gray = cv2.cvtColor(ball_img, cv2.COLOR_BGR2GRAY)
        # purple_gray = cv2.GaussianBlur(purple_gray, (3, 3), 0)
        # ball_gray = cv2.GaussianBlur(ball_gray, (3, 3), 0)
        # cv2.imshow('result',purple_gray)
        # purple_thre = cv2.adaptiveThreshold(purple_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,5,-10)
        # ball_thre = cv2.adaptiveThreshold(ball_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,5,-10)
        res, ball_thre = cv2.threshold(
            ball_gray, 0, 255, cv2.THRESH_BINARY)
        # cv2.imshow('result', purple_thre)
        # cv2.imshow('result', ball_thre)
        ball_thre = cv2.morphologyEx(ball_thre, cv2.MORPH_OPEN, kernel)
        ball_thre = cv2.morphologyEx(ball_thre, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('result_thre', ball_thre)
        """
            5. 球的检测
        """
        # 先进行霍夫圆变换
        ball_circles = cv2.HoughCircles(ball_thre, cv2.HOUGH_GRADIENT_ALT,
                                        1.5, 20, param1=30, param2=0.50, minRadius=10, maxRadius=200)
        # 如果找到了球，则直接判定
        if ball_circles is not None:
            filter_circle = []
            ball_circles = np.uint16(np.around(ball_circles))
            for cir_ in ball_circles:
                cir = cir_[0]
                # 凹凸判断，通过创建掩膜进行逻辑与进行凹凸性判断
                x = cir[0]
                y = cir[1]
                r = cir[2]
                ball_circle_mask = np.zeros_like(ball_thre)
                cv2.circle(ball_circle_mask, (x, y), r, 255, thickness=-1)
                pixel_count = np.sum(np.logical_and(
                    ball_circle_mask, ball_thre) > 0)
                if pixel_count > 0.9 * np.pi * r * r:
                    filter_circle.append((x, y, r))
            if len(filter_circle) > 0:
                for cir_ in filter_circle:
                    ball_x = cir_[0]
                    ball_y = cir_[1]
                    ball_r = cir_[2]
                    cv2.circle(
                        color_image, (cir_[0], cir_[1]), cir_[2], (0, 255, 255), 2)
                    cv2.circle(color_image, (cir_[0], cir_[
                        1]), 2, (255, 255, 0), 2)
                
            else:
                # 提取轮廓
                contours_ball, hierarchy_ball = cv2.findContours(
                    ball_thre, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                cv2.drawContours(
                    color_image, contours_ball, -1, (0, 255, 255), 1)
                if not contours_ball:
                    pass
                else:
                    # 寻找最大面积的轮廓
                    areas_ball = []
                    for c in range(len(contours_ball)):
                        areas_ball.append(cv2.contourArea(contours_ball[c]))

                    max_id_ball = areas_ball.index(max(areas_ball))
                    # 圆拟合
                    if contours_ball[max_id_ball].size < 10:
                        pass
                    else:
                        (x_ball, y_ball), radius_ball = cv2.minEnclosingCircle(
                            contours_ball[max_id_ball])
                        center_ball = (int(x_ball), int(y_ball))
                        radius_ball = int(radius_ball)
                        ball_x = x_ball
                        ball_y = y_ball
                        ball_r = radius_ball
                        cv2.circle(color_image, center_ball,
                                   radius_ball, (0, 0, 255), 3)
                
        cv2.imshow('result', color_image)
        print(ball_x,ball_y,ball_r)
        ball_data = [ball_x,ball_y,ball_r]
        pack_data = struct.pack('<BfffB',0xFF,ball_data[0],ball_data[1],ball_data[2],0xEE)
        serial_port.write(pack_data)
        
        key = cv2.waitKey(1)
        if key == 27:
            break

        time.sleep(0.05)
    else:
        break

cv2.destroyAllWindows()
cap.release()
