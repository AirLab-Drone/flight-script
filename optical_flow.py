import numpy as np
import cv2 as cv
import argparse
from dronekit import connect, VehicleMode
def send_optical_data_dronekit(x_flow_pixels, y_flow_pixels):
    # 光流数据
    # x_flow_pixels = 10  # x轴方向上的光流位移（像素）
    # y_flow_pixels = -5  # y轴方向上的光流位移（像素）
    quality = 200       # 光流质量

    # 创建一个光流消息
    msg = vehicle.message_factory.optical_flow_encode(
        0,                # 时间微秒（0表示当前时间）
        1,                # 传感器ID
        x_flow_pixels,    # x轴方向上的像素位移
        y_flow_pixels,    # y轴方向上的像素位移
        quality,          # 光流质量
        0,                # 地面距离（通常可以设为0或不使用）
        0,                 # X轴方向上的光流速率（通常可以设为0或不使用）
        0
    )

    # 发送光流消息
    vehicle.send_mavlink(msg)

vehicle = connect('/dev/ttyACM0',baud=9600)
cap = cv.VideoCapture(0)

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0, 255, (100, 3))

# Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

while(1):
    ret, frame = cap.read()
    if not ret:
        print('No frames grabbed!')
        break

    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    # Select good points
    if p1 is not None:
        good_new = p1[st==1]
        good_old = p0[st==1]
    if(len(good_new) == 0):
        continue
    # draw the tracks
    mean_x = 0
    mean_y = 0
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mean_x += a-c
        mean_y += b-d
        # print(f'num: {i}, delta_x: {a-c:.2f}, delta_y: {b-d:.2f}')
        # mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
        # frame = cv.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)
    mean_x /= len(good_new)
    mean_y /= len(good_new)
    print(f'x: {mean_x}, y: {mean_y}')
    send_optical_data_dronekit(int(mean_x),int(mean_y))
    # img = cv.add(frame, mask)

    # cv.imshow('frame', img)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break

    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    if(len(good_new)<=1): 
        p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
    else:
        p0 = good_new.reshape(-1, 1, 2)

