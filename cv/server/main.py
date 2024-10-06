#!/usr/bin/python3

# -*- coding:utf-8 -*-

import time
import cv2 as cv
import numpy as np
from opencv_gst_rtsp_server import OpenCVFrameRTSPServer

capture = cv.VideoCapture(0)
grabbed, frame1 = capture.read()

fps = int(capture.get(cv.CAP_PROP_FPS))
fps = fps if  60 > fps > 0 else 30
duration = 1.0/fps 
height, width, channel = frame1.shape

server = OpenCVFrameRTSPServer(
    width=width,
    height=height, 
    channel=channel, 
    fps=fps, 
    use_h265=True, 
    port=8001
)
server.start_background()

prvs = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[..., 1] = 255

while True:
    start_time = time.time()
    grabbed, frame2 = capture.read()

    next_ = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
    flow = cv.calcOpticalFlowFarneback(prvs, next_, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    mag, ang = cv.cartToPolar(flow[..., 0], flow[..., 1])
    ret, thresh = cv.threshold(mag, 3, 255, cv.THRESH_BINARY)
    thresh_sum = thresh.sum(axis=0)
    thresh_sum_nz = np.where(thresh_sum != 0)[0]
    width_center = int(width / 2)
    if len(thresh_sum_nz) != 0:
        thresh_sum_nz_min = thresh_sum_nz.min()
        thresh_sum_nz_max = thresh_sum_nz.max()
        thresh_sum_nz_centr = int(thresh_sum_nz_min + (thresh_sum_nz_max - thresh_sum_nz_min) / 2)
        div = thresh_sum_nz_centr - width_center
    else:
        div = 0
        thresh_sum_nz_min = 0
        thresh_sum_nz_max = width
        thresh_sum_nz_centr = width_center
    frame2 = cv.line(frame2, (thresh_sum_nz_min,0), (thresh_sum_nz_min,height), (250,0,0), 2)
    frame2 = cv.line(frame2, (thresh_sum_nz_max,0), (thresh_sum_nz_max,height), (250,0,0), 2)
    frame2 = cv.line(frame2, (width_center,0), (width_center,height), (0,250,0), 2)
    frame2 = cv.line(frame2, (thresh_sum_nz_centr,0), (thresh_sum_nz_centr,height), (0,0,250), 2)
    print(f'pix: {div}, percent: {round((div/width_center)*100, 2)}%')

    if grabbed:
        server.set_frame(frame=frame2)
    else:
       capture.set(cv.CAP_PROP_POS_FRAMES, 0)
       continue
    end_time = time.time()
    elapsed_time = end_time - start_time
    sleep_duration = duration - elapsed_time
    if sleep_duration > 0:
        time.sleep(sleep_duration)
    prvs = next_

