#WIP. Ask Moses if you're confused
import IPython

import pyrealsense2 as rs

import cv2
import numpy as np
from math import tan, pi

def cameramatrix(intrinsics):
		return np.array([[intrinsics.fx,             0, intrinsics.ppx],
						 [            0, intrinsics.fy, intrinsics.ppy],
						 [            0,             0,              1]])

pipe = rs.pipeline()

cfg = rs.config()

pipe.start(cfg)

profiles = pipe.get_active_profile()



streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
           "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile(), 
					 "accel" : profiles.get_stream(rs.stream.accel),
					 "gyro"  : profiles.get_stream(rs.stream.gyro)} # No idea why these don't need .as_motion_stream_profile()

intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}

print("========= LEFT =========")
print(streams["left"].get_extrinsics_to(streams["accel"]))
print(streams["left"].get_extrinsics_to(streams["gyro"]))

print("\n\n")

print(intrinsics["left"])
print(cameramatrix(intrinsics["left"]))
print("\n\n")


print("========= RIGHT =========")

print(streams["right"].get_extrinsics_to(streams["accel"]))
print(streams["right"].get_extrinsics_to(streams["gyro"]))

print("\n\n")

print(intrinsics["right"])
print(cameramatrix(intrinsics["right"]))
print("\n\n")

