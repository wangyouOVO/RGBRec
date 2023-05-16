import cv2
import json
import numpy as np
import open3d
import open3d.visualization as vis

cameras = open3d.io.read_point_cloud("hello_cameras.ply")
points = open3d.io.read_point_cloud("hello_points.ply")


vis.draw([cameras,points],
             bg_color=(0.8, 0.9, 0.9, 1.0),
             show_ui=True,
             width=1920,
             height=1080)
