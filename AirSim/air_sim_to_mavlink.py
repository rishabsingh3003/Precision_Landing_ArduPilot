# Install required packages:
#
#   pip3 install pymavlink
#   pip3 install apscheduler
#   pip3 install opencv-python
#   pip3 install airsim
#   pip3 install numpy
#   sudo apt-get install python-PIL

sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os

os.environ["MAVLINK20"] = "1"

import math as m
import sys
import time
import airsim
from cv2 import cv2
import numpy as np
from PIL import Image
import threading
from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil
import argparse
import cv2.aruco as aruco


######################################################
##  Parsing user' inputs                            ##
######################################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--obstacle_distance_msg_hz', type=float,
                    help="Update frequency for OBSTACLE_DISTANCE message. If not specified, a default value will be used.")

args = parser.parse_args()

# Default configurations for connection to the FCU
if not args.connect:
    connection_string = 'localhost:14551'
else:
    connection_string = args.connect

if not args.baudrate:
    connection_baudrate = 921600
else:
    connection_baudrate = args.baudrate


# AirSim API
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# get time in correct format
start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)

# get depth from airsim backend
def get_camera_view(client):
    requests = []
    requests.append(airsim.ImageRequest(
                'bottom_center', airsim.ImageType.DepthVis, pixels_as_float=True, compress=False))

    responses = client.simGetImages(requests)
    raw = client.simGetImage('bottom_center', airsim.ImageType.Scene)

    depth = airsim.list_to_2d_float_array(
                    responses[0].image_data_float, responses[0].width, responses[0].height)
    depth = np.expand_dims(depth, axis=2)
    depth = depth.squeeze()

    img_rgb = cv2.imdecode(airsim.string_to_uint8_array(raw), cv2.IMREAD_UNCHANGED)
    return depth, responses[0].width, responses[0].height, img_rgb

# this method converts, (x,y) from depth matrix to NEU 3-D vector in body frame
def convert_depth_3D_vec(x_depth, y_depth, depth, fov):
    # https://stackoverflow.com/questions/62046666/find-3d-coordinate-with-respect-to-the-camera-using-2d-image-coordinates
    h, w = depth.shape
    center_x = w // 2
    center_y = h // 2
    focal_len = w / (2 * np.tan(fov / 2))
    x = depth[y_depth, x_depth]* 100
    y = (x_depth - center_x) * x / focal_len
    z = -1 * (y_depth - center_y) * x / focal_len
    return x,y,z

# get depth of the center of the frame, to be used as a "range finder"
def get_center_depth(depth):
    h, w = depth.shape
    center_x = w // 2
    center_y = h // 2
    x = float(depth[center_y, center_x]* 100)
    return x


# detect the arcuo marker
def detect_shape(frame):
    # operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # check if the ids list is not empty
    # if no check is added the code will crash
    if np.all(ids != None):

        # code to show ids of the marker found
        strg = ''
        for i in range(0, ids.size):
            strg += str(ids[i][0])+', '

            x = np.round(((corners[i-1][0][0][0] + corners[i-1][0][1][0] + corners[i-1][0][2][0] + corners[i-1][0][3][0]) / 4),0)
            y = np.round(((corners[i-1][0][0][1] + corners[i-1][0][1][1] + corners[i-1][0][2][1] + corners[i-1][0][3][1]) / 4),0)

            cv2.putText(frame, "ID:" + str(ids[i-1][0]), (int(x-30),int(y-65)), font, 0.8, (255,0,0),2,cv2.LINE_AA)
            cv2.putText(frame, "," + str(y), (int(x),int(y-40)), font, 0.8, (0,0,255),2,cv2.LINE_AA)
            cv2.putText(frame, str(x), (int(x-80),int(y-40)), font, 0.8, (0,0,255),2,cv2.LINE_AA)
            return [x,y]

    else:
        return None


def mavlink_loop(conn, callbacks):
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while True:
        # send a heartbeat msg
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                0,
                                0,
                                0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)

# send landing target mavlink message to SITL
def send_landing_target_message(land_pos):
	global current_time_ms, mavlink_obstacle_coordinates
	global last_obstacle_distance_sent_ms

	x = -land_pos[0]
	z = land_pos[1]
	y = land_pos[2]
	print (land_pos)
	x_offset_rad = m.atan(-z/x)
	y_offset_rad = m.atan(y/x)
	distance = np.sqrt(x * x + y * y + z * z)
	conn.mav.landing_target_send(
		current_milli_time(),                       # time target data was processed, as close to sensor capture as possible
		0,                                  # target num, not used
		mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
		x_offset_rad,                       # X-axis angular offset, in radians
		y_offset_rad,                       # Y-axis angular offset, in radians
		distance,                           # distance, in meters
		0,                                  # Target x-axis size, in radians, not used
		0,                                  # Target y-axis size, in radians, not used
		0,                                  # x	float	X Position of the landing target on MAV_FRAME, not used
		0,                                  # y	float	Y Position of the landing target on MAV_FRAME, not used
		0,                                  # z	float	Z Position of the landing target on MAV_FRAME, not used
		(1,0,0,0),      # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0), not used
		2,              # type of landing target: 2 = Fiducial marker, not used
		1,              # position_valid boolean, not used
	)

# send distance sensor to act as a downward facing rangefinder
def send_distance_sensor(depth_to_ground):
    # Average out a portion of the centermost part
    curr_dist = int(depth_to_ground * 100)
    if curr_dist > 0 and curr_dist < 4000:
        print("curr_dist= " + str(curr_dist))
        conn.mav.distance_sensor_send(
            current_milli_time(),# ms Timestamp (UNIX time or time since system boot) (ignored)
            0,   # min_distance, uint16_t, cm
            10000,   # min_distance, uint16_t, cm
            curr_dist,      # current_distance,	uint16_t, cm
            0,	            # type : 0 (ignored)
            0,              # id : 0 (ignored)
            mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,              # orientation
            0               # covariance : 0 (ignored)
        )

#pymavlink connection
conn = mavutil.mavlink_connection(
    device = str(connection_string),
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=connection_baudrate,
    force_connected=True,
)

mavlink_callbacks = {
}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

# main loop
while True:
    #depth image from airsim
    now = time.time()
    depth_mat,width,height, colour = get_camera_view(client)
    distance_to_ground = get_center_depth(depth_mat)
    send_distance_sensor(distance_to_ground)
    landing_target = detect_shape(colour)
    if (landing_target is not None):
        x_obj,y_obj,z_obj = convert_depth_3D_vec(int(landing_target[0]), int(landing_target[1]), depth_mat, m.radians(90))
        landing_coordinates = [x_obj,y_obj,z_obj]
        # add a circle on top of the target
        depth_mat = cv2.circle(depth_mat,(int(landing_target[0]), int(landing_target[1])), 5, (0, 0, 0), 5)
        send_landing_target_message(landing_coordinates)

    cv2.imshow("Down RGB", colour)
    cv2.imshow("depth", depth_mat)
    cv2.waitKey(1)