#!/usr/bin/python
import serial
import time
import struct
import rospy
from sensor_msgs.msg import Image
from aras_usb_imu.msg import aras_vio
from cv_bridge import CvBridge
import cv2
from utils import *


pub_imu = rospy.Publisher('/aras_usb_imu/vio', aras_vio,queue_size=0)
rospy.init_node('aras_usb_imu_node', anonymous=True)

param_manager = paramManager()
param_manager.load_params()

port = serial.Serial(param_manager.params['~port'], param_manager.params['~baud'], timeout=1)
imu_transfer_ts = 1000//param_manager.params['~imu_freq']
cam_transfer_ts = 1000//param_manager.params['~cam_freq']
#port.write('G1 {}\r'.format(imu_transfer_ts-1))
#port.write('G2 {}\r'.format(cam_transfer_ts))
port.write('G1 {}\r'.format(1))

vio = aras_vio()
port.flush()

while not rospy.is_shutdown():
    data=port.read_until( terminator=''.join(['abc\n']).encode('utf-8'))[:-4]
    if(len(data)==64):
        packed_data = struct.unpack('3h3h4h3i8I',data) 
        ax,ay,az,wx,wy,wz,mx,my,mz,temp, imu_ts,camera_ts,aux_encoder,\
        rc_cmd1,rc_cmd2,rc_cmd3,rc_cmd4,rc_cmd5,rc_cmd6,rc_cmd7,rc_cmd8=packed_data
        #print(packed_data)
    else:
        print("Failed to receive")
        ax,ay,az,wx,wy,wz,mx,my,mz,camera_ts,imu_ts,aux_encoder=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    vio.acc = [ax, ay, az]
    vio.gyr = [wx, wy, wz]
    vio.mag = [mx, my, mz]
    vio.ts = [camera_ts, imu_ts, 0]
    vio.header.stamp = rospy.Time.now()
    vio.aux_encoder = aux_encoder
    pub_imu.publish(vio)

port.close()
