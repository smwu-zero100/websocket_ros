#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import asyncio
import websockets
import time
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, PointCloud2, Image, PointField
from zero100_msgs.msg import Heading

class Test():
    def __init__(self):
        rospy.init_node('websocket_server')
        rate = rospy.Rate(1000)
        start_server = websockets.serve(
            Test.accept, 
            "0.0.0.0", 
            9090,
            ping_timeout=None)

        asyncio.get_event_loop().run_until_complete(start_server)
        rospy.loginfo("listening")
        asyncio.get_event_loop().run_forever()
        rospy.spin()
        
    async def accept(websocket, path):
        while True:
            data = await websocket.recv()
            Test.publishData(data)

    def publishData(raw_data):
        location_pub = rospy.Publisher('/ipad/location', NavSatFix, queue_size=1)
        heading_pub = rospy.Publisher('/ipad/heading', Heading, queue_size = 1)
        # poincloud_pub = rospy.Publisher('/ipad/pointcloud', PointCloud2, queue_size=1)
        # image_pub = rospy.Publisher('/ipad/image', Image, queue_size=1)
        location_seq = 0
        sdata = raw_data.split(',')

        if(sdata[0] == 'location'):

            try :

                location_msg = NavSatFix()
                heading_msg = Heading()
                gtime = time.time()
                location_msg.header.seq = location_seq
                location_msg.header.stamp.secs = int(gtime)
                location_msg.header.stamp.nsecs = int((gtime-int(gtime)) * 1000000)
                location_msg.header.frame_id = 'location'

                heading_msg.header.seq = location_seq
                heading_msg.header.stamp.secs = int(gtime)
                heading_msg.header.stamp.nsecs = int((gtime-int(gtime)) * 1000000)
                heading_msg.header.frame_id = 'location'

                location_msg.latitude = float(sdata[1])
                location_msg.longitude = float(sdata[2])
                location_msg.altitude = 0
                location_msg.position_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0]
                location_msg.position_covariance_type = 0
                location_pub.publish(location_msg)


                heading_msg.heading = float(sdata[3])

                heading_pub.publish(heading_msg)

            except :
                pass

    #     if(raw_data['topic'] == '/ipad/pointcloud'):
    #         try:
    #             pointcloud_msg = PointCloud2()
    #             raw_msg = raw_data['msg']
    #             pointcloud_msg.header.seq = raw_data['seq']
    #             pointcloud_msg.header.stamp.secs = raw_msg['header']['stamp']['sec']
    #             pointcloud_msg.header.stamp.nsecs = raw_msg['header']['stamp']['nanosec']
    #             pointcloud_msg.header.frame_id = raw_msg['header']['frame_id']

    #             pointcloud_msg.height = raw_msg['height']
    #             pointcloud_msg.width = raw_msg['width']
    #             pointcloud_msg.is_bigendian = raw_msg['is_bigendian']
    #             pointcloud_msg.is_dense = raw_msg['is_dense']
    #             pointcloud_msg.point_step = raw_msg['point_step']
    #             pointcloud_msg.row_step = raw_msg['row_step']
    #             pointcloud_msg.is_dense = raw_msg['is_dense']
                
    #             pointcloud_msg.fields = pointfield_msg
    #             pointcloud_msg.data = raw_msg['data']
    #             poincloud_pub.publish(pointcloud_msg)
                
    #         except:
    #             pass


if __name__ == '__main__':
	try:
		test = Test()
		
	except rospy.ROSInterruptException:
		exit()
