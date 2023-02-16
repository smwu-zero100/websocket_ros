#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import asyncio
import websockets
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, PointCloud2, Image

class Test():
    def __init__(self):
        rospy.init_node('websocket_server')
        rate = rospy.Rate(10)
        # string_pub = rospy.Publisher('/websocket_test', String, queue_size = 1)
        start_server = websockets.serve(Test.accept, "0.0.0.0", 9090)

        asyncio.get_event_loop().run_until_complete(start_server)
        rospy.loginfo("listening")
        asyncio.get_event_loop().run_forever()
        rospy.spin()
        
    async def accept(websocket, path):
        while True:
            data = await websocket.recv()
            json_obj = json.loads(data.decode('utf-8'))
            Test.publishData(json_obj)

    def publishData(raw_data):
        #string_pub = rospy.Publisher('/websocket_test', String, queue_size = 1)
        location_pub = rospy.Publisher('/ipad/location', NavSatFix, queue_size=1)
        # poincloud_pub = rospy.Publisher('/ipad/poincloud', PointCloud2, queue_size=1)
        # image_pub = rospy.Publisher('/ipad/image', Image, queue_size=1)

        location_seq = 0

        if(raw_data['topic'] == '/ipad/location'):
            try :
                location_msg = NavSatFix()
                raw_msg = raw_data['msg']
                location_msg.header.seq = location_seq
                location_msg.header.stamp.secs = raw_msg['header']['stamp']['sec']
                location_msg.header.stamp.nsecs = raw_msg['header']['stamp']['nanosec']
                location_msg.header.frame_id = raw_msg['header']['frame_id']

                location_msg.latitude = raw_msg['latitude']
                location_msg.longitude = raw_msg['longitude']
                location_msg.altitude = 0
                location_msg.position_covariance = raw_msg['position_covariance']
                location_msg.position_covariance_type = 0
                location_pub.publish(location_msg)
                print(location_msg)
                location_seq += 1
                # print(message)

            except :
                pass


if __name__ == '__main__':
	try:
		test = Test()
		
	except rospy.ROSInterruptException:
		exit()
