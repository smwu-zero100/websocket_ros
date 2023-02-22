#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import asyncio
import websockets
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, PointCloud2, Image, PointField
from zero100_msgs.msg import Heading

class Test():
    def __init__(self):
        rospy.init_node('websocket_server')
        rate = rospy.Rate(10)
        # string_pub = rospy.Publisher('/websocket_test', String, queue_size = 1)
        start_server = websockets.serve(Test.accept, "0.0.0.0", 9090, ping_timeout=None)

        asyncio.get_event_loop().run_until_complete(start_server)
        rospy.loginfo("listening")
        asyncio.get_event_loop().run_forever()
        rospy.spin()
        
    async def accept(websocket, path):
        while True:
            data = await websocket.recv()
            json_obj = json.loads(data.decode('utf-8'))
            Test.publishData(json_obj)
            rospy.loginfo("publish!")

    def publishData(raw_data):
        #string_pub = rospy.Publisher('/websocket_test', String, queue_size = 1)
        location_pub = rospy.Publisher('/ipad/location', NavSatFix, queue_size=1)
        heading_pub = rospy.Publisher('/ipad/heading', Heading, queue_size = 1)
        poincloud_pub = rospy.Publisher('/ipad/pointcloud', PointCloud2, queue_size=1)
        # image_pub = rospy.Publisher('/ipad/image', Image, queue_size=1)
        
        pointfield_xmsg = PointField()
        pointfield_ymsg = PointField()
        pointfield_zmsg = PointField()

        pointfield_xmsg.name = 'x'
        pointfield_xmsg.offset = 0
        pointfield_xmsg.datatype = 7
        pointfield_xmsg.count = 1
        pointfield_ymsg.name = 'y'
        pointfield_ymsg.offset = 4
        pointfield_ymsg.datatype = 7
        pointfield_ymsg.count = 1
        pointfield_zmsg.name = 'z'
        pointfield_zmsg.offset = 8
        pointfield_zmsg.datatype = 7
        pointfield_zmsg.count = 1

        pointfield_msg = [pointfield_xmsg, pointfield_ymsg, pointfield_zmsg]

        location_seq = 0

        if(raw_data['topic'] == '/ipad/location'):
            try :
                location_msg = NavSatFix()
                heading_msg = Heading()
                raw_msg = raw_data['msg']
                location_msg.header.seq = raw_data['seq']
                location_msg.header.stamp.secs = raw_msg['header']['stamp']['sec']
                location_msg.header.stamp.nsecs = raw_msg['header']['stamp']['nanosec']
                location_msg.header.frame_id = raw_msg['header']['frame_id']

                heading_msg.header.seq = raw_data['seq']
                heading_msg.header.stamp.secs = raw_msg['header']['stamp']['sec']
                heading_msg.header.stamp.nsecs = raw_msg['header']['stamp']['nanosec']
                heading_msg.header.frame_id = raw_msg['header']['frame_id']

                location_msg.latitude = raw_msg['latitude']
                location_msg.longitude = raw_msg['longitude']
                location_msg.altitude = 0
                location_msg.position_covariance = raw_msg['position_covariance']
                location_msg.position_covariance_type = 0
                location_pub.publish(location_msg)

                heading_msg.heading = raw_msg['altitude']
                print(heading_msg)
                heading_pub.publish(heading_msg)

            except :
                pass

        if(raw_data['topic'] == '/ipad/pointcloud'):
            try:
                pointcloud_msg = PointCloud2()
                raw_msg = raw_data['msg']
                pointcloud_msg.header.seq = raw_data['seq']
                pointcloud_msg.header.stamp.secs = raw_msg['header']['stamp']['sec']
                pointcloud_msg.header.stamp.nsecs = raw_msg['header']['stamp']['nanosec']
                pointcloud_msg.header.frame_id = raw_msg['header']['frame_id']

                pointcloud_msg.height = raw_msg['height']
                pointcloud_msg.width = raw_msg['width']
                pointcloud_msg.is_bigendian = raw_msg['is_bigendian']
                pointcloud_msg.is_dense = raw_msg['is_dense']
                pointcloud_msg.point_step = raw_msg['point_step']
                pointcloud_msg.row_step = raw_msg['row_step']
                pointcloud_msg.is_dense = raw_msg['is_dense']
                
                pointcloud_msg.fields = pointfield_msg
                pointcloud_msg.data = raw_msg['data']
                poincloud_pub.publish(pointcloud_msg)
                
            except:
                pass


if __name__ == '__main__':
	try:
		test = Test()
		
	except rospy.ROSInterruptException:
		exit()
