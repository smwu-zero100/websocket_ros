#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import asyncio
import websockets
from std_msgs.msg import String

class Test():
	def __init__(self):
		# self.data = ''
		rospy.init_node('websocket_server')
		rate = rospy.Rate(10)
		#string_pub = rospy.Publisher('/websocket_test', String, queue_size = 1)
		start_server = websockets.serve(Test.accept, "0.0.0.0", 0000)
		
		asyncio.get_event_loop().run_until_complete(start_server)
		rospy.loginfo("listening")
		
		#string_msg = String()
		#jsonObject = json.loads(self.data.encode('utf-8').decode('utf-8')))
		#string_msg.data = jsonObject.get(name)
		
		#string_pub.publish(string_msg)
		asyncio.get_event_loop().run_forever()
		rospy.spin()
			
	async def accept(websocket, path):
		while True:
			data = await websocket.recv()
			print(data)
			#await websocket.send("echo" + data)


if __name__ == '__main__':
	try:
		test = Test()
		
	except rospy.ROSInterruptException:
		exit()
