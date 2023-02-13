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
		start_server = websockets.serve(Test.accept, "0.0.0.0", 9090)
		
		asyncio.get_event_loop().run_until_complete(start_server)
		rospy.loginfo("listening")

		asyncio.get_event_loop().run_forever()
		rospy.spin()
			
	async def accept(websocket, path):
		while True:
			data = await websocket.recv()
			print(data)


if __name__ == '__main__':
	try:
		test = Test()
		
	except rospy.ROSInterruptException:
		exit()
