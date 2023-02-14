#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import asyncio
import websockets
from std_msgs.msg import String

class Test():
	def __init__(self):
		rospy.init_node('websocket_server')
		rate = rospy.Rate(10)
		string_pub = rospy.Publisher('/websocket_test', String, queue_size = 1)
		start_server = websockets.serve(Test.accept, "0.0.0.0", 9090)
		
		asyncio.get_event_loop().run_until_complete(start_server)
		rospy.loginfo("listening")
		asyncio.get_event_loop().run_forever()

		rospy.spin()
			
	async def accept(websocket, path):
		while True:
			data = await websocket.recv()
			json_obj = json.loads(data.decode('utf-8'))

			# async 라서 global 변수는 안쓰이는 것 같음
			# 어쩔 수 없이 accept 에다가 때려박자...
			string_pub = rospy.Publisher('/websocket_test', String, queue_size = 1)
			string_msg = String()
			string_msg.data = json_obj['name']
			string_pub.publish(string_msg)
			rospy.loginfo(string_msg.data)


if __name__ == '__main__':
	try:
		test = Test()
		
	except rospy.ROSInterruptException:
		exit()
