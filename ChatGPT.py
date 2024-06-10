#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import openai
import threading

# Initialize the OpenAI API
openai.api_key = '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'

def handle_speech_recognized(data):
	messages = data.data
	if not messages.strip():
        rospy.loginfo("Received empty message. Skipping process...")
    	return
    
	rospy.loginfo("From user: %s", messages)
	received = [
    	{"role": "system", "content": "You are a physical Pepper robot, a helpful robot assistant with two hands and a head and capable to dance and hold hand"},
    	{"role": "user", "content": messages}
	]
	response = openai.chat.completions.create(
    	model="gpt-3.5-turbo", messages=received
	)
	result = response.choices[0].message.content
	rospy.loginfo("From ChatGPT: %s", result)
	response_pub.publish(result)

def speech_recognized_callback(data):
	threading.Thread(target=handle_speech_recognized, args=(data,)).start()

if __name__ == '__main__':
	# Initialize ROS node
	rospy.init_node('gpt3_node', anonymous=True)
    
	# Publisher for GPT-3 responses
	response_pub = rospy.Publisher('gpt3_response', String, queue_size=10)
    
	# Subscriber to listen for recognized speech
	rospy.Subscriber('recognized_speech', String, handle_speech_recognized)
    
	rospy.spin()
