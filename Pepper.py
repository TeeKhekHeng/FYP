#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Using ALDialog Methods with ROS"""

import qi
import argparse
import sys
import rospy
from naoqi import *
from std_msgs.msg import String
import time

def main(session):
	"""
	This example uses ALDialog methods with ROS.
	"""
	rospy.init_node('pepper_dialog_node', anonymous=True)
	speech_pub = rospy.Publisher('recognized_speech', String, queue_size=10)
    
	# Getting the service ALDialog
	IP = "192.168.100.198"
	ALDialog = session.service("ALDialog")
	ALDialog.clearConcepts()
	ALDialog.setLanguage("English")
	tts = ALProxy("ALTextToSpeech", IP, 9559)
	motion_service = session.service("ALMotion")   

	state = {'last_message': "", 'repeat_count': 0}#Dictionary that store the last spoken message

	tracking_enabled = True
	face_service = session.service("ALFaceDetection")
	face_service.enableTracking(tracking_enabled)
	rospy.loginfo("Tracking is: %s", face_service.isTrackingEnabled())

	def handle_output(value):
        message = value.data
    	MAX = 3
    	if message == state['last_message']:
        	state['repeat_count'] += 1
    	else:
        	state['last_message'] = message
        	state['repeat_count'] = 1
   	 
    	if state['repeat_count'] < MAX:
        	rospy.loginfo("ChatGPT: %s", message)
        	tts.say(message)
        	if "hello" in message.lower():
            	motion_service.angleInterpolation("RHand", 1, 0.5, True)
            	motion_service.angleInterpolation("RElbowRoll", -5.0, 2, True)
            	motion_service.angleInterpolation("RShoulderPitch", -1.5, 2, True)
            	motion_service.angleInterpolation("RElbowYaw", -5.0, 0.7, True)
            	motion_service.angleInterpolation("RElbowYaw", 5.0, 0.7, True)
            	motion_service.angleInterpolation("RElbowYaw", -5.0, 0.7, True)
            	motion_service.angleInterpolation("RElbowYaw", 5.0, 0.7, True)
        	elif "dance" in message.lower():
            	motion_service.angleInterpolation("RElbowRoll", -5.0, 0.5, True)
            	motion_service.angleInterpolation("LElbowRoll", -5.0, 0.5, True)
            	motion_service.angleInterpolation("HipRoll", 1, 2, True)
            	motion_service.angleInterpolation("HipRoll", -1, 2, True)
            	motion_service.angleInterpolation("HipRoll", 0, 2, True)
        	elif "good morning" in message.lower():
            	motion_service.angleInterpolation("LElbowRoll", -5.0, 0.5, True)
            	motion_service.angleInterpolation("HipPitch", -0.3, 2, True)
            	motion_service.angleInterpolation("HipPitch", 0, 2, True)
        	elif "hand" in message.lower():
            	motion_service.angleInterpolation("LElbowRoll", -5.0, 2, True)
            	time.sleep(2)
            	motion_service.angleInterpolation("LElbowRoll", 0.0, 2, True)
    	else:
        	rospy.loginfo("Message repeated too many times. ")

	rospy.Subscriber('gpt3_response', String, handle_output)

	# writing topics' qichat code as text strings (end-of-line characters are important!)
	topic_content_1 = ('topic: ~example_topic_content()\n'
                   	'language: enu\n'
                   	'concept:(food) [apple fruits chicken beef eggs]\n'
                   	'u: (I [want "would like"] {some} _~food) $onstopped=$1 \n'
                   	'u: ([Hello Hai Hey] {Pepper}) $onstopped=$1\n'
                   	'u: (how are you {today}) $onstopped=$1\n'
                   	'u: (Good morning {Pepper}) $onstopped=$1.\n'
                   	'u: (Good afternoon {Pepper}) $onstopped=$1\n'
                   	'u: (Good night {Pepper}) $onstopped=$1\n'
                   	'u: (What is your name {Pepper}) $onstopped=$1\n'
                   	'u: ([e:FrontTactilTouched e:MiddleTactilTouched e:RearTactilTouched]) You touched my head!\n')

	topic_content_2 = ('topic: ~dummy_topic()\n'
                   	'language: enu\n'
                   	'u: (How is it going) $onstopped=$1\n'
                   	'u: ({Do} {you} like {me}) $onstopped=$1\n'
                   	'u: (how old are {you}) $onstopped=$1\n'
                   	'u: (Can {you} dance) $onstopped=$1\n'
                   	'u: (You are so cute) $onstopped=$1\n'
                   	'u: (Lets hold hand {together}) $onstopped=$1\n')

	topics = ALDialog.getAllLoadedTopics()
	for topic in topics:
    	ALDialog.unloadTopic(topic)

	# Loading the topics directly as text strings
	topic_name_1 = ALDialog.loadTopicContent(topic_content_1)
	topic_name_2 = ALDialog.loadTopicContent(topic_content_2)

	# Activating the loaded topics
	ALDialog.activateTopic(topic_name_1)
	ALDialog.activateTopic(topic_name_2)

	# Define a callback to handle user speech
	def onUserSpeech(value):
    	recognized_speech = value
    	rospy.loginfo("Recognized speech: %s", recognized_speech)
    	speech_pub.publish(recognized_speech)
    
	def stop_handler():
    	rospy.loginfo("Shutting down...")
    	# stopping the dialog engine
    	ALDialog.unsubscribe('my_dialog_example')

    	# Deactivating all topics
    	ALDialog.deactivateTopic(topic_name_1)
    	ALDialog.deactivateTopic(topic_name_2)

    	# Unloading all topics
    	ALDialog.unloadTopic(topic_name_1)
    	ALDialog.unloadTopic(topic_name_2)
    	rospy.signal_shutdown("Shutdown signal received")

	memory = session.service("ALMemory")
	sub = memory.subscriber("Dialog/LastInput")
	sub.signal.connect(onUserSpeech)

	# Starting the dialog engine - we need to type an arbitrary string as the identifier
	ALDialog.subscribe('my_dialog_example')

	rospy.on_shutdown(stop_handler)
	rospy.loginfo("Press Ctrl+C to stop the program.")

	try:
    	rospy.spin()
	except rospy.ROSInterruptException:
    	pass
	finally:
    	stop_handler()
    
if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="192.168.100.198",
                    	help="Robot's IP address. If on a robot or a local Naoqi - use '127.0.0.1' (this is the default value).")
	parser.add_argument("--port", type=int, default=9559,
                    	help="port number, the default value is OK in most cases")

	args = parser.parse_args()
	session = qi.Session()
	try:
    	session.connect("tcp://{}:{}".format(args.ip, args.port))
	except RuntimeError:
    	print ("\nCan't connect to Naoqi at IP {} (port {}).\nPlease check your script's arguments."
           	" Run with -h option for help.\n".format(args.ip, args.port))
    	sys.exit(1)
	main(session)
