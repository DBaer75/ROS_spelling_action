#! /usr/bin/env python

import rospy
import time
import actionlib
from spelling_action_server.msg import robotSpellGoal ,robotSpellFeedback, robotSpellAction, robotSpellResult


def feedbackCallback(feedback):
    #callback only needs to update the user on what letter is being printed
    print("Current letter being printed is: " + str(feedback.letterBeingSpelled.data))

#create node
rospy.init_node('spelling_action_client_node')
print("spelling_action_client_node created")

#connect to action server
spelling_client = actionlib.SimpleActionClient('/spelling_as', robotSpellAction)
print("connecting to spelling action server 'spelling_as'")
spelling_client.wait_for_server()
print("connected to spelling action server 'spelling_as'")

#create a goal
spellingGoal = robotSpellGoal()
spellingGoal.stringToSpell.data = raw_input("Type a string to have bb8 draw it out: ")

#send goal to server
spelling_client.send_goal(spellingGoal, feedback_cb=feedbackCallback)


spelling_client.wait_for_result()