#! /usr/bin/env python

import rospy
import time as t
import numpy as np
import actionlib

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from spelling_action_server.msg import robotSpellFeedback, robotSpellAction, robotSpellResult

#debugging print switches on 
debugOnSub = 0
debugOnTurn = 0

#throttle of how fast the program writes 0:1
speed = 1.0

#controls how big each line is drawn. Could be made as big as needed. 3 is a reasonable value
fontSize = 3

# on letters with a anglular line. ie A, With a verticle line being 90 degrees. 
#   needs to be defined here so dependent line lengths such as the cross bar on "A"
#   can be calculated   
angleSize = 75

#variable to publish to /cmd_vel 
tMsg = Twist()

#variables to hold the position and orientatioin of the robot read from /odom
pos_x = 0.0
pos_y = 0.0
ang_z = 0.0
odom_cur = [pos_x, pos_y, ang_z]

#create publisher
cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def subCallback(data): 
    #called whenever /odom is updated
    #   gets data from /odom
    #   converts into x, y, and angular_z(rad/sec)
    #   stores results in global variables
    global odom_cur
    odom_cur[0] = data.pose.pose.position.x
    odom_cur[1] = data.pose.pose.position.y
    quat_x = data.pose.pose.orientation.x
    quat_y = data.pose.pose.orientation.y
    quat_z = data.pose.pose.orientation.z
    quat_w = data.pose.pose.orientation.w

    #convert quarterion orientation to radians. This math is heavily based on example C code given on the wiki article for quaterions.
    #   As this was the only point that I needed to work in quaterions I do not need an entire library 
    anglerad = np.arctan2(2 * (quat_w * quat_z + quat_x * quat_y),1 - 2 * (quat_y * quat_y + quat_z * quat_z))
    odom_cur[2] = anglerad #follows right hand rule. ie x+ is 0, y+ is pi/2

    if (debugOnSub):
        print("pos_x = " + str(odom_cur[0]))
        print("pos_y = " + str(odom_cur[1]))
        print("ang_z = " + str(odom_cur[2]))
        
def stopRobot():
    #Stops robot by publishing empty twist msg to /cmd_vel
    global tMsg
    tMsg = Twist()
    cmdPub.publish(tMsg)
    return

def turnRobot(degreeRequest):
    #turns robot to a absolute degree orientation
    #degrees from 180 to -180 with 
    #   x+ = 0
    #   y+ = 90
    global tMsg
    global odom_cur
    global speed
    radR = degreeRequest*(np.pi/180) #convert to radians
    radC = odom_cur[2] #get current orientation
    if (debugOnTurn):
        print("radR = " + str(radR) + " radC = " + str(radC))

    while (abs(radR-radC)>(np.pi/90)): #accurate to 2 degrees. higher accuracy requires lower speed to prevent overshoot. 
        radC = odom_cur[2] 
        if ((radR-radC)>(np.pi/90)):
            tMsg.angular.z = speed/2
        elif ((radR-radC)<(np.pi/90)):
            tMsg.angular.z = -speed/2
        #print("radR = " + str(radR) + " radC = " + str(radC))
        cmdPub.publish(tMsg)
        radC = odom_cur[2]
    stopRobot()
    return

def driveRobot(distanceR):
    #drives robot directly forward a specific distance
    global tMsg
    global odom_cur
    global speed

    #get current position
    pos_x_start = odom_cur[0]
    pos_y_start = odom_cur[1]

    dist_travelled = 0
    while(dist_travelled<=abs(distanceR)):
        #calculate the distance traveled so far
        x_diff = odom_cur[0] - pos_x_start
        y_diff = odom_cur[1] - pos_y_start
        dist_travelled = np.sqrt((x_diff**2)+(y_diff**2))

        tMsg.linear.x = speed
        cmdPub.publish(tMsg)
    stopRobot()
    return

def moveUpRight(distance):
    global angleSize   
    turnRobot(angleSize)
    driveRobot(distance)

def moveDownRight(distance):
    global angleSize
    turnRobot(-angleSize)
    driveRobot(distance)

def moveDownLeft(distance):
    global angleSize
    turnRobot(-180+angleSize)
    driveRobot(distance)

def moveLeft(distance):
    turnRobot(179)
    driveRobot(distance)

def moveRight(distance):
    turnRobot(0)
    driveRobot(distance)

def moveUp(distance):
    turnRobot(90)
    driveRobot(distance)

def moveDown(distance):
    turnRobot(-90)
    driveRobot(distance)

def setForNextLetter(letteri):
    #drive robot to the start position of the next letter
    #   each letter has a area defined by fontSize x fontSize
    #   this function places the robot at the lower left corner of the letter to be printed next
    global odom_cur
    global fontSize

    #define to point to be travelled to
    pos_x_i = letteri*fontSize
    pos_y_i = 0

    #get current postion
    cur_x = odom_cur[0]
    cur_y = odom_cur[1]

    #calculate angle and postion to point
    x_offset = pos_x_i-cur_x
    y_offset = pos_y_i-cur_y
    #print("x_offset is: " + str(x_offset) + " y_offset is: " + str(y_offset))
    distToPoint = np.sqrt((x_offset**2)+(y_offset**2))
    angleToPoint = np.arctan2(y_offset,x_offset) #rad/s
    #print("distToPoint: " + str(distToPoint) + " angleToPoint: " + str(angleToPoint))

    #navigate to point
    turnRobot(angleToPoint*180/(np.pi))
    driveRobot(distToPoint)
    return



class RobotSpellClass(object):
    #create variables to handle feedback and result
    _feedback = robotSpellFeedback()
    _result = robotSpellResult()
    
    def __init__(self):
        #create server node
        self._as = actionlib.SimpleActionServer("spelling_as", robotSpellAction, self.goal_callback, False)
        print("Creating spelling action server 'spelling_as'")
        self._as.start()
    def letterPrint(self, letter):
        #Does the actual printing of the letter
        #   Current letters implemented are [A,B,C,D,E]
        #   All letters will be printed in caps and "BLOCKLETTER" font
        if ((letter=='a') or (letter=='A')):
            moveUpRight(fontSize)
            moveDownRight(fontSize/2)
            cross_bar_size = 2*(fontSize/2)*np.cos(angleSize*np.pi/180) #calculate cross bar lenght
            #print("cross bar size is "+ str(cross_bar_size))
            moveLeft(cross_bar_size)
            moveRight(cross_bar_size)
            moveDownRight(fontSize/2)
        if ((letter=='b') or (letter=='B')):
            moveUp(fontSize)
            moveRight(fontSize*0.75)
            moveDown(fontSize/2)
            moveLeft(fontSize*0.75)
            moveRight(fontSize*0.75)
            moveDown(fontSize/2)
            moveLeft(fontSize*0.75)
            moveRight(fontSize*0.75)
        if ((letter=='c') or (letter=='C')):
            moveUp(fontSize)
            moveRight(fontSize*0.75)
            moveLeft(fontSize*0.75)
            moveDown(fontSize)
            moveRight(fontSize*0.75)
        if ((letter=='d') or (letter=='D')):
            moveUp(fontSize)
            moveRight(fontSize*0.7)

            #makes a small angle cut off topright and bottomright of letter
            x = (fontSize*0.01)/np.tan((-np.pi/2)+(angleSize*np.pi/180)) 
            z = np.sqrt((x**2) + ((fontSize*0.01)**2))
            #print("x: "+ str(x) + "z: " + str(z))
            moveDownRight(z)
            moveDown(fontSize-4*x)
            moveDownLeft(z)

            moveLeft(fontSize*0.6)
            moveRight(fontSize*0.6)
        if ((letter=='e') or (letter=='E')):
            moveUp(fontSize)
            moveRight(fontSize*0.75)
            moveLeft(fontSize*0.75)
            moveDown(fontSize/2)
            moveRight(fontSize*0.75)
            moveLeft(fontSize*0.75)
            moveDown(fontSize/2)
            moveRight(fontSize*0.75)
        return 1

    def goal_callback(self, goal):
        #called whenever the action is called by a client
        #   spells out the string and updates client with feedback

        #print("string to spell is" + str(goal.stringToSpell.data) + "\n")
        #print("len of string to spell is " + str(int(len(str(goal.stringToSpell.data)))) + "\n")

        #return to origin
        setForNextLetter(0)

        #iterate through all letters sent
        for i in range(len(str(goal.stringToSpell.data))):
            #print(i)
            
            #check if preempted
            if self._as.is_preempt_requested():
                print("Preempted. Cancelling drawing")
                self._as.set_preempted()
                self._result = False
                break
            #print("Letter being printed is " + str(goal.stringToSpell.data)[i] + "\n")

            #return feedback to client
            self._feedback.letterBeingSpelled.data = str(goal.stringToSpell.data)[i] 
            self._as.publish_feedback(self._feedback)

            #print the letter
            successOrFail = self.letterPrint(str(goal.stringToSpell.data)[i])

            #navigate to the lower left corner of the next letter. Letter drawing is all done by driving in specific angle and distance which introduces errors.
            #   This call makes sure that the errors in one letter do not add to the next letter.  
            setForNextLetter(i+1)

        #return result of drawing to client   
        self._result.successFail.data = successOrFail
        self._as.set_succeeded(self._result)
        return 
    

if __name__ == '__main__':
    #main function to initiate node and subscriber
    rospy.init_node('spelling_action_server_node')
    poseSub = rospy.Subscriber('/odom', Odometry, subCallback)
    RobotSpellClass()
    while not rospy.is_shutdown():
        continue