#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry

from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_srvs.srv import Empty

import tf

world_size = 7


def eucliDist(x1,y1,x2,y2):   
	# np.abs(pose.position.x - ball["x"]) + np.abs(pose.position.y - ball["y"])
	return np.sqrt((x2-x1)**2+(y2-y1)**2)

def angleToFrom(x1,y1,x2,y2): 
	return np.arctan2(y1-y2,x1-x2)

## Process robot odometry

def odometryCb0(msg):
	global robotPoses
	robotPoses[0] = msg.pose.pose
	checkOdom(0, msg.pose.pose)

def odometryCb1(msg):
	global robotPoses
	robotPoses[1] = msg.pose.pose
	checkOdom(1, msg.pose.pose)

def odometryCb2(msg):
	global robotPoses
	robotPoses[2] = msg.pose.pose
	checkOdom(2, msg.pose.pose)

def odometryCb3(msg):
	global robotPoses
	robotPoses[3] = msg.pose.pose
	checkOdom(3, msg.pose.pose)


def checkOdom(rnum, pose):
	
	checkBall(rnum, pose)
	
	checkWall(rnum, pose)
	
def checkWall(rnum, pose):
	
	closeToWallDist = world_size/2.0-0.2
	
	applyX = 0
	applyY = 0
	
	if pose.position.x < -closeToWallDist:
		applyX = 2
	elif pose.position.x > closeToWallDist:
		applyX = -2
		
	if pose.position.y < -closeToWallDist:
		applyY = 2
	elif pose.position.y > closeToWallDist:
		applyY = -2
		
	if applyX != 0 or applyY != 0:	
		
		rospy.wait_for_service('/gazebo/apply_body_wrench')
		apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
		
		wrench = Wrench()
		wrench.force.x = applyX
		wrench.force.y = applyY
		
		apply_body_wrench(body_name = "robo"+str(rnum)+"::base_footprint",
				 wrench = wrench,
				 duration = rospy.Duration(.05))


def checkBall(rnum, pose):
	
	global ball
	global scoreTime
	
	# pause 1 sec before allowing second check
	if rospy.get_rostime() + rospy.Duration(1) > scoreTime:
		
		dist = eucliDist(pose.position.x, pose.position.y, ball[0], ball[1])
		#print rnum, dist
		if dist < 0.25:
			scoreTime = rospy.get_rostime() 
			scoreAdd(rnum)
			spellAdd(rnum)
			moveBallRandom()

def scoreAdd(rnum):
	
	global hitCount
	hitCount[rnum] += 1	
	rospy.set_param("/robo"+str(rnum)+"/score", hitCount[rnum])
	#global scorePublishers
	#scorePublishers[rnum].publish(hitCount[rnum])
	
	print "current score", hitCount
	
	if hitCount[rnum] == 5:
		rospy.logwarn("Congratulations %d, reached required score!!", rnum)
		totalTime = (rospy.get_rostime() - gameStartTime).to_sec()
		rospy.logwarn("Total time: %d secs", totalTime)
		rospy.logwarn("\nGame Over!!\n")
		#global gameOver
		#gameOver = True
		rospy.signal_shutdown("Game is finished")
		

def spellAdd(rnum):
	global spellCount
	for i in range(4):
		if i != rnum:
			 spellCount[i] += 1
			 rospy.set_param("/robo"+str(i)+"/spell", spellCount[i])

def spellRemove(rnum):
	spellCount[rnum] -= 1
	rospy.set_param("/robo"+str(rnum)+"/spell", spellCount[rnum])


def moveBallRandom():
	
	global ball
	
	found = False
	
	while not found:
		
		randX = -(world_size/2.0-1)+np.random.random()*(world_size-2)
		randY = -(world_size/2.0-1)+np.random.random()*(world_size-2)
		
		# check vicinity to robots
		close = False
		for i in range(4):
			if eucliDist(randX, randY, robotPoses[i].position.x, robotPoses[i].position.y) < 1:
				close = True
				break
		
		if not close:
			found = True
	
	rospy.wait_for_service('/gazebo/set_model_state')
	set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
	
	pose = Pose()
	pose.position.x = randX
	pose.position.y = randY
		
	ball[0] = randX
	ball[1] = randY
	
	newstate = ModelState()
	newstate.model_name = "red_ball"
	newstate.pose = pose
	
	rospy.loginfo("Ball at %3.1f %3.1f", ball[0], ball[1])
	
	set_state(newstate)	
	
## Process velocity commands


def velCb0(msg):
	velCheck(0, msg)
def velCb1(msg):
	velCheck(1, msg)
def velCb2(msg):
	velCheck(2, msg)
def velCb3(msg):
	velCheck(3, msg)
def velCheck(num, msg):
	
	maxVel = 0.25
	maxAngVel = 0.5
	
	global waitForStart
	if waitForStart:
		msg.linear.x = 0
		msg.linear.y = 0
		msg.angular.z = 0
		velPublishers[num].publish(msg)
	
	else:
		if petrified[num]:
			if(rospy.get_time() - petrifiedTime[num] > 5):
				petrified[num] = False
			else:
				msg.linear.x = 0
				msg.linear.y = 0
				msg.angular.z = 0
				velPublishers[num].publish(msg)
		else:		
			# limit max values
			msg.linear.y = 0
			if msg.linear.x > maxVel:
				msg.linear.x = maxVel
			if msg.linear.x < -maxVel:
				msg.linear.x = -maxVel
			if msg.angular.z > maxAngVel:
				msg.angular.z = maxAngVel
			if msg.angular.z < -maxAngVel:
				msg.angular.z = -maxAngVel
			velPublishers[num].publish(msg)

def getRobotCallerNumber(req):
	ridstring = req._connection_header['callerid']
	dummystr, r1  = ridstring.split("/robo")
	r2, dummystr  = r1.split("/", 1)
	rid = int(r2)
	
	return rid

def spellHandler(rnum, spelltype):
	
	if spelltype != "k" and spelltype != "p":
		rospy.logerror("what kind of black magic is this?")
		return []
	
	global spellLast
	sinceLast = rospy.get_time() - spellLast
	if sinceLast < 1:
		pass
		#rospy.loginfo("too litle time since last spell %3.2s, caller %d", sinceLast, rnum)
	else:
		spellLast = rospy.get_time()
		#rospy.loginfo("since last: %d", sinceLast)
		
		global spellCount
		if spellCount[rnum] > 0:
			spellRemove(rnum)
			robotsInFront = getRobotsInFront(rnum)
			rospy.loginfo("%d used spell.", rnum)
			rospy.loginfo("%d robots affected", len(robotsInFront))
			for rob in robotsInFront:
				if spelltype == "k":
					kamehameha(rob)
				elif spelltype == "p":
					petrificus(rob)
			global robotPoses
			ballInFront = isInFront(ball[0], ball[1], robotPoses[rnum])
			if ballInFront[0]:
				moveBallRandom()
			
		else:
			# someone is trying to cast spell without credit!
			rospy.loginfo("fake kamehameha! cheater is %d", rnum)
			rospy.set_param("/robo"+str(rnum)+"/spell", 0)
		
	return []

def getRobotsInFront(rnum):
	
	global robotPoses
	infront = []
	for i in range(4):
		if i == rnum:
			continue
		robInFront = isInFront(robotPoses[i].position.x, robotPoses[i].position.y, robotPoses[rnum])
		if robInFront[0]:
			infront.append((i, robInFront[1], robInFront[2]))
	return infront

def isInFront(checkX, checkY, rPose):
	dist = eucliDist(checkX, checkY, rPose.position.x, rPose.position.y)
	if dist > 3:
		return (False, 0, 0)
	angle = angleToFrom(checkX, checkY, rPose.position.x, rPose.position.y)
	rQuat = [rPose.orientation.x, rPose.orientation.y, rPose.orientation.z, rPose.orientation.w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(rQuat)
	angle_diff = angle-yaw
	while angle_diff > np.pi: angle_diff -= 2*np.pi
	while angle_diff < -np.pi: angle_diff += 2*np.pi
	angle_diff = np.abs(angle_diff)
	if angle_diff > np.pi/6:
		return (False, 0, 0)
	return (True, dist, angle)

def kamehamehaHandler(req):
	rospy.logerr("kamehamehaHandler")
	return spellHandler(getRobotCallerNumber(req), "k")
	
def kamehameha(rob):
	
	# petrify it, but for a shorter time
	petrified[rob[0]] = True
	petrifiedTime[rob[0]] = rospy.get_time() + 3
	
	rospy.wait_for_service('/gazebo/apply_body_wrench')
	apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
	
	wrench = Wrench()
	wrench.force.x = (4-rob[1])/1.0*np.cos(rob[2])
	wrench.force.y = (4-rob[1])/1.0*np.sin(rob[2])
	
	apply_body_wrench(body_name = "robo"+str(rob[0])+"::base_footprint",
			 wrench = wrench,
			 duration = rospy.Duration(.5))
	
def petrificusHandler(req):
	return spellHandler(getRobotCallerNumber(req), "p")

def petrificus(rob):
	petrified[rob[0]] = True
	petrifiedTime[rob[0]] = rospy.get_time()


def startHandler(req):
	global waitForStart
	waitForStart = False
	return []
	
robotPoses=[]
ball=[]

hitCount=[]
scorePublishers=[]


velPublishers=[]

spellCount=[]

petrified=[]
petrifiedTime=[]

waitForStart = True
#gameOver = False

if __name__ == "__main__":
	rospy.init_node('odometryPrinter') #make node 
	
	for i in range(4):
		robotPoses.append(Pose())
		hitCount.append(0)
		spellCount.append(0)
		rospy.set_param("/robo"+str(i)+"/spell", 0)
		rospy.set_param("/robo"+str(i)+"/score", 0)
		petrified.append(False)
		petrifiedTime.append(rospy.get_time())
	
	spellLast = rospy.get_time()


	velPublishers.append( rospy.Publisher('/r0_vel', Twist, queue_size=2, latch=True) )
	velPublishers.append( rospy.Publisher('/r1_vel', Twist, queue_size=2, latch=True) )
	velPublishers.append( rospy.Publisher('/r2_vel', Twist, queue_size=2, latch=True) )
	velPublishers.append( rospy.Publisher('/r3_vel', Twist, queue_size=2, latch=True) )
	
	rospy.Subscriber('/robo0/cmd_vel',Twist,velCb0)
	rospy.Subscriber('/robo1/cmd_vel',Twist,velCb1)
	rospy.Subscriber('/robo2/cmd_vel',Twist,velCb2)
	rospy.Subscriber('/robo3/cmd_vel',Twist,velCb3)

	# wait for service call
	rospy.Service('/start_game', Empty, startHandler)
	
	waitrate = rospy.Rate(1)
	while waitForStart and not rospy.is_shutdown():
		waitrate.sleep()
	
	ball.append(0)
	ball.append(0)
	moveBallRandom()
	scoreTime = rospy.get_rostime()
	gameStartTime = rospy.get_rostime()
	
	rospy.Subscriber('/robo0/odom',Odometry,odometryCb0)
	rospy.Subscriber('/robo1/odom',Odometry,odometryCb1)
	rospy.Subscriber('/robo2/odom',Odometry,odometryCb2)
	rospy.Subscriber('/robo3/odom',Odometry,odometryCb3)
	
	rospy.Service('/kamehameha', Empty, kamehamehaHandler)
	rospy.Service('/petrificus', Empty, petrificusHandler)
	
	#while not gameOver and not rospy.is_shutdown():
	#	waitrate.sleep()
	rospy.spin()
