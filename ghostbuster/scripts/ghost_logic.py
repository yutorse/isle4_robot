#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty


## Process robot odometry
def odometryCb(msg):
	global robotPose
	robotPose = msg.pose.pose
	

def eucliDist(x1,y1,x2,y2):   
	# np.abs(pose.position.x - ball["x"]) + np.abs(pose.position.y - ball["y"])
	return np.sqrt((x2-x1)**2+(y2-y1)**2)
	
def busterHandler(req):
	
	global ghosts
	global robotPose
	
	busted = []

	# pause 1 sec before allowing second check
	for ghostid, ghostpos in ghosts.items():
		
		dist = eucliDist(robotPose.position.x, robotPose.position.y, ghostpos['x'], ghostpos['y'])
		
		print ("Dist to ghost ", ghostid, " : ", dist)
		
		if dist < 0.5:
			rospy.logwarn("Ghost %s is busted!", ghostid)
			busted.append(ghostid)

	if len(busted) > 0:	
		for gid in busted:
			del ghosts[gid]
		
		if len(ghosts) < 1:
			rospy.logwarn("All ghosts busted - DONE!")
			rospy.signal_shutdown("Game is finished")		
		else:
			publishGhosts()
	
	return []

def addGhost():
	global ghostcounter, ghosts, freesize, freemaprealx, freemaprealy, freemapindex
	ghostcounter += 1
	ghostind = np.random.randint(freesize)
	ghosts[ghostcounter] = {'x':freemaprealx[ghostind], 'y':freemaprealy[ghostind], 'val': griddata[freemapindex[ghostind]]}
	

def publishGhosts():
	global ghosts
	p = PolygonStamped()
	p.header.frame_id="map"
	p.header.stamp = rospy.Time.now()
	for key, val in ghosts.items():
		p.polygon.points.append(Point32(x=val['x'],y=val['y'],z=0))
	
	global ghostPublisher
	ghostPublisher.publish(p)


ghostcounter = 0
ghosts = {}


if __name__ == "__main__":
	# make node 
	rospy.init_node('odometryPrinter') 
	
	# read map
	rospy.wait_for_service("static_map")
	mapsrv = rospy.ServiceProxy("static_map", GetMap)
	resp = mapsrv()
	
	# get coordinates of free space in map
	griddata = np.asarray(resp.map.data)
	freemapindex = np.argwhere((griddata >= 0) & (griddata < 50))
	
	freesize = freemapindex.shape[0]
		
	freemapy = freemapindex // resp.map.info.width
	freemapx = freemapindex - freemapy * resp.map.info.width
	#freemaprealx = resp.map.info.origin.position.x + freemapx * resp.map.info.resolution
	#freemaprealy = resp.map.info.origin.position.y + freemapy * resp.map.info.resolution
	
	# find and remove cells too close to wall
	# this implementation is slow but usable
	occupmapindex = np.argwhere(griddata > 50)
	occupmapy = occupmapindex // resp.map.info.width
	occupmapx = occupmapindex - occupmapy * resp.map.info.width
	
	kidelete = []
	for ki in range(freesize):
		if np.min(np.abs(occupmapx - freemapx[ki]) + np.abs(occupmapy - freemapy[ki])) < 8:
			kidelete.append(ki)
			
	freemapindex = np.delete(freemapindex, kidelete)
	freesize = freemapindex.shape[0]
	
	freemapy = freemapindex // resp.map.info.width
	freemapx = freemapindex - freemapy * resp.map.info.width
	freemaprealx = resp.map.info.origin.position.x + freemapx * resp.map.info.resolution
	freemaprealy = resp.map.info.origin.position.y + freemapy * resp.map.info.resolution
		
	
	# first 13 ghosts 
	for i in range(13):
		addGhost()

	ghostPublisher = rospy.Publisher('/ghost', PolygonStamped, queue_size=2, latch=True)
	
	publishGhosts()

	rospy.Subscriber('/odom',Odometry,odometryCb)
	
	rospy.Service('/buster', Empty, busterHandler)
	
	while not rospy.is_shutdown():
		# add ghost
		addGhost()
		publishGhosts()
		
		rospy.sleep(40 + np.random.randn()*10)
		
	
	
