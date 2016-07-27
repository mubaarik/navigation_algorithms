#!/usr/bin/python
import numpy as np 
import rospy 
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
class localPlanner:
	'''simple ros node for avoiding abstacles using laserscan data'''
	def __init__(self):
		#Ros subscribers and publishers
		self.subs=rospy.Subscriber("racecar/laser/scan", LaserScan, self.laserCallback)
		self.pubs = rospy.Publisher("/racecar/ackermann_cmd_mux/input/default", AckermannDriveStamped,queue_size = 3)

		self.midlle_cell =(np.pi/9, -np.pi/9)
		
		self.thresh_hold=1.5
		#print self.left_cells
	def isFree(self,cell,data):
		cell=cell
		cell0=cell[0]
		cell1=cell[1]
		full_range=data.angle_max-data.angle_min
		
		cell_strt=int((data.angle_max+cell1)/(full_range)*len(data.ranges))
		cell_end=int((data.angle_max+cell0)/(full_range)*len(data.ranges))
		ranges=sorted(data.ranges[cell_strt:cell_end])
		if np.mean(ranges[0:len(ranges)/6])<self.thresh_hold:
			return False
		return True
	def laserCallback(self,msg):
		left_base=-np.pi/36
		wind=np.pi/12
		right_base=np.pi/36
		incr=np.pi/180
		msgs = AckermannDriveStamped()
		msgs.drive.speed=.84
		if self.isFree(self.midlle_cell,msg):
			#msgs.drive.speed=0
			msgs.drive.steering_angle=0
			self.pubs.publish(msgs)
		else:
			turn="left"
			index=0
			opening=False
			while not opening and right_base<=np.pi/4:
				left_cell=(left_base,left_base-wind)
				right_cell = (right_base,right_base+wind)
				if self.isFree(left_cell,msg):
					
					opening=True
					
				elif self.isFree(right_cell,msg):
					
					opening=True
					turn="right"
				
				left_base-=incr
				right_base+=incr
			if opening:
				if turn == "right":
					msgs.drive.steering_angle=right_base+wind/2 #np.mean(self.right_cells[index])
					#msgs.drive.steering_angle=self.right_cells[index][0]
					self.pubs.publish(msgs)
				else:
					msgs.drive.steering_angle = left_base-wind/2 #np.mean(self.left_cells[index])
					#msgs.drive.steering_angle = self.left_cells[index][1]
					self.pubs.publish(msgs)
			else:
				msgs.drive.speed=0
				msgs.drive.steering_angle=0
				self.pubs.publish(msgs)

				print "No path found"


if __name__ == "__main__":
	rospy.init_node("localPlanner")
	node = localPlanner()
	rospy.spin()
