import pybullet as p
import sys
from camera import ur5_camera
import time

print(sys.version)
pi=3.14159265359
# start simulation
ABSE = lambda a,b: abs(a-b)

def check(Te,Qe):
	result = True
	for x in Te:
		if x > 0.04:
			result = False
	for y in Qe:
		if y > 0.12:#0.12
			result = False
	# print('Result = ',result)
	return result
	


def pos(x1,y1,z1,roll0,yaw0,pitch0,robotID,eefID):
	flag = True
	# xin = p.addUserDebugParameter("x", -2, 2, x)
	# yin = p.addUserDebugParameter("y", -2, 2, y)
	# zin = p.addUserDebugParameter("z", -2, 2, z)
	# roll1 = p.addUserDebugParameter("roll", -2*pi, 2*pi, roll)
	# yaw1 = p.addUserDebugParameter("yaw", -2*pi, 2*pi, yaw)
	# pitch1 = p.addUserDebugParameter("pitch", -2*pi, 2*pi, pitch)
	# capture1 = p.addUserDebugParameter("capture",-3,3,capture)
	while(flag):
		# print('looping',flag)
		# capture = p.readUserDebugParameter(capture1)
		# ur5_camera(robotID)
		# x = p.readUserDebugParameter(xin)
		# y = p.readUserDebugParameter(yin)
		# z = p.readUserDebugParameter(zin)
		# roll = p.readUserDebugParameter(roll1)
		# yaw = p.readUserDebugParameter(yaw1)
		# pitch = p.readUserDebugParameter(pitch1)
		x = x1
		y = y1
		z = z1
		roll = roll0
		yaw = yaw0
		pitch = pitch0
		robotEndOrn = p.getQuaternionFromEuler([roll,yaw,pitch])
		jointPose = p.calculateInverseKinematics(robotID, eefID, [x,y,z],robotEndOrn)
		# print("---------------jointPose-------------------------")
		# print(jointPose)
		joint_index=[1,2,3,4,5,6]
		pose_group=[jointPose[0],jointPose[1],jointPose[2],jointPose[3],jointPose[4],jointPose[5]]
		p.setJointMotorControlArray(robotID, joint_index, p.POSITION_CONTROL,targetPositions=pose_group)
		rXYZ = p.getLinkState(robotID, eefID)[-2]    # real XYZ
		rWxWyWz = p.getLinkState(robotID, eefID)[-1] 
		# print(x,y,z,rXYZ,rWxWyWz)
		Te = list(map(ABSE,[x,y,z],rXYZ)) #translation error
		Qe = list(map(ABSE,robotEndOrn,rWxWyWz)) #rotation error
		# print("x_err= {:.3f}, y_err= {:.3f}, z_err= {:.3f}".format(*Te))
		# print("W1_err= {:.3f}, W2_err= {:.3f}, W3_err= {:.3f}, W4_err= {:.3f}".format(*Qe))
		# for x in Te:
		# 	print(x)
		if(check(Te,Qe)):
			# print('Te,Qe = ',Te,Qe)
			flag = False
		time.sleep(0.05)
		p.stepSimulation()
	# p.disconnect()

