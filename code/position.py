import inverse_kinematics_position as ikp
import pybullet as p
import pybullet_data
import time
import numpy as np
from predict_rcnn import main_pixel_co
from camera import ur5_camera
from pix_co_to_cartesian_co import pix_to_cartesian_pos

serverMode = p.DIRECT # GUI/DIRECT
sisbotUrdfPath = "./urdf/sisbot.urdf"
# palm_tree_Path = "./palm_tree/palm_tree.urdf"
apple_Path = "./elements_of_trees/apple/apple.urdf"
trunk_Path = "./elements_of_trees/trunk/trunk.urdf"
leaf_path = "./elements_of_trees/leaf/leaf.urdf"
sphere_Path = "./elements_of_trees/sphere/sphere.urdf"


# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
p.setGravity(0,0,-10) # NOTE
planeID = p.loadURDF("plane.urdf")

pi=3.14159265359
# # setup sisbot
#robotStartPos = [0,0,0]
#robotStartOrn = p.getQuaternionFromEuler([0,0,0])
# trunkStartPos1 = [0.9,0.1,-0.2] # For loading cube or tree on that position and orientation
# trunkStartOrn1 = p.getQuaternionFromEuler([0,0,0])
#==== Sphere orientation ==================================
#trunkStartPos1 = [0.9,0.1,-0.2] # For loading cube or tree on that position and orientation
#trunkStartOrn1 = p.getQuaternionFromEuler([0,0,0])
# appleStartPos1 = [0.06,0,0]
#sphereStartPos1 = [0.5,0.15,0.7]
#sphereStartOrn1 = p.getQuaternionFromEuler([pi/2,-0.5,0])
# appleStartPos2 = [0.5-0.016125,0.18+0.05,0.66+0.0325]
# appleStartOrn2 = p.getQuaternionFromEuler([pi/2,-0.5,0])
#leafStartPos1 = [0.38+0.02,0.18-0.06,0.7]
#leafStartOrn1 = p.getQuaternionFromEuler([0.1,-pi/2+0.3,pi/2])
#leafStartPos2 = [0.38+0.02,0.17-0.06,0.66]
#leafStartOrn2 = p.getQuaternionFromEuler([0.1,-pi/2+0.3,pi/2])
# appleStartPos2 = [0.4,-0.08,0.8]
# appleStartOrn2 = p.getQuaternionFromEuler([pi/2,-0.9,0])
#=================Complete Tree==============================
robotStartPos = [-0.4,0,0.10]
robotStartOrn = p.getQuaternionFromEuler([0,0,-0.3])
trunkStartPos1 = [0.8,0,-0.2] # For loading cube or tree on that position and orientation
trunkStartOrn1 = p.getQuaternionFromEuler([0,0,0])
appleStartPos1 = [0.45,0,0.76]
appleStartOrn1 = p.getQuaternionFromEuler([pi/2,-0.5,0])
appleStartPos2 = [0.42,0.1,0.66]
appleStartOrn2 = p.getQuaternionFromEuler([pi/2,0.1,-1])
appleStartPos3 = [0.4,-0.48,0.76]
appleStartOrn3 = p.getQuaternionFromEuler([pi/2-0.5,0.5,0])
appleStartPos4 = [0.52,-0.4,0.69]
appleStartOrn4 = p.getQuaternionFromEuler([pi/2-0.5,0.1,-1])
appleStartPos5 = [0.47,-0.48,0.76]
appleStartOrn5 = p.getQuaternionFromEuler([pi/2-0.5,0.5,0])
appleStartPos6 = [0.4,0.48,0.93]
appleStartOrn6 = p.getQuaternionFromEuler([pi/2-0.5,0.5,0])
appleStartPos7 = [0.5,0.4,1.2]
appleStartOrn7 = p.getQuaternionFromEuler([pi/2-0.5,0.1,-1])
appleStartPos8 = [0.45,0.54,0.95]
appleStartOrn8 = p.getQuaternionFromEuler([pi/2,-0.5,0])
appleStartPos9 = [0.7,0,0.5]
appleStartOrn9 = p.getQuaternionFromEuler([pi/2-0.5,0.1,-1])
# #=================================================
leafStartPos1 = [0.38,0,0.76]
leafStartOrn1 = p.getQuaternionFromEuler([0.1,-pi/2+0.3,pi/2])
leafStartPos2 = [0.5,0.04,0.8]
leafStartOrn2 = p.getQuaternionFromEuler([0,-pi/2,-pi/2+0.2])
leafStartPos3 = [0.38,-0.045,0.8]
leafStartOrn3 = p.getQuaternionFromEuler([0.1,-pi/2+0.3,pi/2])
leafStartPos4 = [0.39,-0.05,0.8]
leafStartOrn4 = p.getQuaternionFromEuler([0,-pi/2,-pi/2+0.2])
leafStartPos5 = [0.38,-0.045,0.9]
leafStartOrn5 = p.getQuaternionFromEuler([0,+pi/2+0.3,pi/2])  # up direction
leafStartPos6 = [0.39,-0.05,0.9]
leafStartOrn6 = p.getQuaternionFromEuler([0,+pi/2,-pi/2+0.2]) # up direction
leafStartPos7 = [0.44,0.2,0.76]
leafStartOrn7 = p.getQuaternionFromEuler([0.5,-pi/2+0.3,pi/2])
leafStartPos8 = [0.49,0.18,0.7]
leafStartOrn8 = p.getQuaternionFromEuler([0,-pi/2,-pi/2+0.2])
leafStartPos9 = [0.45,0.2,0.76]
leafStartOrn9 = p.getQuaternionFromEuler([0.1,-pi/2+0.3,pi/2])
leafStartPos10 = [0.48,0.1,0.65]
leafStartOrn10 = p.getQuaternionFromEuler([0,-pi/2,-pi/2+0.2])
leafStartPos11 = [0.48,0.1,0.7]
leafStartOrn11 = p.getQuaternionFromEuler([0,+pi/2,-pi/2+0.2]) # up direction
leafStartPos12 = [0.75,0,0.5]
leafStartOrn12 = p.getQuaternionFromEuler([0.1,-pi/2+0.3,pi/2])
leafStartPos13 = [0.70,0,0.5]
leafStartOrn13 = p.getQuaternionFromEuler([0,-pi/2,-pi/2+0.2])
# leafStartPos12 = [0.58,0.46,1.15]
# leafStartOrn12 = p.getQuaternionFromEuler([0.1,-pi/2+0.7,-pi/2])
# leafStartPos13 = [0.57,0.48,1.15]
# leafStartOrn13 = p.getQuaternionFromEuler([0,-pi/2+0.3,-pi/2])
leafStartPos14 = [0.47,0.4,1.25]
leafStartOrn14 = p.getQuaternionFromEuler([0.1,-pi/3,-pi/2])
leafStartPos15 = [0.48,0.41,1.25]
leafStartOrn15 = p.getQuaternionFromEuler([pi/2,-pi/4,-pi/2])
leafStartPos16 = [0.45,0.52,1.05]
leafStartOrn16 = p.getQuaternionFromEuler([0.1,pi/2,-pi/2])
leafStartPos17 = [0.45,0.52,1.05]
leafStartOrn17 = p.getQuaternionFromEuler([pi/2,pi/2,-pi/2])
leafStartPos18 = [0.47,0.52,0.95]
leafStartOrn18 = p.getQuaternionFromEuler([0.1,-pi/5,-pi/2])
leafStartPos19 = [0.40,0.51,0.95]
leafStartOrn19 = p.getQuaternionFromEuler([0,-pi/4,-pi/2])
leafStartPos20 = [0.47,-0.48,0.76]
leafStartOrn20 = p.getQuaternionFromEuler([0.1,-pi/2,-pi/2])
leafStartPos21 = [0.47,-0.48,0.76]
leafStartOrn21 = p.getQuaternionFromEuler([pi/2,-pi/2,-pi/2])
leafStartPos22 = [0.49,-0.42,0.8]
leafStartOrn22 = p.getQuaternionFromEuler([0.1,pi/2,-pi/2])
leafStartPos23 = [0.49,-0.42,0.8]
leafStartOrn23 = p.getQuaternionFromEuler([pi/2,pi/2,-pi/2])
leafStartPos24 = [0.49,-0.35,0.75]
leafStartOrn24 = p.getQuaternionFromEuler([0,0,-pi/2])
leafStartPos25 = [0.47,-0.37,0.75]
leafStartOrn25 = p.getQuaternionFromEuler([-pi/2,0,-pi/2])
leafStartPos26 = [0.7,0,0.5]
leafStartOrn26 = p.getQuaternionFromEuler([0,0,pi/2])
leafStartPos27 = [0.73,0.04,0.5]
leafStartOrn27 = p.getQuaternionFromEuler([0,pi/2,pi/2])
leafStartPos28 = [0.74,0,0.55]
leafStartOrn28 = p.getQuaternionFromEuler([0,-pi/2,0.5])
leafStartPos29 = [0.73,0,0.6]
leafStartOrn29 = p.getQuaternionFromEuler([pi/6,pi/2,pi/4])
leafStartPos30 = [0.73,0,0.61]
leafStartOrn30 = p.getQuaternionFromEuler([pi/2-0.5,pi/2,0])
# print("----------------------------------------")
# print("Loading robot from {}".format(sisbotUrdfPath))
# #==================================================
textureId_trunk = p.loadTexture('./elements_of_trees/trunk/apple_tree_bark.jpg')
trunkID = p.loadURDF(trunk_Path, trunkStartPos1, trunkStartOrn1, useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)# or p.URDF_USE_SELF_COLLISION)
p.changeVisualShape(trunkID, -1, textureUniqueId=textureId_trunk)
# #==================================================
textureId = p.loadTexture('./elements_of_trees/apple/texture.jpg')
apple1 = p.loadURDF(apple_Path, appleStartPos1, appleStartOrn1,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple1, -1, textureUniqueId=textureId)
apple2 = p.loadURDF(apple_Path, appleStartPos2, appleStartOrn2,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple2, -1, textureUniqueId=textureId)
apple3 = p.loadURDF(apple_Path, appleStartPos3, appleStartOrn3,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple3, -1, textureUniqueId=textureId)
apple4 = p.loadURDF(apple_Path, appleStartPos4, appleStartOrn4,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple4, -1, textureUniqueId=textureId)
apple5 = p.loadURDF(apple_Path, appleStartPos5, appleStartOrn5,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple5, -1, textureUniqueId=textureId)
apple6 = p.loadURDF(apple_Path, appleStartPos6, appleStartOrn6,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple6, -1, textureUniqueId=textureId)
apple7 = p.loadURDF(apple_Path, appleStartPos7, appleStartOrn7,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple7, -1, textureUniqueId=textureId)
apple8 = p.loadURDF(apple_Path, appleStartPos8, appleStartOrn8,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple8, -1, textureUniqueId=textureId)
apple9 = p.loadURDF(apple_Path, appleStartPos9, appleStartOrn9,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(apple9, -1, textureUniqueId=textureId)
# #==================================================
textureId_leaf = p.loadTexture('./elements_of_trees/leaf/leaf1.jpg')
leafID1 = p.loadURDF(leaf_path, leafStartPos1, leafStartOrn1, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID1, -1, textureUniqueId=textureId_leaf)
leafID2 = p.loadURDF(leaf_path, leafStartPos2, leafStartOrn2, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID2, -1, textureUniqueId=textureId_leaf)
leafID3 = p.loadURDF(leaf_path, leafStartPos3, leafStartOrn3, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID3, -1, textureUniqueId=textureId_leaf)
leafID4 = p.loadURDF(leaf_path, leafStartPos4, leafStartOrn4, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID4, -1, textureUniqueId=textureId_leaf)
leafID5 = p.loadURDF(leaf_path, leafStartPos5, leafStartOrn5, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID5, -1, textureUniqueId=textureId_leaf)
leafID6 = p.loadURDF(leaf_path, leafStartPos6, leafStartOrn6, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID6, -1, textureUniqueId=textureId_leaf)
leafID7 = p.loadURDF(leaf_path, leafStartPos7, leafStartOrn7, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID7, -1, textureUniqueId=textureId_leaf)
leafID8 = p.loadURDF(leaf_path, leafStartPos8, leafStartOrn8, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID8, -1, textureUniqueId=textureId_leaf)
leafID9 = p.loadURDF(leaf_path, leafStartPos9, leafStartOrn9, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID9, -1, textureUniqueId=textureId_leaf)
leafID10 = p.loadURDF(leaf_path, leafStartPos10, leafStartOrn10, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID10, -1, textureUniqueId=textureId_leaf)
leafID11 = p.loadURDF(leaf_path, leafStartPos11, leafStartOrn11, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID11, -1, textureUniqueId=textureId_leaf)
leafID12 = p.loadURDF(leaf_path, leafStartPos12, leafStartOrn12, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID12, -1, textureUniqueId=textureId_leaf)
leafID13 = p.loadURDF(leaf_path, leafStartPos13, leafStartOrn13, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID13, -1, textureUniqueId=textureId_leaf)
leafID14 = p.loadURDF(leaf_path, leafStartPos14, leafStartOrn14, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID14, -1, textureUniqueId=textureId_leaf)
leafID15 = p.loadURDF(leaf_path, leafStartPos15, leafStartOrn15, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID15, -1, textureUniqueId=textureId_leaf)
leafID16 = p.loadURDF(leaf_path, leafStartPos16, leafStartOrn16, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID16, -1, textureUniqueId=textureId_leaf)
leafID17 = p.loadURDF(leaf_path, leafStartPos17, leafStartOrn17, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID17, -1, textureUniqueId=textureId_leaf)
leafID18 = p.loadURDF(leaf_path, leafStartPos18, leafStartOrn18, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID18, -1, textureUniqueId=textureId_leaf)
leafID19 = p.loadURDF(leaf_path, leafStartPos19, leafStartOrn19, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID19, -1, textureUniqueId=textureId_leaf)
leafID20 = p.loadURDF(leaf_path, leafStartPos20, leafStartOrn20, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID20, -1, textureUniqueId=textureId_leaf)
leafID21 = p.loadURDF(leaf_path, leafStartPos21, leafStartOrn21, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID21, -1, textureUniqueId=textureId_leaf)
leafID22 = p.loadURDF(leaf_path, leafStartPos22, leafStartOrn22, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID22, -1, textureUniqueId=textureId_leaf)
leafID23 = p.loadURDF(leaf_path, leafStartPos23, leafStartOrn23, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID23, -1, textureUniqueId=textureId_leaf)
leafID24 = p.loadURDF(leaf_path, leafStartPos24, leafStartOrn24, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID24, -1, textureUniqueId=textureId_leaf)
leafID25 = p.loadURDF(leaf_path, leafStartPos25, leafStartOrn25, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID25, -1, textureUniqueId=textureId_leaf)
leafID26 = p.loadURDF(leaf_path, leafStartPos26, leafStartOrn26, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID26, -1, textureUniqueId=textureId_leaf)
leafID27 = p.loadURDF(leaf_path, leafStartPos27, leafStartOrn27, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID27, -1, textureUniqueId=textureId_leaf)
leafID28 = p.loadURDF(leaf_path, leafStartPos28, leafStartOrn28, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID28, -1, textureUniqueId=textureId_leaf)
leafID29 = p.loadURDF(leaf_path, leafStartPos29, leafStartOrn29, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID29, -1, textureUniqueId=textureId_leaf)
leafID30 = p.loadURDF(leaf_path, leafStartPos30, leafStartOrn30, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
p.changeVisualShape(leafID30, -1, textureUniqueId=textureId_leaf)
# #===================================================
robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
#=====================Till Here ====================
# robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn, flags=p.URDF_USE_INERTIA_FROM_FILE)
# trunkID = p.loadURDF(trunk_Path, trunkStartPos1, trunkStartOrn1, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE or p.URDF_USE_SELF_COLLISION)
# apple1 = p.loadURDF(apple_Path, appleStartPos1, appleStartOrn1,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
#leafID1 = p.loadURDF(leaf_path, leafStartPos1, leafStartOrn1, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
#leafID2 = p.loadURDF(leaf_path, leafStartPos2, leafStartOrn2, useFixedBase=True,flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
#p.changeVisualShape(leafID1, -1, textureUniqueId=textureId_leaf)
#p.changeVisualShape(leafID2, -1, textureUniqueId=textureId_leaf)
# apple2 = p.loadURDF(apple_Path, appleStartPos2, appleStartOrn2,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
# cube = p.loadURDF("cube_small.urdf", robotStartPos1, robotStartOrn1)
#sphereID = p.loadURDF(sphere_Path, sphereStartPos1, sphereStartOrn1,useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
# physicsClient.addUserData(planeID, "MyKey1", "MyValue1")
eefID=7
x_init=0.1
y_init=0.15 #0.15
z_init=0.7
roll_init=0.2
yaw_init=-0.1
pitch_init=0 #0
isnum=p.isNumpyEnabled()
print('===Numpy Enable=====')
print('Is numpy enabled = ',isnum)
ikp.pos(x_init,y_init,z_init,roll_init,yaw_init,pitch_init,robotID,eefID)

	
flag = True
xin = p.addUserDebugParameter("x", -2, 2, 0)
yin = p.addUserDebugParameter("y", -2, 2, 0)
zin = p.addUserDebugParameter("z", -2, 2, 0)
roll1 = p.addUserDebugParameter("roll", -pi, pi, 0)
yaw1 = p.addUserDebugParameter("yaw", -pi, pi, 0)
pitch1 = p.addUserDebugParameter("pitch", -pi, pi, 0)
xc = 0
yc = 0
zc = 0
rollc = 0
yawc = 0
pitchc = 0

if __name__ == "__main__":
	import argparse
	parser = argparse.ArgumentParser(description='PyTorch Detection')
	parser.add_argument('--data_path', required=True, help='path to the data to predict on')
	parser.add_argument('--output_file', required=True, help='path where to write the prediction outputs')
	parser.add_argument('--weight_file', required=True, help='path to the weight file')
	parser.add_argument('--device', default='cpu', help='device to use. Either cpu or cuda')
	model = parser.add_mutually_exclusive_group(required=True)
	model.add_argument('--frcnn', action='store_true', help='use a Faster-RCNN model')
	model.add_argument('--mrcnn', action='store_true', help='use a Mask-RCNN model')
	args = parser.parse_args()


while(flag):
	print(xc,yc,zc,rollc,yawc,pitchc)
	depth,f_x,f_y = ur5_camera(robotID)
	depth_array = np.array(depth)
	np.savetxt('depth_array.csv', depth_array, delimiter=',', fmt='%f')
	h,w,bbox_centers_pix,roll_yaw_pitch = main_pixel_co(args,f_x,f_y,depth_array)
	print('h_Img,w_Img,centers = ',h,w,bbox_centers_pix)
	print('========================')
	for i in range(len(bbox_centers_pix)):
		xin,yin,zin = pix_to_cartesian_pos(int(bbox_centers_pix[i][0]),int(bbox_centers_pix[i][1]),w,h,f_x,f_y,depth_array)
		#roll1,yaw1,pitch1 = roll_yaw_pitch[i][0],roll_yaw_pitch[i][1],roll_yaw_pitch[i][2],
		print('xin_bbox_center ,yin_bbox_center ,zin_bbox_center ,f_x ,f_y = ',xin,yin,zin,f_x,f_y)
		print('========================')
		c = input('please enter a number in loop ')
		x = xin - xc
		y = -(yin - yc)
		z = -(zin - zc)
		roll = roll1 - rollc
		yaw =-(yaw1 - yawc)
		pitch = -(pitch1 - pitchc)
		# x = p.readUserDebugParameter(xin) - xc
		# y = p.readUserDebugParameter(yin) - yc
		# z = p.readUserDebugParameter(zin) - zc
		print("roll1,yaw1,pitch1 = ",roll1,yaw1,pitch1)
		#roll = p.readUserDebugParameter(roll1) - rollc
		#yaw = p.readUserDebugParameter(yaw1) - yawc
		#pitch = p.readUserDebugParameter(pitch1) - pitchc
		cameraTargetPos = [x,y,z]
		cameraTargetOrn = p.getQuaternionFromEuler([roll,yaw,pitch])
		camera_pos_W=p.getLinkState(robotID,linkIndex=7,computeForwardKinematics=1)[-2]
		camera_ort_W=p.getLinkState(robotID,linkIndex=7,computeForwardKinematics=1)[-1]
		# print(camera_pos_W,camera_ort_W)
		new_camera_pos_W,new_camera_ort_W = p.multiplyTransforms(camera_pos_W,camera_ort_W,cameraTargetPos,cameraTargetOrn)
		# print(new_camera_pos_W,new_camera_ort_W) // Go near to the fruit
		ikp.pos(new_camera_pos_W[0],new_camera_pos_W[1],new_camera_pos_W[2],new_camera_ort_W[0],new_camera_ort_W[1],new_camera_ort_W[2],robotID,eefID)
		c = input('please enter a number in loop')
		ikp.pos(x_init,y_init,z_init,roll_init,yaw_init,pitch_init,robotID,eefID)  # Go back to origin

	c = input('please enter a number')
	xc = p.readUserDebugParameter(xin)
	yc = p.readUserDebugParameter(yin)
	zc = p.readUserDebugParameter(zin)
	rollc = p.readUserDebugParameter(roll1)
	yawc = p.readUserDebugParameter(yaw1)
	pitchc = p.readUserDebugParameter(pitch1)

# p.disconnect()
