import pybullet as p
import numpy as np
# import pybullet_data
import time
# import random
from PIL import Image
from math import*

width=480
height=480
near=0.01
far=5
fov=40
aspect=width/height
pi=3.14159265359

def ur5_camera(robotID):
	pos=p.getLinkState(robotID,linkIndex=7,computeForwardKinematics=1)[-2]
	ort=p.getLinkState(robotID,linkIndex=7,computeForwardKinematics=1)[-1]
	rot_mat=np.array(p.getMatrixFromQuaternion(ort))
	rot_mat=np.reshape(rot_mat,[3,3])
	dir=rot_mat.dot([1,0,0])
	up_vector=rot_mat.dot([0,0,1])
	s = 0.01
	view_matrix=p.computeViewMatrix(pos,pos+s*dir,up_vector)
	p.addUserDebugText(text=".",textPosition=pos+s*dir,textColorRGB=[1,0,0],textSize=10)
	projection_matrix = p.computeProjectionMatrixFOV(fov,aspect,near,far)
	f_len = projection_matrix[0]
	_,_,rgbImg,depthImg_buffer,segImg=p.getCameraImage(width,height,view_matrix,projection_matrix,renderer=p.ER_BULLET_HARDWARE_OPENGL)
	depthImg_buf = np.reshape(depthImg_buffer, [width, height])
	depthImg = far * near / (far - (far - near) * depthImg_buf)
	f_x = width/(2*tan(fov*pi/360))
	f_y = height/(2*tan(fov*pi/360))
	for i in range(10000):
		if(not(i%10000)):
			rgbImg_array = np.array(rgbImg)
			im = Image.fromarray(rgbImg_array)
			im = im.convert("RGB")
			im.save("/home/nalin/Desktop/prem/fruit_plucking2/apple_detection1/detection/images/{}.jpg".format(i))
			# = = Store data for segmentation = =
	  #       segImg_array = np.array(segImg)+1
      #    	  length = len(np.unique(segImg_array))
	  #       unique_values = np.unique(segImg_array)
	  #       print('unique = ', unique_values)
	  #       print('len = ',length)
	  #       print('shape = ', segImg_array.shape)
	  #       print('trunk pix index',unique_values[1])
	  #       segImg_array[segImg_array == unique_values[1]] = unique_values[0]   # color of trunk is repleced by background    
	  #       img = Image.fromarray((segImg_array)*255/length)
	  #       img = img.convert("RGB")
	  #       img.save("Address/{}.png".format(time.time()))
	  #       print('Image saved')
		
		#time.sleep(0.75)
	p.removeAllUserDebugItems()
	return (depthImg,f_x,f_y)
