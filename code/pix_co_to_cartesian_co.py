import numpy as np
from math import*

def pix_to_cartesian_pos(x_pix,y_pix,w,h,f_x,f_y,depth_array):
	pix_x_bbox = x_pix
	pix_y_bbox = y_pix
	depth_pix = depth_array[pix_y_bbox][pix_x_bbox]
	xin = depth_pix
	# print('depth_pix,pix_x_bbox,pix_y_bbox = ',xin,pix_x_bbox,pix_x_bbox)
	yin = (pix_x_bbox - (w/2))*(depth_pix/(f_x))    
	zin = (pix_y_bbox - (h/2))*(depth_pix/(f_y))
	return(xin,yin,zin)

def pix_orientation(x_pix,y_pix,w,h,f_x,f_y,depth_array):
	apple_center = [0.4,0,0]
	x_app_surface,y_app_surface,z_app_surface = pix_to_cartesian_pos(x_pix,y_pix,w,h,f_x,f_y,depth_array)
	Xc,Yc,Zc = np.array(apple_center)-np.array([x_app_surface,y_app_surface,z_app_surface])
	Rxy = sqrt((Xc**2)+(Yc**2))
	theta = atan2(Yc/Rxy,Xc/Rxy)
	R = sqrt(Xc**2+Yc**2+Zc**2)
	phi = atan2(Zc/Rxy,Rxy/R)
	# print('x_app_surface,y_app_surface,z_app_surface = ',x_app_surface,y_app_surface,z_app_surface)
	# print('Xc,Yc,Zc = ',Xc,Yc,Zc)
	# print('Rxy = ',Rxy)
	# print('Theta = ',degrees(theta))
	# print('Phi = ',degrees(phi))
	return(theta,phi)
