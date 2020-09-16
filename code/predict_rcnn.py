import os
import torch
import torch.utils.data
import torchvision
import numpy as np
import cv2
from math import*

from pix_co_to_cartesian_co import pix_orientation
# from data.apple_dataset import AppleDataset
from data.apple_dataset_pre import AppleDataset
# from data.apple_dataset_data1 import AppleDataset_data1
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

import utility.utils as utils
import utility.transforms as T


######################################################
# Predict with either a Faster-RCNN or Mask-RCNN predictor
######################################################
def get_transform(train):
	transforms = []
	transforms.append(T.ToTensor())
	if train:
		transforms.append(T.RandomHorizontalFlip(0.5))
	return T.Compose(transforms)


def get_maskrcnn_model_instance(num_classes):
	# load an instance segmentation model pre-trained pre-trained on COCO
	model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=False)
	# get number of input features for the classifier
	in_features = model.roi_heads.box_predictor.cls_score.in_features
	# replace the pre-trained head with a new one
	model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

	# now get the number of input features for the mask classifier
	in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
	hidden_layer = 256
	# and replace the mask predictor with a new one
	model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask, hidden_layer, num_classes)
	return model


def get_frcnn_model_instance(num_classes):
	# load an instance segmentation model pre-trained pre-trained on COCO
	model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=False)

	# get number of input features for the classifier
	in_features = model.roi_heads.box_predictor.cls_score.in_features
	# replace the pre-trained head with a new one
	model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
	return model

def drawBoundingBox(stacked,centers,roll_yaw_pitch,args,f_x,f_y,depth_array):#,imgcv,result):
#	imgcv= cv2.imread('/home/nalin/Desktop/DataSet/detection/original_data_sets/test/images/{}'.format(stacked[0][0]))
	imgcv= cv2.imread(os.path.join(args.data_path, 'images/{}'.format(stacked[0][0])))
	h,w,_ = imgcv.shape
	for box in stacked:
		print('========')
		print(box)
		print('========')
		x1 = int(box[-5])
		y1 = int(box[-4])
		x2 = int(box[-3])
		y2 = int(box[-2])
		# x.astype(int)
		conf = box[-1]
		print('x1 = ',x1)
		print('y1 = ',y1)
		print('x2 = ',x2)
		print('y2 = ',y2)
		# print(conf)
		# label = box['label']
		if float(conf) > 0.75:
			cv2.rectangle(imgcv,(x1,y1),(x2,y2),(0,255,0),1)
			centers.append([0.5*(x1+x2),0.5*(y1+y2)])
			print('=============================================')
			# img_to_array = np.asarray(imgcv)
			# print('img_to_array shape = ',img_to_array.shape)
			# print('img_to_array shape = ',img_to_array[:,:,0].shape)
			# np.savetxt('npB.csv', imgcv[:,:,0], delimiter=',')
			np.savetxt('npG.csv', imgcv[:,:,1], delimiter=',')
			# np.savetxt('npR.csv', imgcv[:,:,2], delimiter=',')
			print('Image shape = ',imgcv.shape)
			# print('=================================================')
			# for i in range [y1:y2]:
			# 	for j in range [x1:x2]:
			# 		print('pixel values of each [i][j] = ',imgcv[i,j,1])
			theta_ij_total = 0
			phi_ij_total = 0
			length_data = 0
			i = y1
			j = x1
			while(i >= y1 and i < y2):
				while(j >= x1 and j < x2):
					if( imgcv[i,j,1]<100 ):
						# print('i = ',i)
						# print('j = ',j)
						# print('pixel values of each [i][j] = ',imgcv[i,j,1])
						theta_ij,phi_ij = pix_orientation(j,i,h,w,f_x,f_y,depth_array)
						theta_ij_total = theta_ij_total+theta_ij
						phi_ij_total = phi_ij_total+phi_ij
						length_data = length_data+1
					j = j+1
				i = i+1
				j = x1

			theta = theta_ij_total/length_data
			phi = phi_ij_total/length_data
			# theta,phi = pix_orientation(int(0.5*(y1+y2)),int(0.5*(x1+x2)),h,w,f_x,f_y,depth_array)
			r11 = cos(theta)*cos(phi)
			r21 = sin(theta)*cos(phi)
			r31 = -sin(phi)
			r32 = 0
			r33 = cos(phi)
			yaw = atan2(r21,r11)
			pitch = atan2(-r31,sqrt(r32**2+r33**2))
			roll = atan2(r32,r33)
			print('roll,yaw,pitch = ',degrees(roll),degrees(yaw),degrees(pitch))
			roll_yaw_pitch.append([roll,yaw,pitch])
			cv2.putText(imgcv,conf,(x1,y1),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0),1)
			cv2.imwrite(os.path.join(args.data_path, 'images_with_bbox/{}'.format(stacked[0][0])), imgcv)
	box_image = cv2.imread(os.path.join(args.data_path, 'images_with_bbox/{}'.format(stacked[0][0])))
	#print('Path = ',)
	return (h,w,centers,roll_yaw_pitch) 

def main_pixel_co(args,f_x,f_y,depth_array): #_pixel_co
	num_classes = 2
	device = args.device

	# Load the model from
	print("Loading model")
	# Create the correct model type
	if args.mrcnn:
		model = get_maskrcnn_model_instance(num_classes)
	else:
		model = get_frcnn_model_instance(num_classes)

	# Load model parameters and keep on CPU
	checkpoint = torch.load(args.weight_file, map_location=device)
	# print(checkpoint)
	# for key, value in checkpoint.items() :
	# 	print(key)
	model.load_state_dict(checkpoint, strict=False)
	model.eval()

	print("Creating data loaders")
	dataset_test = AppleDataset(args.data_path, get_transform(train=False))
	# dataset_test = AppleDataset_data1(args.data_path, get_transform(train=False))
	data_loader_test = torch.utils.data.DataLoader(dataset_test, batch_size=1,
												   shuffle=False, num_workers=1,
												   collate_fn=utils.collate_fn)

	# Create output directory
	base_path = os.path.dirname(args.output_file)
	if not os.path.exists(base_path):
		os.makedirs(base_path)

	# Predict on bboxes on each image
	f = open(args.output_file, 'a')
	for image, targets in data_loader_test:    # for different images one after another
		image = list(img.to(device) for img in image)
		outputs = model(image)
		print('Output = \n',outputs)
		centers = []
		roll_yaw_pitch = []
		for ii, output in enumerate(outputs):  # bbox of one instance(apple) in one image
			# print('ii,output = ', ii, output)
			img_id = targets[ii]['image_id']
			img_name = data_loader_test.dataset.get_img_name(img_id)
			print("Predicting on image: {}".format(img_name))
			boxes = output['boxes'].detach().numpy()
			scores = output['scores'].detach().numpy()

			im_names = np.repeat(img_name, len(boxes), axis=0)
			stacked = np.hstack((im_names.reshape(len(scores), 1), boxes.astype(int), scores.reshape(len(scores), 1)))

			# File to write predictions to
			print('stacked = ',stacked)
			h,w,centers,roll_yaw_pitch = drawBoundingBox(stacked,centers,roll_yaw_pitch,args,f_x,f_y,depth_array)
			print('centers = ',centers)
			# print('h,w,centers = ',h,w,centers)
			np.savetxt(f, stacked, fmt='%s', delimiter=',', newline='\n')         
	return(h,w,centers,roll_yaw_pitch)
