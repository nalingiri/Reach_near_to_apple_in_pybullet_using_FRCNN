# Reach_near_to_apple_in_pybullet_using_FRCNN

1) One hard coding path(Line no : 39) is available in camera.py file. Make it according to your system. More importantly we have to save the capture image in "detection/images/". 
3) Two folders are missing "output_frcnn" and "elements_of_trees". Place those folder inside the code folder. Below is the link for those files. 
 https://drive.google.com/drive/folders/1zy6RUBx9idpDAeZXJyDESyJdWj8_mOSR?usp=sharing
  One frcnn trained file is their. This is for demonstration. You can place your own trained network.
2) COMMAND LINE:-
python position.py  --data_path /path/detection --output_file /path/code/prediction_frcnn/out.txt --weight_file /path/code/output_frcnn/model_49.pth --device cpu --frcnn
