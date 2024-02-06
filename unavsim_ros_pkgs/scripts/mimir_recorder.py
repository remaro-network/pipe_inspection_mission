#!/usr/bin/env python3

'''
INTERFACE for simulation setup
data collection with airsim and unreal for:
simple_flight uav
zed mini stereo camera
    rgb left, rgb right, depth


COORDINATE SYSTEM:
All AirSim API uses NED coordinate system, i.e., +X is North, +Y is East and +Z is Down. 
All units are in SI system. P
lease note that this is different from coordinate system used internally by Unreal Engine. 
In Unreal Engine, +Z is up instead of down and length unit is in centimeters instead of meters. 
AirSim APIs takes care of the appropriate conversions. 
The starting point of the vehicle is always coordinates (0, 0, 0) in NED system. 
Thus when converting from Unreal coordinates to NED, we first subtract the starting offset and then scale by 100 for cm to m conversion. 
The vehicle is spawned in Unreal environment where the Player Start component is placed. 
There is a setting called OriginGeopoint in settings.json which assigns geographic longitude, longitude and altitude to the Player Start component.
'''

from fileinput import filename
from tkinter import Y
from venv import create
import airsim
import numpy as np
import csv
import os
import time
import datetime
from math import *
import time
import json
import cv2
from scipy.spatial.transform import Rotation as R
import imageio

from helpers import exist_and_not_empty, MyEncoder,NoIndent, write_pfm16, write_exr
from ament_index_python.packages import get_package_share_directory
'''

'''

class MIMIRrecorder:

    def __init__(self, client, create_dataset=False):

        ''' 
        LOAD INIT VALUES FROM A CONFIG FILE 
        '''
        # Find the package's share directory
        package_dir = get_package_share_directory('unavsim_ros_pkgs')
        # Construct the paths to the config files
        recorder_config_path = os.path.join(package_dir, 'config', 'recorder_config.json')
        settings_path = os.path.join(package_dir, 'config', 'settings.json')
        # CONNECTION TO AIRSIM 
        self.client = client


        self.create_dataset = create_dataset
        self.config   = self.loadJSON(recorder_config_path)
        self.settings = self.loadJSON(settings_path)

        # Create path
        vehicle_list = list(self.settings["Vehicles"].keys())
        cameras_list = list(self.settings["Vehicles"][vehicle_list[0]]["Cameras"].keys())
        self.createPaths(self.config["dataset_base_path"], dataset=self.config["dataset_base_name"], sequence=self.config["map_name"],
                        vehicles= vehicle_list, cameras=cameras_list,
                        track=self.config["track_name"], create_dataset=create_dataset)
        

        if not self.config["imurate"]%self.config["framerate"] == 0:
            print('ERROR: imurate must be multiple of framerate')
            exit()
        else:
            self.rate_divider = int(self.config["imurate"]/self.config["framerate"])
        
        # Set segmentation labels
        label_dict = self.setSegmentation(self.config["SegmentationIDs"])

        # Save configuration into dataset
        self.saveJSONtoDataset(inputpath=os.getcwd(), filename='config.json', outputpath= self.DATASET_TRACK)
        self.saveJSONtoDataset(inputpath=os.getcwd(), filename='settings.json', outputpath= self.DATASET_TRACK)
        self.saveJSONtoDataset(inputpath=None, data=label_dict,filename='segmentation.json', outputpath= self.DATASET_TRACK)

        self.saveIntrinsics(vehicle_list,cameras_list,sensors=["rgb","depth","segmentation", "imu0"])

    def setSegmentation(self, label_dict):
        for objectname, segmentationID in label_dict.items():
            success = self.client.simSetSegmentationObjectID("[\w]*"+objectname+"[\w]*", segmentationID, True)
            if "SM_KI-84" in objectname:
                label_dict["Airplane"] = label_dict[objectname]
                del label_dict[objectname]
            if "InstancedFoliageActor" in objectname:
                label_dict["Seaweed"] = label_dict[objectname]
                del label_dict[objectname]
            if "SM_Sub" in objectname:
                label_dict["Submarine"] = label_dict[objectname]
                del label_dict[objectname]
            if "rock" in objectname:
                label_dict["Rock"] = label_dict[objectname]
                del label_dict[objectname]



        
        
        return label_dict
    

    def saveIntrinsics(self, vehicles=["auv0"], cameras = ["front_left","front_right","bottom_center"],sensors=["rgb","depth","segmentation","imu0"]):

        # sensor_yaml = {}
        for vehicle in vehicles:
            for camera in cameras:
                for sensor in sensors:
                    if not "imu" in sensor:
                        # Projection matrix
                        projection_matrix = self.client.simGetCameraInfo(camera_name = camera, vehicle_name = vehicle).proj_mat
                        camera_settings_dict = self.settings["Vehicles"][vehicle]["Cameras"][camera]["CaptureSettings"]
                        # Distortion params
                        target_distortion = self.config["distortion_coeffs"][camera]
                        self.client.simSetDistortionParams(camera_name = camera, distortion_params=target_distortion ,vehicle_name = vehicle)
                        distortion_params = self.client.simGetDistortionParams(camera_name = camera, vehicle_name = vehicle)
                        # Intrinsics
                        width = camera_settings_dict[0]["Width"]
                        height = camera_settings_dict[0]["Height"]
                        fov = camera_settings_dict[0]["FOV_Degrees"]
                        intrinsic_matrix = self.intrinsicsfromSettings(width,height,fov)
                        # Extrinsics
                        x = self.settings["Vehicles"][vehicle]["Cameras"][camera]["X"]
                        y = self.settings["Vehicles"][vehicle]["Cameras"][camera]["Y"]
                        z = self.settings["Vehicles"][vehicle]["Cameras"][camera]["Z"]
                        roll = self.settings["Vehicles"][vehicle]["Cameras"][camera]["Roll"]
                        pitch = self.settings["Vehicles"][vehicle]["Cameras"][camera]["Pitch"]
                        yaw = self.settings["Vehicles"][vehicle]["Cameras"][camera]["Yaw"]
                        extrinsic_matrix = self.extrinsicsfromSettings(x,y,z,roll,pitch,yaw)

                        # TODO: dont assume all camera types have same in/extrinsics
                        campath = self.sensor_paths[sensor][camera]
                        sensor_yaml = {
                            "sensor_type": camera,
                            "comment": sensor,
                            "T_BS": [NoIndent(elem) for elem in extrinsic_matrix.tolist()],
                            "rate_hz": self.config["framerate"],
                            "resolution": [width,height],
                            "camera_model": "pinhole",
                            "intrinsics": [NoIndent(elem) for elem in intrinsic_matrix.tolist()],
                            "distortion_model": "radial-tangential",
                            "distortion_coefficients": distortion_params
                        }
                        self.saveJSONtoDataset(inputpath=None,data=sensor_yaml,filename="sensor.yaml",outputpath=campath)
                    else:
                        # TODO: what if there is more than one imu/sensor?

                        # Extrinsics
                        x = self.settings["Vehicles"][vehicle]["X"]
                        y = self.settings["Vehicles"][vehicle]["Y"]
                        z = self.settings["Vehicles"][vehicle]["Z"]
                        roll = self.settings["Vehicles"][vehicle]["Roll"]
                        pitch = self.settings["Vehicles"][vehicle]["Pitch"]
                        yaw = self.settings["Vehicles"][vehicle]["Yaw"]
                        extrinsic_matrix = self.extrinsicsfromSettings(x,y,z,roll,pitch,yaw)

                        sensorpath = self.sensor_paths[sensor]
                        sensor_yaml = {
                            "sensor_type": sensor,
                            "T_BS": [NoIndent(elem) for elem in extrinsic_matrix.tolist()],
                            "rate_hz": self.config["imurate"],
                            "angular_random_walk": self.settings["Vehicles"][vehicle]["Sensors"]["Imu"]["AngularRandomWalk"],
                            "giro_bias_stability_tau": self.settings["Vehicles"][vehicle]["Sensors"]["Imu"]["GyroBiasStabilityTau"],
                            "giro_bias_stability": self.settings["Vehicles"][vehicle]["Sensors"]["Imu"]["GyroBiasStability"],
                            "velocity_random_walk": self.settings["Vehicles"][vehicle]["Sensors"]["Imu"]["VelocityRandomWalk"],
                            "accel_bias_stability": self.settings["Vehicles"][vehicle]["Sensors"]["Imu"]["AccelBiasStability"],
                            "accel_bias_stability_tau": self.settings["Vehicles"][vehicle]["Sensors"]["Imu"]["AccelBiasStabilityTau"],
                        }
                        self.saveJSONtoDataset(inputpath=None,data=sensor_yaml,filename="sensor.yaml",outputpath=sensorpath)

        return True

    def intrinsicsfromSettings(self, width, height, fov):
        fov_rad = fov*pi/180
        fx = width /(2 * np.tan(fov_rad/2))
        fy = fx
        cx = width/2
        cy = height/2
        intrinsicMatrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=np.float32)
        return intrinsicMatrix

    def extrinsicsfromSettings(self,x,y,z,roll,pitch,yaw):
        rot_matrix = R.from_euler('xyz',[roll,pitch,yaw], degrees=True).as_matrix()
        h = np.array([0,0,0,1])
        t = np.array([[x],[y],[z]])
        T_0_sensor = np.hstack((rot_matrix,t))
        T_0_sensor = np.vstack((T_0_sensor,h))
        return T_0_sensor


        
    def createPaths(self, basepath ,dataset="MIMIR",sequence="SeaFloor",track = "track1",vehicles=["auv0"],
                    sensors=["rgb","depth","segmentation", "imu0"], 
                    cameras = ["front_left","front_right","bottom_center"],
                    create_dataset = True):
        ''' Create file tree '''
        self.sensor_paths = {}

        self.DATASET_PATH       = os.path.join(basepath, dataset)
        self.DATASET_SEQUENCE   = os.path.join(self.DATASET_PATH, sequence)

        trackpath = os.path.join(self.DATASET_SEQUENCE, track)
        if not os.path.isdir(trackpath):
            self.DATASET_TRACK   = trackpath
        else:
            ndirs = len(next(os.walk(self.DATASET_SEQUENCE))[1])
            self.DATASET_TRACK   = trackpath + '_' + str(ndirs)


        for vehicle in vehicles:
            self.DATASET_PATH_VEHICLE  = os.path.join(self.DATASET_TRACK,vehicle)
            self.DATASET_PATH_GT    = os.path.join(self.DATASET_PATH_VEHICLE,"pose_groundtruth")
            for sensor in sensors:                
                if not "imu" in sensor:
                    camdirs = [os.path.join(self.DATASET_PATH_VEHICLE, sensor,"cam"+str(i)) for i in range(len(cameras))]
                    self.sensor_paths[sensor] =  dict(zip(cameras, camdirs))
                else:
                    self.sensor_paths[sensor] = os.path.join(self.DATASET_PATH_VEHICLE, sensor)

        if create_dataset:
            '''
            CREATE DATASET FOLDERS
            '''
                
            self.createFolder(self.DATASET_PATH_GT)
            for name, path in self.sensor_paths.items():
                try:
                    self.createFolder(path)
                except:
                    # we're in the nested dict
                    for n,p in path.items():
                        self.createFolder(p)


    def loadJSON(self, file):
        with open(file) as json_file:
            data_dict = json.load(json_file)
        return data_dict

    # reset the Gates and hiding them underground
    def resetRacingGates(self):
        # hide all the gates underground
        for i in range(self.config.max_gate_number):
            self.moveGate(i+1, np.array([i, i, -60]).astype(float), 0)  

    def saveJSONtoDataset(self, inputpath = None, data = {}, filename = str(), outputpath = str()):  
        # save json to dict
        if inputpath is not None:
            with open(os.path.join(inputpath,filename)) as json_in:
                data = json.load(json_in)
        with open(os.path.join(outputpath,filename), "w+") as json_out:
            json.dump(data, json_out, cls=MyEncoder, indent=4)
        return True

    # get current position of UAV from simulation
    # returns [x, y, z, qw, qx, qy, qz]
    def getPoseUAV(self):
        dronePose = self.client.simGetVehiclePose()
        retVal = np.array(
            [dronePose.position.x_val, dronePose.position.y_val, dronePose.position.z_val, dronePose.orientation.w_val,
             dronePose.orientation.x_val, dronePose.orientation.y_val, dronePose.orientation.z_val])
        return retVal


    def getMAVCurrentPoseROSENU(self):
        
        drone_pose = self.client.simGetVehiclePose() # in NED
        drone_pose_enu_ros = np.array([drone_pose.position.x_val, - drone_pose.position.y_val, - drone_pose.position.z_val, #x, y, z
                                    drone_pose.orientation.w_val, drone_pose.orientation.x_val, - drone_pose.orientation.y_val, - drone_pose.orientation.z_val])       #qw, qx, qy, qz
        return drone_pose_enu_ros



    def getAirsimTimeStamp(self):
        return self.client.getMultirotorState().timestamp



    def requestSingleCamera(self,name:str):
        
        res = self.client.simGetImages(
                [
                    airsim.ImageRequest(name, airsim.ImageType.Scene, False, False),
                    airsim.ImageRequest(name, airsim.ImageType.DepthPlanar, True, False),
                    airsim.ImageRequest(name, airsim.ImageType.Segmentation, False, False)
                ]
            )


        return res#[0],None,None,None


    def convertnSave(self, stamp,camera_ID,
                    rgb = None , depth = None, segmentation = None, of = None):

        # save image
        stamp = str(stamp)
        if rgb is not None:
            img2uint = np.fromstring(rgb.image_data_uint8, dtype=np.uint8)  # get numpy array
            img_rgb = img2uint.reshape(rgb.height, rgb.width, 3) 
            if not img_rgb.size :
                return False
            path = self.sensor_paths["rgb"][camera_ID]
            filename = stamp+".png"
            cv2.imwrite(os.path.join(path,filename), img_rgb)
            self.saveImgStamp(stamp,path,filename)
        if segmentation is not None:
            img2uint = np.fromstring(segmentation.image_data_uint8, dtype=np.uint8)  # get numpy array
            img_seg = img2uint.reshape(segmentation.height, segmentation.width, 3) 
            if not img_seg.size:
                return False
            path = self.sensor_paths["segmentation"][camera_ID]
            filename = stamp+".png"
            cv2.imwrite(os.path.join(path,filename), img_seg)
            self.saveImgStamp(stamp,path,filename)
        if depth is not None:
            path = self.sensor_paths["depth"][camera_ID]
            filename = stamp+".pfm"
            airsim.write_pfm(os.path.join(path,filename), airsim.get_pfm_array(depth))
            # filename = 'pfm16'+stamp+".pfm"
            # write_pfm16(os.path.join(path,filename), airsim.get_pfm_array(depth)) # retrieves float32 array
            # filename = 'exr32'+stamp+".exr"
            # write_exr(os.path.join(path,filename), airsim.get_pfm_array(depth))
            
            self.saveImgStamp(stamp,path,filename)
        if of is not None:
            path = self.sensor_paths["optical_flow"][camera_ID]
            filename = stamp+".pfm"
            write_pfm16(os.path.join(path,filename), airsim.get_pfm_array(of))
            self.saveImgStamp(stamp,path,filename)

        return True

    def saveImgStamp(self, stamp, path, filename):
        # get imu data
        ImuData = self.client.getImuData()
        if exist_and_not_empty(os.path.join(path,'data.csv')):
            with open(os.path.join(path,'data.csv'), 'a+') as f:
                writer = csv.writer(f)
                writer.writerow([stamp,filename])
        else:
            header = ["timestamp","filename"]
            with open( os.path.join(path,'data.csv'), 'w+') as f:
                writer = csv.writer(f)
                # write header
                writer.writerow(header)
                writer.writerow([stamp,filename])

        return True


    # print pose list
    # pose: [x, y, z, qw, qx, qy, qz]
    def printPose(self, pose):
        print(f"x: {pose[0]} y: {pose[1]} z: {pose[2]} qw: {pose[3]} qx: {pose[4]} qy: {pose[5]} qz: {pose[6]} ")

    # create folder at path with name
    def createFolder(self, path):
            os.makedirs(path)


    def getPositionUAV(self):
        dronePose = self.client.simGetVehiclePose()
        retVal = np.array(
            [dronePose.position.x_val, dronePose.position.y_val, dronePose.position.z_val, dronePose.orientation.w_val,
             dronePose.orientation.x_val, dronePose.orientation.y_val, dronePose.orientation.z_val])
        return retVal

    def captureAndSaveIMU(self, stamp):
        # get imu data
        ImuData = self.client.getImuData()
        if exist_and_not_empty(os.path.join(self.sensor_paths["imu0"],'data.csv')):
            with open(os.path.join(self.sensor_paths["imu0"],'data.csv'), 'a+') as f:
                writer = csv.writer(f)
                writer.writerow([stamp,
                                ImuData.angular_velocity.x_val,ImuData.angular_velocity.y_val,ImuData.angular_velocity.z_val,
                                ImuData.linear_acceleration.x_val,ImuData.linear_acceleration.y_val,ImuData.linear_acceleration.z_val,
                                ImuData.orientation.w_val, ImuData.orientation.x_val,ImuData.orientation.y_val,ImuData.orientation.z_val])
        else:
            header = ["timestamp","vx","vy","vz",
                      "ax","ay","az",
                      "q_0_kf_w","q_0_kf_x","q_0_kf_y","q_0_kf_z"]
            with open( os.path.join(self.sensor_paths["imu0"],'data.csv'), 'w+') as f:
                writer = csv.writer(f)
                # write header
                writer.writerow(header)
                writer.writerow([stamp,
                                ImuData.angular_velocity.x_val,ImuData.angular_velocity.y_val,ImuData.angular_velocity.z_val,
                                ImuData.linear_acceleration.x_val,ImuData.linear_acceleration.y_val,ImuData.linear_acceleration.z_val,
                                ImuData.orientation.w_val, ImuData.orientation.x_val,ImuData.orientation.y_val,ImuData.orientation.z_val])

        return True

    def captureAndSavePose(self, stamp):
        pose = self.getPositionUAV()
        if exist_and_not_empty(os.path.join(self.DATASET_PATH_GT,'data.csv')):
            with open(os.path.join(self.DATASET_PATH_GT,'data.csv'), 'a+') as f:
                writer = csv.writer(f)
                writer.writerow([stamp]+ pose.tolist())
        else:
            header = ["timestamp","t_0_kf_X","t_0_kf_Y","t_0_kf_Z",
                      "q_0_kf_w","q_0_kf_x","q_0_kf_y","q_0_kf_z"]
            with open( os.path.join(self.DATASET_PATH_GT,'data.csv'), 'w+') as f:
                writer = csv.writer(f)
                # write header
                writer.writerow(header)
                writer.writerow([stamp]+ pose.tolist())

    def captureAndSaveImages(self,stamp):
        # AirSim API rarely returns empty image data
        # 'and True' emulates a do while loop
        loopcount = 0
        while (self.create_dataset and True):

            # get images from AirSim API
            left_rgb, left_depth, left_seg = self.requestSingleCamera("front_left")
            right_rgb, right_depth, right_seg= self.requestSingleCamera("front_right")
            bottom_rgb, bottom_depth, bottom_seg= self.requestSingleCamera("bottom_center")
            left_saved = self.convertnSave(stamp,"front_left",left_rgb,left_depth,left_seg)
            right_saved = self.convertnSave(stamp,"front_right",right_rgb,right_depth,right_seg)
            bottom_saved = self.convertnSave(stamp,"bottom_center",bottom_rgb,bottom_depth,bottom_seg)


            
            # check if image contains data, repeat request if empty
            if left_saved and right_saved and bottom_saved:
                break  # end of do while loop
            else:
                loopcount += 1
                print(f"left imgs saved: {left_saved}, right imgs saved:{right_saved}, bottom imgs saved: {bottom_saved}" )
                print("airsim returned empty image." + str(loopcount)) 

    # capture Image and Gate Data
    def captureDataAndPauseSimulation(self, rate_handler):
        # Pause simulation to capture image and current data
        self.client.simPause(True)
        # generate an unique name of the image
        stamp = self.getAirsimTimeStamp() 
        # measure processing time
        time_real_start = time. time()

        self.captureAndSavePose(stamp)
        self.captureAndSaveIMU(stamp)
        if rate_handler == 0:
            self.captureAndSaveImages(stamp)

        time_real_end = time. time()

        # print("Time taken to capture image and data: " + str(time_real_end - time_real_start))
        self.client.simPause(False) # resume simulation


if __name__ == '__main__':

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    x = MIMIRrecorder(client, create_dataset=True)


    # wait for the mission to complete
    i = 0
    while (True):
        if i == 30:
            break # testing stuff        

        # capture data here        
        x.captureDataAndPauseSimulation(i%x.rate_divider)
        
        time.sleep(1/x.config["imurate"]) # sleep time reflect how fast we will sample new trajectory
        i += 1

