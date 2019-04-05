# ros\_posenet
## Overview
人間検出をPoseNetで行うROSパッケージです。

## 動作確認環境
  * ROS Kinetic
  * NodeJS 8.x 
  * Cuda 9.0 + cuDNN 7.1 (for GPU acceleration only)
  * PoseNet  
    ROS wrapper for PoseNET library in NodeJS
    PoseNet demo: [Here](https://storage.googleapis.com/tfjs-models/demos/posenet/camera.html)

## Setup
  *  このワークスペースをcloneした後、buildしてください。
  * パッケージの中で`npm install`を実行してください。

  * もしオフラインで利用する場合は以下のファイルのURLをlocalhost:8000に変更してください。  
     `..node_modules/@tensorflow-models/posenet/dist/checkpoints.js`  
     `..node_modules/@tensorflow-models/posenet/dist/posenet.js`  

## 実行
  ・カメラの情報から関節を検出する場合
  * `bash camera_posenet.sh`

  ・キネクトの情報から関節を検出する場合
  * `bash kinect_posenet.sh`

## setting
Following ROS parameters should be set:  

  *  `gpu: (true / false)` - Specifies if GPU acceleration should be used  
  * `topic` - Uncompressed RGB8 encoded image topic  
  * `out_topic` -  specifies topic for result output.   Output topic publishes JSON string that needs to be decoded as `std_msgs/String` message  
  *  Algorithm parameters to adjust performance. See [launch file](launch/posenet.launch) for full list> References could be found [PoseNet Official Github](https://github.com/tensorflow/tfjs-models/tree/master/posenet#inputs-2)


## Launch Camera Node

### Subscribe Topic

* **`/usb_cam/image_raw`** usbカメラからの画像を受け取り( sensor_msgs/Image )


### Publish Topic

* **`/ros_posenet/poses`** pose情報（奥行き無し）の結果( ros_posenet/Poses )



## Launch Kinect Node

### Subscribe Topic

* **`/ros_kinect/color`** kinectからの画像を受け取り( sensor_msgs/Image )


### Publish Topic

* **`/ros_posenet/poses`** pose情報（奥行きあり）の結果( ros_posenet/Poses )

## Limitations
 * Only multiple pose detection implemented
 * Requires internet to download the model weights
 * Only ROS tpopics with RGB8 encoding are supported as inputs

