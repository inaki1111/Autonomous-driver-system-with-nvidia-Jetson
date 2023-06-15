# Autonomous Vehicle Navigation using NVIDIA Jetson and YOLOv5

This project was created for the class Intelligent Robotics Systems in collaboration with Manchester Robotics. It is primarily developed by three students: Hilda Beltran, Diego Diaz, and Iñaki Roman. The aim of this project is to utilize the Puzzlebot provided by Manchester Robotics and create an autonomous vehicle capable of navigating through a track and detecting signals using YOLOv5 and computer vision techniques.

## System Requirements

- NVIDIA Jetson Nano (2GB)
- Manchester Robotics Hackerboard
- Ubuntu 18.04
- ROS Melodic
- Python 2 and 3
- YOLOv5 with custom dataset

## Installation

To boot an NVIDIA Jetson with Ubuntu 18.04, follow these instructions:

Obtain the necessary hardware:

NVIDIA Jetson Nano (2GB) development kit.
MicroSD card (minimum 16GB recommended).
Power supply for the Jetson Nano.
Download the Ubuntu 18.04 image:

Visit the official NVIDIA Developer website at https://developer.nvidia.com/jetson-nano-sd-card-image.
Download the Ubuntu 18.04 SD card image specifically designed for the Jetson Nano.
Flash the Ubuntu image onto the microSD card:

Insert the microSD card into your computer.
Use a tool like Etcher (available at https://www.balena.io/etcher/) to flash the downloaded Ubuntu 18.04 image onto the microSD card.
Select the downloaded image file and the microSD card as the target device.
Start the flashing process and wait for it to complete.
Insert the microSD card into the Jetson Nano:

Locate the microSD card slot on the Jetson Nano board.
Carefully insert the microSD card into the slot until it is fully seated.
Power on the Jetson Nano:

Connect the power supply to the Jetson Nano.
Power on the device.
Initial setup and configuration:

Connect a monitor, keyboard, and mouse to the Jetson Nano for the initial setup.
Follow the on-screen instructions to complete the Ubuntu 18.04 setup process.
Set up the desired preferences, network settings, and user account.
Update the system:

Open a terminal on the Jetson Nano.
Run the following commands to update the system:
sql
Copy code
sudo apt update
sudo apt upgrade
Your NVIDIA Jetson Nano is now booted with Ubuntu 18.04 and ready for further configuration and development.

Note: These instructions are specific to the NVIDIA Jetson Nano (2GB) model and Ubuntu 18.04. Make sure to follow the official documentation provided by NVIDIA for the specific Jetson model you are using to ensure the correct installation and setup process.




2. Set up the environment by installing the required dependencies. You can use the following commands as a starting point:
- Instructions for installing ROS Melodic can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu).
- Instructions for installing NVIDIA Jetson SDK Manager can be found [here](https://developer.nvidia.com/sdk-manager).

3. Install YOLOv5 and train it with a custom dataset. You can follow the steps outlined in this [Medium article](https://medium.com/mlearning-ai/training-yolov5-custom-dataset-with-ease-e4f6272148ad).


## Models Trained for Classification

We have chosen to use YOLOv5 for our autonomous vehicle navigation project due to its exceptional performance in object detection tasks. YOLOv5 is a state-of-the-art algorithm known for its speed and accuracy in real-time object detection. It can detect and classify objects with impressive precision, making it an ideal choice for our application. With YOLOv5, our vehicle will be able to accurately identify and track various objects, including signals, on the track in real-time. The model's efficiency and speed will enable our autonomous vehicle to navigate swiftly and make informed decisions based on the detected objects. Overall, YOLOv5 provides the robustness and reliability necessary for successful object detection, making it the optimal choice for our project.

This tutorial was taken directly from the ultralytics repository https://github.com/ultralytics/yolov5

See the YOLOv5 Docs for full documentation on training, testing and deployment. See below for quickstart examples.

Install
Clone repo and install requirements.txt in a Python>=3.7.0 environment, including PyTorch>=1.7.

git clone https://github.com/ultralytics/yolov5  # clone
cd yolov5
pip install -r requirements.txt  # install



Inference
YOLOv5 PyTorch Hub inference. Models download automatically from the latest YOLOv5 release.

import torch

# Model
model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom

# Images
img = "https://ultralytics.com/images/zidane.jpg"  # or file, Path, PIL, OpenCV, numpy, list

# Inference
results = model(img)

# Results
results.print()  # or .show(), .save(), .crop(), .pandas(), etc.



Inference with detect.py
detect.py runs inference on a variety of sources, downloading models automatically from the latest YOLOv5 release and saving results to runs/detect.

python detect.py --weights yolov5s.pt --source 0                               # webcam
                                               img.jpg                         # image
                                               vid.mp4                         # video
                                               screen                          # screenshot
                                               path/                           # directory
                                               list.txt                        # list of images
                                               list.streams                    # list of streams
                                               'path/*.jpg'                    # glob
                                               'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                               'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Training
The commands below reproduce YOLOv5 COCO results. Models and datasets download automatically from the latest YOLOv5 release. Training times for YOLOv5n/s/m/l/x are 1/2/4/6/8 days on a V100 GPU (Multi-GPU times faster). Use the largest --batch-size possible, or pass --batch-size -1 for YOLOv5 AutoBatch. Batch sizes shown for V100-16GB.

python train.py --data coco.yaml --epochs 300 --weights '' --cfg yolov5n.yaml  --batch-size 128
                                                                 yolov5s                    64
                                                                 yolov5m                    40
                                                                 yolov5l                    24
                                                                 yolov5x                    16
                                                                 
                                                                 

We choose to use the samallest factor of yolov5 which is yolov5n, the table bellow was taken straight from the ultralytics repository and scores the perfomance on different checkpoints


![155040757-ce0934a3-06a6-43dc-a979-2edbbd69ea0e](https://github.com/inaki1111/Autonomous-driver-system-with-nvidia-Jetson/assets/96154499/1f9a6e5b-327b-46d3-90f9-f18e995dda17)

Figure Notes
COCO AP val denotes mAP@0.5:0.95 metric measured on the 5000-image COCO val2017 dataset over various inference sizes from 256 to 1536.
GPU Speed measures average inference time per image on COCO val2017 dataset using a AWS p3.2xlarge V100 instance at batch-size 32.
EfficientDet data from google/automl at batch size 8.
Reproduce by python val.py --task study --data coco.yaml --iou 0.7 --weights yolov5n6.pt yolov5s6.pt yolov5m6.pt yolov5l6.pt yolov5x6.pt



Model	size
(pixels)	mAPval
50-95	mAPval
50	Speed
CPU b1
(ms)	Speed
V100 b1
(ms)	Speed
V100 b32
(ms)	params
(M)	FLOPs
@640 (B)
YOLOv5n	640	28.0	45.7	45	6.3	0.6	1.9	4.5
YOLOv5s	640	37.4	56.8	98	6.4	0.9	7.2	16.5
YOLOv5m	640	45.4	64.1	224	8.2	1.7	21.2	49.0
YOLOv5l	640	49.0	67.3	430	10.1	2.7	46.5	109.1
YOLOv5x	640	50.7	68.9	766	12.1	4.8	86.7	205.7
YOLOv5n6	1280	36.0	54.4	153	8.1	2.1	3.2	4.6
YOLOv5s6	1280	44.8	63.7	385	8.2	3.6	12.6	16.8
YOLOv5m6	1280	51.3	69.3	887	11.1	6.8	35.7	50.0
YOLOv5l6	1280	53.7	71.3	1784	15.8	10.5	76.8	111.4
YOLOv5x6
+ TTA	1280
1536	55.0
55.8	72.7
72.7	3136
-	26.2
-	19.4
-	140.7
-	209.8
-



## Usage

1. Start the ROS environment and launch the necessary nodes for communication with the Puzzlebot and other components.

2. Run the main navigation script to initiate the autonomous vehicle navigation system. You can use the following command as an example:



3. Monitor the vehicle's progress, detected signals, and any debug information through the provided user interface or visualization tools.

## Contributing

Contributions to this project are welcome. If you encounter any issues or have suggestions for improvements, please open an issue or submit a pull request. When contributing, please follow the coding standards and conventions used in the project.

## License

[MIT License](LICENSE)

## Acknowledgments

We would like to express our gratitude to Manchester Robotics for providing the Puzzlebot and their support throughout this project. We are thankful for the guidance and instruction provided by the following professors at Tec de Monterrey:

- Dr. Jesús Arturo Escobedo Cabello
- Dr. Jose Antonio Cantoral Ceballos
- Dr. Josué González García
- Dr. Francisco Javier Navarro Barrón

We would also like to acknowledge the following individuals who provided additional knowledge and assistance during the development of this project:

- Enrique Martinez
- Javier Suarez
- Daniel Figueroa
- Diego Alfonso

For those seeking more knowledge on deep learning, we highly recommend checking out Dr. Jose Antonio Cantoral's YouTube channel for insightful content: [Dr. Jose Antonio Cantoral on YouTube](https://www.youtube.com/@PepeCantoralPhD)

## Authors

- Hilda Beltran
- Diego Diaz
- Iñaki Roman

