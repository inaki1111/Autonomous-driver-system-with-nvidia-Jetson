markdown
Copy code
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

To install Ubuntu 18.04 on a laptop with dual boot and ROS, follow these instructions:

Prepare the installation media:

Download the Ubuntu 18.04 ISO file from the official Ubuntu website (https://ubuntu.com/download).
Create a bootable USB drive using software like Rufus (for Windows) or Etcher (for Windows, macOS, and Linux). Follow the instructions provided by the software to create the bootable USB drive.
Backup your data:

Before proceeding with the installation, it is highly recommended to backup all important data from your laptop to an external storage device. This ensures that your data is safe in case of any unforeseen issues during the installation process.
Adjust partition size (if required):

If you already have another operating system installed on your laptop, such as Windows, you may need to adjust the partition size to make space for Ubuntu. You can use disk management tools like GParted (https://gparted.org/) to resize existing partitions and create free space for Ubuntu installation.
Boot from the installation media:

Insert the bootable USB drive into your laptop.
Restart the laptop and access the BIOS/UEFI settings by pressing the appropriate key during startup (usually Esc, F2, or Del). Consult your laptop's manual or search online for the specific key to access the BIOS/UEFI settings.
In the BIOS/UEFI settings, change the boot order to prioritize the USB drive. Save the changes and exit the BIOS/UEFI settings.
Install Ubuntu 18.04:

The laptop should now boot from the USB drive and present the Ubuntu installation screen.
Select "Install Ubuntu" from the menu.
Follow the on-screen instructions to choose your language, keyboard layout, and other preferences.
When prompted to select the installation type, choose "Something else" (for dual boot).
In the partitioning screen, select the free space you created earlier and click the "+" button to create a new partition.
Create at least two partitions: one for the root file system ("/") and one for the swap space. You can allocate more partitions based on your specific requirements.
Make sure to select the correct partition to install the root file system ("/").
Complete the installation process by following the remaining on-screen instructions, including creating a user account and setting up a password.
Install ROS:

Once Ubuntu 18.04 is installed and you have logged in, open a terminal.
Follow the official ROS installation instructions for ROS Melodic (http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS on your laptop.
Proceed with the installation of ROS packages and dependencies as required by your specific project or application.
Dual boot configuration:

After installing Ubuntu and ROS, you should have the option to choose between Ubuntu and the other operating system (e.g., Windows) during startup.
Restart your laptop and select the desired operating system from the boot menu.
Congratulations! You have successfully installed Ubuntu 18.04 with dual boot and ROS on your laptop. You can now start using Ubuntu and ROS for your robotics projects.


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

```shell
sudo apt update
sudo apt upgrade
Your NVIDIA Jetson Nano is now booted with Ubuntu 18.04 and ready for further configuration and development.

Note: These instructions are specific to the NVIDIA Jetson Nano (2GB) model and Ubuntu 18.04. Make sure to follow the official documentation provided by NVIDIA for the specific Jetson model you are using to ensure the correct installation and setup process.

Yolov5
Install YOLOv5 and train it with a custom dataset. You can follow the steps outlined in this Medium article.

Models Trained for Classification
We have chosen to use YOLOv5 for our autonomous vehicle navigation project due to its exceptional performance in object detection tasks. YOLOv5 is a state-of-the-art algorithm known for its speed and accuracy in real-time object detection. It can detect and classify objects with impressive precision, making it an ideal choice for our application. With YOLOv5, our vehicle will be able to accurately identify and track various objects, including signals, on the track in real-time. The model's efficiency and speed will enable our autonomous vehicle to navigate swiftly and make informed decisions based on the detected objects. Overall, YOLOv5 provides the robustness and reliability necessary for successful object detection, making it the optimal choice for our project.

This tutorial was taken directly from the ultralytics repository https://github.com/ultralytics/yolov5

See the YOLOv5 Docs for full documentation on training, testing, and deployment. See below for quickstart examples.

Install
Clone the repo and install requirements.txt in a Python>=3.7.0 environment, including PyTorch>=1.7.

shell
Copy code
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt
Inference
YOLOv5 PyTorch Hub inference. Models download automatically from the latest YOLOv5 release.

python
Copy code
import torch

# Model
model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom

# Images
img = "https://ultralytics.com/images/zidane.jpg"  # or file, Path, PIL, OpenCV, numpy, list

# Inference
results = model(img)

# Results
results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
Inference with detect.py:

detect.py runs inference on a variety of sources, downloading models automatically from the latest YOLOv5 release and saving results to runs/detect.

shell
Copy code
python detect.py --weights yolov5s.pt --source 0                               # webcam
python detect.py --weights yolov5s.pt --source img.jpg                         # image
python detect.py --weights yolov5s.pt --source vid.mp4                         # video
python detect.py --weights yolov5s.pt --source screen                          # screenshot
python detect.py --weights yolov5s.pt --source path/                           # directory
python detect.py --weights yolov5s.pt --source list.txt                        # list of images
python detect.py --weights yolov5s.pt --source list.streams                    # list of streams
python detect.py --weights yolov5s.pt --source 'path/*.jpg'                    # glob
python detect.py --weights yolov5s.pt --source 'https://youtu.be/Zgi9g1ksQHc'  # YouTube
python detect.py --weights yolov5s.pt --source 'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream
Training
The commands below reproduce YOLOv5 COCO results. Models and datasets download automatically from the latest YOLOv5 release. Training times for YOLOv5n/s/m/l/x are 1/2/4/6/8 days on a V100 GPU (Multi-GPU times faster). Use the largest --batch-size possible, or pass --batch-size -1 for YOLOv5 AutoBatch. Batch sizes shown for V100-16GB.

shell
Copy code
python train.py --data coco.yaml --epochs 300 --weights '' --cfg yolov5n.yaml  --batch-size 128
python train.py --data coco.yaml --epochs 300 --weights '' --cfg yolov5s.yaml  --batch-size 64
python train.py --data coco.yaml --epochs 300 --weights '' --cfg yolov5m.yaml  --batch-size 40
python train.py --data coco.yaml --epochs 300 --weights '' --cfg yolov5l.yaml  --batch-size 24
python train.py --data coco.yaml --epochs 300 --weights '' --cfg yolov5x.yaml  --batch-size 16
Pretrained Checkpoints
Model	Size (pixels)	mAPval 50-95	mAPval 50	Speed CPU b1 (ms)	Speed V100 b1 (ms)	Speed V100 b32 (ms)	Params (M)	FLOPs @640 (B)
YOLOv5n	640	28.0	45.7	45	6.3	0.6	1.9	4.5
YOLOv5s	640	37.4	56.8	98	6.4	0.9	7.2	16.5
YOLOv5m	640	45.4	64.1	224	8.2	1.7	21.2	49.0
YOLOv5l	640	49.0	67.3	430	10.1	2.7	46.5	109.1
YOLOv5x	640	50.7	68.9	766	12.1	4.8	86.7	205.7
YOLOv5n6	1280	36.0	54.4	153	8.1	2.1	3.2	4.6
YOLOv5s6	1280	44.8	63.7	385	8.2	3.6	12.6	16.8
YOLOv5m6	1280	51.3	69.3	887	11.1	6.8	35.7	50.0
YOLOv5l6	1280	53.7	71.3	1784	15.8	10.5	76.8	111.4
YOLOv5x6 TTA	1280/1536	55.0/55.8	72.7/72.7	3136	26.2	19.4	140.7	209.8
Usage
Start the ROS environment and launch the necessary nodes for communication with the Puzzlebot and other components.

Run the main navigation script to initiate the autonomous vehicle navigation system. You can use the following command as an example:

shell
Copy code
python main_navigation.py
Monitor the vehicle's progress, detected signals, and any debug information through the provided user interface or visualization tools.
Contributing
Contributions to this project are welcome. If you encounter any issues or have suggestions for improvements, please open an issue or submit a pull request. When contributing, please follow the coding standards and conventions used in the project.

License
MIT License

Acknowledgments
We would like to express our gratitude to Manchester Robotics for providing the Puzzlebot and their support throughout this project. We are thankful for the guidance and instruction provided by the following professors at Tec de Monterrey:

Dr. Jesús Arturo Escobedo Cabello
Dr. Jose Antonio Cantoral Ceballos
Dr. Josué González García
Dr. Francisco Javier Navarro Barrón




