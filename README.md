# Autonomous Vehicle Navigation using NVIDIA Jetson and YOLOv5

This project was created for the class Intelligent Robotics Systems in collaboration with Manchester Robotics. It is primarily developed by three students: Hilda Beltran, Diego Diaz, and Iñaki Roman. The aim of this project is to utilize the Puzzlebot provided by Manchester Robotics and create an autonomous vehicle capable of navigating through a track and detecting signals.

## System Requirements

- NVIDIA Jetson Nano (2GB)
- Manchester Robotics Hackerboard
- Ubuntu 18.04
- ROS Melodic
- Python 2 and 3
- YOLOv5 with custom dataset

## Installation

1. Clone the repository:

git clone https://github.com/inaki1111/Autonomous-driver-system-with-nvidia-Jetson.git
cd Autonomous-driver-system-with-nvidia-Jetson



2. Set up the environment by installing the required dependencies. You can use the following commands as a starting point:
- Instructions for installing ROS Melodic can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu).
- Instructions for installing NVIDIA Jetson SDK Manager can be found [here](https://developer.nvidia.com/sdk-manager).

3. Install YOLOv5 and train it with a custom dataset. You can follow the steps outlined in this [Medium article](https://medium.com/mlearning-ai/training-yolov5-custom-dataset-with-ease-e4f6272148ad).

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

