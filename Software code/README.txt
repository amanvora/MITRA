The project is called Mapping based In-house Trained Robot for patient Assistance. It is the design of a prototype system communicating with a slave robot in a medical environmental.

The robot code has two stages:
1. Training: This stores a set of dynamically allocated coordinate points by the user as the robot traverses the path of the environment. Important/key points are stored. At the end of the training, the robot prepares a map of the coordinates and has the knowledge of the connectivity of each point with respect to the other.
2. Testing: The robot once trained is ready to fetch instructions from the system. The fetch instructions are about the name of the devices to be fetched from a storehouse and where to go and place the components. The robot plans a path to the warehouse from its current location. It traverses its path iteratively updating its current location. Once the devices are fetched it plans a path to the destination and traverses it to reach the destination.

Software requirements: 

1.	Arduino IDE : Libraries required
	i.	HMC5883L
	ii.	Wire.h

2.	Python: Libraries required
	i.	OpenCV
	ii.	Numpy
	iii.	speech_recognition
	iv.	pyaudio
	v.	serial

Hardware components used:
1.	Arduino UNO
2.	HMC5883L magnetometer
3.	L293D Motor driver
4.	DC motors (2)
5.	IR sensors (2)
6.	ZigBee modules (2)
7.	HC-SR04 ultrasonic module
8.	16x2 LCD display