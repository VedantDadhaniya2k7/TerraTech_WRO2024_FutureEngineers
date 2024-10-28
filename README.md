WRO 2024 Future Engineers - Team TerraTech (Team code: 1844)
====

This repository contains engineering materials of the self-driven vehicle of Team TerraTech participating in the WRO Future Engineers competition in the season 2024.

## Content

* `Documentation` contains an Engineering Document describing our Robot, Its components, Why we selected each Component, Code, and, its implementation.
* `Team_Photos` contains photos of the team (an official one and one funny photo with all team members)
* `Vehicle_Photos` contains photos of the vehicle (from every side, from top and bottom)
* `Video` contains videos that demonstrate the robot's functionality including it completing 3 laps of the Open Challenge
* `Schematics` contains schematic diagrams of each electromechanical components (electronic components and motors) used in the vehicle and how they connect to each other and RaspberryPi and also one schematic with the complete circuit connections shown
* `Source_code` contains code of control software for all components which were programmed to participate in the competition
* `3D_models` contains the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. It contains the files in both STL and gcode format
* `Component_tests` contains code for testing and making sure each component of the whole robot is functional. Includes test codes for servo motor, rear motors, camera recognition, as well as the vl53l1x distance measurement sensor

## Introduction

### Team Introduction
![image](https://github.com/user-attachments/assets/1ba109ee-f440-4766-902e-4db60402014c)

_**Vedant Dadhaniya** (Left Side) - Team Lead, Head of Programming and 3-D Design_

Vedant is a STEM enthusiast who has recurringly been participating in the World Robot Olympiad since 2019 and this is his 4th time participating for it. He began by playing with jigsaw puzzles when he developed interest in LEGO which soon led to introduction to the world of robotics through EV3. Over the years, he has developed many skills in robotics including 3D part designing, electrical wiring, and languages such as Python, C#, HTML/CSS, and Java. He brings his computer vision skills with Raspberry Pi which he had worked on during the COVID Pandemic. He is an avid reader and also likes to play the guitar.

_**Devansh Harivallabhdas** (Right Side) - Head of Hardware and 3-D Printing_ 

Devansh is a Robotics and stem enthusiast from Ahmedabad. It is his first time at World Robot Olympiad. He is 14 and studies in class 9. As a child , Lego building was his favourite activity . He discovered his love for wires , motors and circuits in the Covid lockdown 4 years ago. Ever since he is pursuing it . He also has keen interest in Environmental applications ,Physics and chemistry and a passionate urban farmer. His dream is to create technologies and solutions that can impact the conservation of our environment.

### Reason for choosing the team name _TerraTech_

Terra is Latin for the word earth , since the theme for this WRO is Earth Allies. And Tech as we all know is the shortform of the word technology.




## About the Project

### Open Challenge Working Idea

![image](https://github.com/user-attachments/assets/9cfa5ad9-1015-4733-9848-602d0a3858d4)




## How to replicate the robot

First collect all the parts required as mentioned in our [Engineering document](http://bit.ly/3C0usWp)
and print all the parts as given in the `3D_models` folder of this repository

Now assemble all the 3d printed parts and the electromechanical components such that the fundamental component assembly looks like this:

![Robot design](https://github.com/user-attachments/assets/fd2e6c28-7161-4f5d-b01d-4daa3982a86d)

Then connect the jumper cables as per the following electrical circuit diagram:

![Electrical diagram](https://github.com/user-attachments/assets/33cc4a9a-c106-4b18-8678-df0ee51bfef2)

Before booting the RaspberryPi 4b, flash a 32GB microSD card with a Bookworm image and insert it into the microSD card slot of the RaspberryPi
Now load the Source code onto the raspberry pi using USB to USB communication or a portable data storage device.

Now in the terminal of the RaspberryPi, setup a virtual environment as such:

```> python3 -m venv <give a name to your virtual environment>```

Activate the virtual environment using the following command:

```> source <name of virtual environment>/bin/activate```

Now install the following libraries using ```> pip install <library name>``` :

* opencv-python
* adafruit-circuitpython-vl53l1x
* board
* digitalio
* numpy

Now the robot is ready to run

# References

* https://learn.adafruit.com/adafruit-vl53l1x/python-circuitpython
* https://learn.sparkfun.com/tutorials/qwiic-distance-sensor-vl53l1x-hookup-guide/python-examples
* https://docs.sunfounder.com/projects/raphael-kit/en/latest/appendix/install_the_libraries.html#create-virtual
* https://docs.sunfounder.com/projects/raphael-kit/en/latest/appendix/i2c_configuration.html#i2c-config
* https://www.youtube.com/watch?v=2bganVdLg5Q&t=42s
* https://github.com/World-Robot-Olympiad-Association/wro2022-fe-template
* https://www.youtube.com/playlist?list=PLGs0VKk2DiYyXlbJVaE8y1qr24YldYNDm
