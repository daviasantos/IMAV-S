

# <img src="FIGURES\LogoIMAVS.png" style="zoom:20%;" />

IMAV-S is a software-in-the-loop (SIL) flight dynamics & control simulator of multirotor aerial vehicles developed for R&D in LRA (*Laboratório de Robótica Aérea*) at *Instituto Tecnológico de Aeronáutica* (Brazil). The MAV physics, the actuators and sensors, and the control algorithms are coded in MATLAB script, with communicates via TPC socket with a Unity 3D game representing a virtual indoor flight arena. The simulator runs in (pseudo) real time, thus allowing a realistic piloting experience.

The physics as well as the flight control, guidance, and navigation algorithms are implemented in MATLAB scripts, using the object-oriented paradigm. 

IMAV-S is an easy-to-use SIL flight simulator intended to help people who are interested to learn or to do research and development on aerial robotics. One could say that IMAV-S behaves pretty much the same as a real drone flight in your lab's arena!

Hope you enjoy and share it!

For obtaining background material about MAV dynamics, control, guidance, and navigation, you can visit [my homepage](https://www.professordavisantos.com/). 


## How to use it

First of all, fork this repository into your GitHub profile and then download or clone it on your computer. 

To run the simulator, you need to download the zipped file containing the virtual indoor arena. You can find it in [this page](http://www.professordavisantos.com/imav-s/). After that, you need to extract this file on your computer. Then you will obtain a folder contains the executable file `IMAVArena.exe`. This virtual arena has been developed in Unity 3D by Mr. Nicolas Granese, who is a great guy!

This repo has one main folder named `MATLAB`, where you can find all the simulator source files.  
It contains two sub-folders: `Classes` and `Conversions`. The sub-folder `Classes` in turn contains one class for each functionality of the simulator (*e.g.*, `CControl` is a class which implements the flight control laws). On the other hand, the sub-folder `Conversion` contains basic math functions for coordinate conversions, signal saturation, and construction of a skew-symmetric matrix. 

In `\MATLAB` you also find the main function `IMAVS.m` as well as a data file named `Parameters.mat`, which contains all the simulation default parameters. 

If you want to change the default parameters in `Parameters.mat`, you can use the IMAVS MATLAB app. The installer for this app is also available `\MATLAB`. To install the IMAVS app, just click on `IMAVS.mlappinstall` from the MATLAB *current directory*. 

Finally, to execute the IMAV-S Simulator, I suggest using MATLAB 2019b (or later) and a PS4-compatible joystick (otherwise, you will need to modify the `CJoy` class accordingly.).  

**Steps to run IMAV-S (virtual arena with 3D visualization):**

1. Turn on the joystick. 
2. Run `~/VirtualIndoorArena/IMAVArena.exe`.
3. (Optional) Run the IMAVS app and make the desired modifications on the parameter set. There you can also find some instructions (*e.g.*, how to use the joystick).
4. From MATLAB, run `~/MATLAB/IMAVS.m`. It will show basic info on the command window, after which you can press ENTER to start.

After the above steps, the simulator will enter into a state machine; the current discrete state will be informed in both the game screen and MATLAB command line. 

5. If the state is READY, the MAV is stationary at the origin. At this point, you can arm the rotors (see the corresponding joystick command below).
6. After arming, the rotors start spinning, but the MAV keeps stationary. At this point, you can either disarm the rotors to go back to READY state or you can take off automatically (see the corresponding joystick command below).
7. After taking off, the MAV is automatically switched to MANUAL state. From this point on, you can land the MAV automatically (see the corresponding joystick command below). You can also fly it manually or even start following a sequence of waypoints automatically (this is the WAYPOINT discrete state).
8. If the system is in the WAYPOINT state, you can bring it back to MANUAL by just commanding the joystick axes with any command.     



## Joystick commands

The joystick commands are illustrated below. 

<img src="FIGURES\Joyfront1.png" style="zoom:30%;" />



<img src="FIGURES\Joyfront2.png" style="zoom:27%;" />



<img src="FIGURES\Joyback.png" style="zoom:31%;" />





## Comments about the implementation

The MATLAB part of IMAV-S is implemented using the object orientation paradigm. Here you can see the list of classes developed for this project:

* `CMav`: Includes the MAV parameters, input and output variables, and the functions to compute resulting torque and force, propellers' thrusts, and integration of the equations of motion.

* `CSensors`: Implements the models of the navigation sensor's measurements. 

* `CUncer`: Implements the force and torque disturbances, as well as magnetic interference. 

* `CState`: Implements the state machine which rules the control modes (MANUAL, WAYPOINT, TAKE-OFF, LANDING, ARMED, etc.) and transition between them. 

* `CControl`: Includes the flight controller parameters, input and output variables, and the functions to compute the attitude control, the position control, the control allocation, the reference filter, and the 3D attitude command.

* `CGuidance`: Implements waypoint-based guidance as an external law over the MAV in closed loop with position and attitude controllers. 
  
* `CNavigation`: Implements an attitude determination extended Kalman filter (EKF) and process the GPS measurements to estimate 3D position and velocity. 

* `CJoy`: Implements the interface with a Bluetooth joystick. 

* `CSocket`: Implements the TCP socket between the MATLAB and the Unity 3D app.

* `CData`: Prepares the message to send from MATLAB to the Unity 3D app. 

  

## Acknowledgment

I would like to thank a lot all my students and colleagues who somehow helped me to start this project. Many of the ideas and methods implemented in IMAV-S have been discussed in the graduate courses [MP-282 Dynamic Modeling and Control of Multirotor Aerial Vehicles](https://www.professordavisantos.com/modeling-control-mav/) and [MP-208 Optimal Filtering with Aerospace Applications](https://www.professordavisantos.com/category/courses/mp-208/), which I have offered since 2014 in the Aeronautics and Mechanics Graduate Program at [ITA/Brazil](https://www.ita.br). I am also grateful to the research agencies which supported this project: FAPESP (grant 2019/05334-0) and CNPq (302637/2018-4).









