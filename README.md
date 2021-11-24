# Mav Robotics 2021 FTC Robot
## Development Environment Overview
We use the following tools/libraries to do development
- [Android Studio](https://developer.android.com/studio/index.html) - The IDE for programming, compiling, and installing the code to the robot
- [FTC Robot Controller](https://github.com/FIRST-Tech-Challenge/FtcRobotController) - This is the base Software Development Kit (**SDK**) we use for interacting with the Robot
- [FTC Lib](https://github.com/FTCLib/FTCLib) - A set of libraries that emulate the code structure used for FRC programming. It provides a lot of simple methods and abstractions to use hardware and vision more easily
- [Github]([https://](https://github.com/Mav-Robotics/2021-FTC-Robot)
  
## Environment Setup
1. Install Android Studio by following the directions [here](https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/android-studio-guide.pdf)
2. If you are starting from scratch (normally you won't be, a mentor will set the project up for you) you can clone the current season SDK from [here](https://github.com/FIRST-Tech-Challenge/FtcRobotController) into your directory
3. Launch Android Studio
4. [Setup Android Debug Bridge](https://blog.jcole.us/2017/04/13/wireless-programming-for-ftc-robots/#add-several-useful-external-tools) which allows you to talk to the Robot over WiFi.

# Current Robot Configuration
## Project layout
The project is separated into several packages
- `org/firstinspires/ftc/teamcode` - This is the base of the source code and contains the RobotMap and RobotTeleop
  - RobotMap - Central place for all varialbes, motor speeds, and other definitions
  - RobotTeleop - The main Teleop Mode used for competition
- `org/firstinspires/ftc/teamcode/autos` - All Autonomous opmodes
- `org/firstinspires/ftc/teamcode/commands` - Commands used by Teleop and Autonomous commands
- `org/firstinspires/ftc/teamcode/subsystems` - The robot is broken down into subsystems that can run independently and can be controlled by commands in Teleop and Autonomous modes

## Controllers
### Overview
  The robot uses both a Rev Control Hub and a Rev Epansion hub. These must have the latest firmware installed. You can find the [instructions here](https://docs.revrobotics.com/rev-hardware-client/expansion-hub/updating-expansion-hub).
  
  The expansion hub connects to the control hub with a power cable and RS458 cable. Both of these are required.

  **To upgrade the expansion hub you must also conenct a USB cable from the Control Hub to the USB-C port of the Expansion Hub**

## Motors
### Overview
- Currently using 
## Servos
## Sensors