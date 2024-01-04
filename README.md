# FRC 2024
Team 418's 2024 robot code. The robot's code is written in Java and is based off of WPILib's Java command based control system.

The code is organised into several subsystems, each responsible for a different aspect of the robot's functionality. This document explains this code.

## [Subsystems](src/main/java/frc/robot/subsystems)
### [Drive Subsystem](src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java)
The drive subsystem controls the drivetrain of the robot, which is a 4 wheel swerve drive, using MAXSwerve modules. Each corner of the robot is driven by a set of a single NEO brushless motor, and a NEO 550 motor for wheel rotation, providing ample power for our robot.
The NEO motor also provides the benefit of having a built-in encoder to accuately measure wheel rotations. The motors are connected to the wheels by a gearbox with a 4.71:1 gear ratio.
We also implement a closed loop control system for our drivetrain, using the yaw axis gyroscope on the navX2 IMU for feedback. This allows the robot to maintain orientation when being hit by other robots, driving over rough terrain, etc. This closed loop control includes a rudimentary traction control system, allowing the robot to limit wheel slip. The IMU is also used to aid in automatically balancing the robot in the pitch axis.
Lastly, the drivetrain also implements odometry and visual localisation through AprilTags for accurate pose estimation, which allows the robot to use the IMU, encoder, and camera data to accuately determine its position on the field. We use this information to do several things, including pathfinding on the field, avoiding known obstacles. This allows for automating high-precision common tasks, reducing the need for driver skill.

Key points:
* 4 wheel swerve drive, using REV MAXSwerve modules
* Powered by 4 NEO brushless motors, NEO 550 motors used for rotation
* Closed loop drive control system with traction control
* Odometry for accurate positioning during autonomous and teleop
* Custom pathfinding algorithm for automatically driving to points of interest

### [Vision Subsystem](src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java)
The vision subsystem interfaces with the vision coprocessor, providing accurate localisation data to the rest of the robot program. The coprocessor (a MinisForum UM560XT) runs [PhotonVision](https://photonvision.org), and calculates the position of each connected camera in space based on the known position of the AprilTags in view. The robot then calculates the it's own position based on the known location of the cameras on the robot.
The vision subsystem runs a separate thread on the roboRIO, getting the latest pose estimates from the coprocessor for each camera.

Key points:
* X86 Mini PC used as coprocessor (MinisForum UM560XT)
* PhotonVision
