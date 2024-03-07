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
* Powered by 4 NEO Vortex brushless motors, NEO 550 motors used for rotation
* Closed loop drive control system with traction control
* Odometry for accurate positioning during autonomous and teleop
* Custom pathfinding algorithm for automatically driving to points of interest

### [Vision Subsystem](src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java)
The vision subsystem interfaces with the vision coprocessor, providing accurate localisation data to the rest of the robot program. The coprocessor (a MinisForum UM690S) runs [PhotonVision](https://photonvision.org), and calculates the position of each connected camera in space based on the known position of the AprilTags in view. The robot then calculates the it's own position based on the known location of the cameras on the robot.
The vision subsystem runs a separate thread on the roboRIO, getting the latest pose estimates from the coprocessor for each camera.
It also looks for notes on the ground in front of the intake, allowing for aiming at game objects automatically.

Key points:
* X86 Mini PC used as coprocessor (MinisForum UM560XT)
* PhotonVision

### [Shooter Subsystem](src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java)
The shooter subsystem is composed of 4 motors, with two flywheels each being driven by a NEO Vortex, as well as an indexer motor which transports a note from the ground intake into the shooter and an angle motor which adjusts the shooter's angle with a leadscrew mechanism (both controlled by a NEO). The indexer motor is also attached to a beam break, which is used to stop the intake when an object is present. The shooter also has 2 magnetic 
The shooter subsystem interfaces with the vision subsystem, waiting until an AprilTag is visible before shooting and adjusting the shooter's angle and flywheel speeds based off an interpolated curve of shooter states and the robot's estimated pose. The shooter also waits for the drive subsystem to orient the drivetrain towards the target. Aiming automatically accounts for the robot's velocity.
We also implement smooth motion, generating and executing a trapezoid motion profile as the shooter moves between states. Finally, the shooter subsystem allows the robot to intake from source.

Key points:
* 2 NEO Vortex flywheels
* 1 NEO leadscrew for angle adjust
* 1 NEO indexer
* Beam break for object presence detection
* Interfaces with vision for automating aiming, waiting for the robot to be oriented and calculating the correct state based on estimated pose while accounting for current velocity
* Smooth motion with a trapezoid motion profile
* Intake from source

### [Intake Subsystem](src/main/java/frc/robot/subsystems/Intake/IntakeSubsystem.java)
The intake subsystem powers a ground intake that feeds the note to the shooter and is composed of one NEO Vortex brushless motor.

Key points:
* Ground intake powered by a NEO Vortex brushless motor
* Feeds notes to the shooter subsystem

### [Climber Subsystem](src/main/java/frc/robot/subsystems/Climber/ClimberSubsystem.java)
The climber subsystem controls two telescoping arms on each side with two brushless NEO motors. We also use magnetic reed switches to act as hard limits for the climber.

Key points:
* Two telescoping arms each powered by a brushless NEO motor
* Limit switches to safeguard mechanism