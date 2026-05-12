UR5e Onboarding Guide - VXLab 

Virtual Experiences Laboratory, May 2024 

Key configuration components: 

Hardware (MAC) and IP addresses 

OS and ROS versions 

Demo scripts 

Safety: 

Plant risk assessment 

Safe Work Instruction 

Example activity risk assessment 

Development box (IP 10.234.7.84; MAC b8:ca:3a:7f:c1:37) 

Arm 

via Controller (IP 10.234.6.49; MAC 00:30:d6:2e:b2:aa) 

UR5e MoveIt Git URL: https://github.com/Kibibibit/UR5e_Env 

Gripper 

via EyeBox (IP 10.234.6.47; MAC 18:fd:74:e4:fc:18) 

Camera 

connected directly to Dev box via USB3 

 

README.md in the github repository has more detailed instructions. 

Installation and setup guide 

Initial setup 

Make sure to install docker, following these guides: 

https://docs.docker.com/engine/install/ubuntu/ 
https://docs.docker.com/engine/install/linux-postinstall/ 

Once docker is installed, install docker-compose: 
sudo apt install docker-compose 

Run the following commands: 

cd ~/UR5e_Env 
./docker-build.sh 
./docker-start.sh 

The docker should now be running. You can run ./docker-attach.sh to get access to the docker to run commands. 

Moveit Install (Optional, Already complete on VXLab PC) 

To install moveit, attach to the docker, and type the following commands: 
cd ~/.post-install 
./moveit_install.sh 
This will take about 30 minutes to execute, but should only need to be done once. 

 

Running Arm Drivers 

Teach Pendant 

On the teach pendant, go Open>>Installation>>RemoteROS. If asked to update, do so. 

Select Program>>URCaps>>External Control 

Verify that it says control by PC, and that the IP and port match the PC. (Port should be 50002) 

Turn on the Arm, ensuring E-Stop is released. 

PC 

Open a terminal, and type: 
cd ~/UR5e_Env 

If the docker has not been started, follow the steps above. Then attach to the docker with: ./docker-attach.sh 

Once in the docker, type: 
ur_driver 

A terminal with Rviz showing the state of the robot should pop up. 

Teach Pendant 

Press the “Play” button (Arrow Icon) in the bottom right of the screen and select either option. 

The PC terminal window should say something about the program being received and the robot being ready for commands. 

PC 

Open a new terminal tab, and type: 
cd ~/UR5e_Env 
./docker-attach.sh 

You should now be attached to the docker. Then run: 
moveit_config_driver 

 A new Rviz display should show, with a blue ball representing the Tool Control Point. You can click and drag it around to move it. 

Click “plan” to visualise the trajectory of the arm, and then “execute” to move the Arm.

Kibibibit
UR5e_Env
Repository navigation
Code
Issues
Pull requests
Agents
Actions
Projects
Security and quality
Insights
Owner avatar
UR5e_Env
Public
Kibibibit/UR5e_Env
Go to file
t
T
Name		
LaskarisD
LaskarisD
added comments
51a675b
 · 
2 years ago
.helper-scripts
Added some formatting to print logs
2 years ago
blender-files
Added blender files
2 years ago
workspace
added comments
2 years ago
.gitignore
Updated gitignore
2 years ago
.gitmodules
Added find object2d
2 years ago
Dockerfile
Added open cv
2 years ago
README.md
added setup instructions, known issues
2 years ago
docker-attach.sh
Added a UR driver start script
2 years ago
docker-build.sh
Fixed build if statement
2 years ago
docker-compose.yml
removed moveit workspace from docker compose
2 years ago
docker-delete.sh
Neatened up bash scripts
2 years ago
docker-kill.sh
Dockerfile updates
2 years ago
docker-start.sh
Sorted moveit py install
2 years ago
docker-stop.sh
Sorted moveit py install
2 years ago
Repository files navigation
README
UR5e_Env
A Ros2 environment for working with Moveit2 on the ur5e arm.

Scripts
Name	Function
docker-build.sh	Builds the docker container. It also completely deletes the previous image when run, so be aware of that. It will not delete anything you've placed in /workspace/src however, as that is stored in a volume. Adding the -y flag will skip asking for confirmation.
docker-start.sh	Runs the docker container in headless mode. You can use docker-attach.sh to access it.
docker-stop.sh	Stops the docker container without destroying it, unlike docker-kill.sh
docker-attach.sh	Provides terminal access to the docker container. You can exit with CTRL+D without destroying the container.
docker-kill.sh	Kills and removes the container. Adding the -y flag will skip asking for confirmation.
docker-delete.sh	Deletes the ROS docker image from the system. This is needed to clear some errors that can occur when rebuilding the container, and is run automatically by docker-build.sh. The -y flag can skip asking for confirmation.
Installation
Requirements
Docker - Don't use apt without following this guide!
Docker Post-install - If you don't follow this, it won't work!
docker-compose - sudo apt install docker-compose
Drivers and Aliases
The docker container contains several drivers and aliases for running different parts of the codebase.

arm_drivers - This runs the arm, gripper and camera drivers.
moveit_config_driver - This runs the moveit drivers, along with our custom moveit action server.
find_object_2d - Boots findobject2d, using its findobject3d launch file.
realsense_driver - This boots just the camera driver, if needed.
All of these aliases are set in workspace/testing_scripts/helper-aliases.sh, and can be modified or added to there.

Usage
Starting Up
Run the script ./docker-build.sh. This builds the docker container, and will take 5+ minutes to run on the first build. Future builds will be signifigantly faster.
Run the script ./docker-start.sh. This will start the docker container, and you are now ready to start working.
Attaching to the environment
Bash
To access the docker container, to run scripts, you can use ./docker-attach.sh. This will open a terminal inside the container for running commands.

VSCode
You can also access the workspace inside the docker container with the vscode exension dev containers. Once the container is running, hit ctrl+shift+P, and type attach, and find the option: Dev Containers: Attach to running container. Select the container, and when asked what folder to use, select /home/rosuser/workspace/. You can now develop inside the container. All your changes will be saved into the workspace folder on your local machine.

Running UR5e Drivers
UR Controller and Gripper Controller
First, on the UR5e teach pendant, select: Open>>Installation>>RemoteRos. If asked to update the program, select Update Program.
Then, go to the Program tab, and under URCaps, add External Control. Make sure the IP matches the PC IP, and that the port is 50002.
Turn ON the arm, ensuring that the E-Stop is released.
Attach to the container (./docker-attach.sh) and run:

arm_drivers
An RViz display showing the current state of the robot should appear.
If you want to disable RVIZ, you can add --no-rviz to the end of the command. If you're working without the gripper, you can add --no-gripper to disable it.

MoveIt
In a new terminal tab, attach to the container (./docker-attach.sh)
Run:

moveit_config_driver
You can now move the blue sphere to move the arm around, and use plan to visualise the trajectory of the movement, and then execute to move the arm.
This is needed for moveit commands to work from other packages, so if you want to disable RVIZ, you can add --no-rviz to the end of the command. If you're working without the gripper, you can add --no-gripper to disable it

Creating new Packages
To create a new package, attach to the docker, and go into ~/workspace/src and run the package create command. Make sure to run your build commands in ~/workspace and not src

Shutting down
Stopping the container
./docker-stop.sh will shut down the container, but leave it for future use. It can be restarted with ./docker-start.sh

Destroying the container
If you need to rebuild the container, you should run ./docker-delete.sh, which will wipe the container and require a rebuild to use again. You shouldn't have to do this very often.

Playing Connect4 Package for UR5e Cobot
Setup
The board is found in /workspace/src/par_pkg/objects under the filename 0.png. Ensure that the board is printed on A3 paper with a margin width of 0.

Secure board on a table to ensure movement cannot occur during gameplay

Start docker environment and attach to it

Ensure the workspace is built using the command build_workspace

Start up UR5e and Moveit drivers

Move arm directly above board with camera facing down. The best position is with camera having a birds eye view of the top row with the Human Drop Zone and Robot Piece. This can be achieved directly using the teach pendant or using the shell script waypoint_move.sh under ~/workspace/testing_scripts

Note: waypoint_move.sh requires arguments of position x, y, z and roation of the end effector which are all floats. This can be ran as ./waypoint_move.sh <X> <Y> <Z> <rotation> an example of this would be ./waypoint_move.sh -0.5 0 0.2 0

Launch Find Object 2D using the command find_object_2d and ensure that the detecting the board object (will state that object is detected in the console)

Launch the Connect4 program by running the command ros2 launch par_pkg par_pkg.launch.py

The node is up and ready when the console says it ahs found the grid and is waiting for the human piece

Playing
This has been designed to play with game pieces made with 2x2 lego blocks that have been stacked to 2 blocks each game piece. 25 pieces per player is a suitable amount to play the game.

To start playing place a block in one colour at the top of the column in the Human Drop Zone where you would like the piece to fall to
If the robot has detected the piece it will confirm in the console and display a three second countdown to start its process of moving the block
The robot will move the block to where it would fall to in a regular connect4 board
After this the console will say it is waiting for a robot piece. Place a game piece in the grid square labeled Robot Piece
The arm upon recognition should grab the piece and move it to the AI's corresponding response. Execution is complete when the console is asking for the players turn.
Repeat steps 1-5 until someone wins in connect 4
Known Issues
Robot is known to come across issues with precision to the the players piece and appears to stop on a randomly while moving the robots piece on a rare instance. The robot is able to mostly recover and still play the game.

Issues also arise where it rarely has detected a piece in the wrong grid square when attempting, making the current state of the game incorrect. Game is able to continue, just with the player's piece in the wrong spot.

Universal robots has a known bug where the connection to the PC will drop if no action is sent to the robot, which will stop the robots ability to move its arm. Restarting the connection on the pendant most of the time will fix this issue, however sometimes drivers have to be restarted.

Moveit driver is also known to timeout and refuse any commands, in which case it may be needed to restart the stack

Camera has a black spot on the depth sensor where it cannot see anything in that spot, possibly attributed to realsense driver reading the stereo camera as mono. Move board to ensure that all grid squares are visible, viewing the image in rviz under the topic /camera_image can help achieve this.

Credits
Sam Griffiths (RMIT)
Jasper Avice Demay (RMIT)
FindObject2D
Kiyokawa Takuya