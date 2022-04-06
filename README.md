# Turtlebot3 Photo Collection

This repo uses a turtlebot3 robot to capture, tag, and upload vision ML training photos to Microsoft Custom Vision. It utilizes a reinforcement learning policy developed using the turtlebot3_bonsai sample from the [ROS-bonsai-connector](https://github.com/microsoft/ROS-bonsai-connector) and the turtlebot3_cartographer node (SLAM) to drive the robot. The project was developed using ROS2 Foxy Fitzroy. 

# Getting Started
This repo assumes that the user has a physical turtlebot3, a turtlebot3 simulator in a Bonsai workspace, and familiarity with ROS. 

* export bonsai brain and deploy to your turtlebot3
* clone this repo and https://github.com/microsoft/ROS-bonsai-connector to your turtlebot3's ROS workspace
* populate the custom_vision.yaml file in the config folder with your project information
* `colcon build $$ source install/setup.bash`
* on the robot: `ros2 launch photo_collector_robot.launch.py`
* on your local machine: `ros2 launch photo_collector_local.launch.py`

At this point, you should be able to see the lidar data populating an RVIZ2 window. 

To change the state of the robot, use `ros2 param set /main state_machine_val <value>`

0 = Idle
1 = Mapping (this will utilize the Bonsai policy to navigate, recommended to train an 'avoid obstacles' policy)
2 = Photographing

For the photographing state, a target goal pose needs to be set. You can do this using the 'set goal pose' option in RVIZ or via the command line. 

The robot will take 30 photos based on the target pose and upload them to Custom Vision with a uuid tag. Custom vision recommends a minimum of 20-30 photos of an object for training a classification model. The tag may be renamed. 

## Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.opensource.microsoft.com.

When you submit a pull request, a CLA bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., status check, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## Trademarks

This project may contain trademarks or logos for projects, products, or services. Authorized use of Microsoft 
trademarks or logos is subject to and must follow 
[Microsoft's Trademark & Brand Guidelines](https://www.microsoft.com/en-us/legal/intellectualproperty/trademarks/usage/general).
Use of Microsoft trademarks or logos in modified versions of this project must not cause confusion or imply Microsoft sponsorship.
Any use of third-party trademarks or logos are subject to those third-party's policies.
