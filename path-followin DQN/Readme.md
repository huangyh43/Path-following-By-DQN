# File list
VrepApi: the API to connect V-rep simulator with python

VrepScene: the simulator scenes of motorbike in path-following (Running and Sensing)

Save: the datastore for trained network

control.py: To check whether it can connect the robot in Vrep and successfully control it

Main source list:
|-- environment.py
|--|-- To design the connect to simulator with ROS
|--|-- To design the status information of robot running
|--|-- To design the Reward function for each robotic action in different status information
|-- DQNAgent.py
|--|-- To design the Network structure for DQN (A Reinforcement Learning method)
|--|-- To save or load the learned model
|-- train_ctrl.py: Iterative running and learning a network
|-- test_ctrl.py: Using the learned network to control robot action
