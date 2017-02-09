## Torque balancing controller using MATLAB 
This is the controller implemented on the humanoid robot iCub for balancing. The control strategy is `momentum-based`.
The control objective is the achievement of two different tasks: the primary task is a desired centroidal momentum dynamics. The secondary task is the stability of the robot's zero dynamics. 

### Run a simulation
To run a balancing simulation, open the script `initializeTorqueBalancing.m`. The user can set all the simulation parameters as balancing on one foot or two feet, enable robot movements, 
and set the integration options. The control gains and the desired center of mass trajectory are defined in the functions `gains.m` and `trajectoryGenerator.m` in the [config](config) folder. 

### Visualize the results
It is currently available an iDyntree iCub simulator to visualize the robot movements. Furthermore, the user can plot the forward dynamics integration results such that contact forces, control torques, the joint dynamics and CoP positions. 

### Matlab tools
Most of the tools used for simulation are inside the `tools` folder. This folder is divided into subfolders according to specific tools, e.g. all the functions related to robot dynamics, forward kinematics and state are inside the `tools/robotFunctions` folder. 
