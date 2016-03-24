## Stack of task balancing

iCub balancing control implemented using a 'joint-space' controller. 

### Start the simulation

-open 'integrateForwardDynamics_js.m'

-at the top of the script, the user can define different types of simulations, for example 1 foot or 2 feet balancing, 
 activate the CoM movements, change the robot configuration. at the bottom of the script it is possible to modify the simulation time and the 
 integration time step both for the forward dynamics integrator ('ode15s') and for the inverse kinematics integrator

### Modify gains

-the gains can be modified opening the Matlab function 'gainsAndConstraints_js.m'

### Visualize more graphics

-open 'forwardDynamics_js.m'

-at the bottom of the script, add to the structure 'visual_param' the parameters you need to visualize

-open 'visualizer_js.m'

-add a 'figure' and 'plot' the parameters you seleced previously 

### Inverse kinematics

the inverse kinematics integrator is developed in the folder 'inverse_kinematics'. 