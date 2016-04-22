## Stack of task balancing

iCub balancing control implemented using a 'momentum-based' controller. 

### Start the simulation

-open 'integrateForwardDynamics_SoT.m'

-at the top of the script, the user can define different types of simulations, for example 1 foot or 2 feet balancing, activate QP solver, 
 activate the CoM movements, change the robot configuration. at the bottom of the script it is possible to modify the simulation time and the 
 integration time step.

### Modify gains and CoM movements

-all these parameters can be modified opening the Matlab function 'gainsAndConstraints_SoT.m'

### Visualize more graphics

-open 'forwardDynamics_SoT.m'

-at the bottom of the script, add to the structure 'visual_param' the parameters you need to visualize

-open 'visualizer_SoT.m'

-add a 'figure' and 'plot' the parameters you seleced previously 