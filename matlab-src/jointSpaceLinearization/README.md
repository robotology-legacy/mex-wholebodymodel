
## Joint space linearization

In this folder, there are 2 different script for linearizing the joint space dynamics, when the robot is balancing on 1 foot. 
The user can choose which foot is fixed on the ground, the robot joint configurations and the CoM and postural gains.
The output are the eigenvalues of the state matrix of the linearized system.
By default, the system to be linearized is with the new 'Stack of Task' control, i.e. the control loop is closed with
position-angular terms at the momentum level and the postural task has been modified.
