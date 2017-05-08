function STATE = robotState(chi,MODEL)
%ROBOTSTATE demux the current robot state to get the base pose, base
%           velocity, joint position and joint velocity.
%
% Format: STATE = ROBOTSTATE(chi,MODEL)
%
% Inputs:  - state vector chi [13+4*ndof x 1];
%          - MODEL is a structure defining the robot model;
%
% Output:  - STATE which is a structure containing the following variables:
%
% x_b            the cartesian position of the base (R^3)
% qt_b           the quaternion describing the orientation of the base (global parametrization of SO(3))
% qj             the joint positions (R^ndof)
% dx_b           the cartesian velocity of the base (R^3)
% w_omega_b      the velocity describing the orientation of the base (SO(3))
% dqj            the joint velocities (R^ndof)
% w_R_b          the base orientation as a rotation matrix (R^(3x3))
% nu             the state velocity (R^(ndof+6))
% basePose       the floating base pose (R^7)
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% CONFIG.demux.baseOrientationType sets the base orientation in stateDemux.m
% as positions + quaternions (1) or transformation matrix (0)
MODEL.demux.baseOrientationType = 1;

%% State demux
[basePose,qj,baseVelocity,dqj] = stateDemux(chi,MODEL);
% base position and orientation (in quaternions)
x_b                            = basePose(1:3,:);
qt_b                           = basePose(4:7,:);
% normalize quaternions to avoid numerical errors
if MODEL.CONFIG.normalize_quaternions
    qt_b                       = qt_b/norm(qt_b);
end
% base velocity; conversion of the base orientation into a rotation matrix;
% state velocity
dx_b                           = baseVelocity(1:3,:);
w_omega_b                      = baseVelocity(4:6,:);
[~,w_R_b]                      = frame2posRotm(basePose);
nu                             = [dx_b;w_omega_b;dqj];

%% Output structure
STATE.qj                       = qj;
STATE.dqj                      = dqj;
STATE.nu                       = nu;
STATE.basePose                 = basePose;
STATE.w_R_b                    = w_R_b;
STATE.x_b                      = x_b;
STATE.dx_b                     = dx_b;
STATE.qt_b                     = qt_b;
STATE.w_omega_b                = w_omega_b;

end

