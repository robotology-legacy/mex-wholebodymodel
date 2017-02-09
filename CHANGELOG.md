# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/) 
and this project does not adheres to [Semantic Versioning](http://semver.org/).

## New release: 
### [0.2](https://github.com/robotology/mex-wholebodymodel/compare/recovery...master) - 2017-01-25

#### Added
- New version of the mexWholeBodyModel (mex-WBM) subroutine for Matlab. Thanks to [@Ganimed](https://github.com/Ganimed), a 
  complete refactory of the toolbox has been done, to speedup and stabilize the call of the kinematic/dynamic functions.

#### Renamed Functions
- **_Wrapper-Function Names:_**
  - wbm_djdq --> wbm_dJdq
  - wbm_getStates:
    - Changed order of the output arguments from [qj, xTb, qjDot, vb] to [vqT_b, q_j, v_b, dq_j], where
      qj = q_j, xTb = vqT_b, qjDot = dq_j and vb = v_b.
  - wbm_getWorldFrameFromFixedLink -->	wbm_getWorldFrameFromFixLnk
    - Changed output order:  [world_R_base, world_p_base] --> [wf_p_b, wf_R_b]
- **_Utility-Function Names:_**
  - quaternionDerivative --> dquat
    - New input arguments: quat, omega, gain (optional), chktol (optional).
  - frame2posrot --> frame2posRotm
  - fromSkew2Vector --> skewm2vec
  - parametrization --> rotm2eulAngVelTF
    - New input arguments: rotm, sequence ('ZYX' or 'ZYZ', default 'ZYX'). 
  - quaternion2dcm --> quat2rotm
  - skew --> skewm
  - whatname --> getJointAnnotationICub

#### New Utility-Functions:
- A lot of utility-functions were added to the utilities-directory in order to be independent from other
  third party libriaries, i.e. transformation functions. For further details have a look into the [+utilities](/mex-wholebodymodel/matlab/utilities/+WBM/+utilities/) directory.
  
#### Renamed Key-Names:
- generalised-forces --> generalized-forces
- model-initialise --> model-initialize
- model-initialise-urdf --> model-initialize-urdf
  
#### New Key-Names:
- coriolis-forces
- get-base-state
- gravity-forces
- inverse-dynamics
- transformation-matrix

#### Deleted Functions (outdated):
- fromMesh2sortedVector
- resizeData
- sortedVector2mesh
- visualizeForwardDynamics

#### Future planned renamings:
- Change the variable _q_j_ to _s_j_, where _s_ denotes the _"shape"_ of the robot based on its current
  joint positions _j_.
- The same will be done for the variables _dq_j_ and _ddq_j_, such that _dq_j_ --> _ds_j_ and _ddq_j_ --> _dds_j_.

#### Controllers and WBM-Class
- The controllers in the [controllers](/controllers) directory are adapted to the new wrapper and utility-functions.
- The first [beta release](/mex-wholebodymodel/matlab/wrappers/+WBM) (at the moment undocumented) of the object-oriented version of the Whole Body Model for YARP-based robots is now available for testing purposes.

#### New visualizer
- New toolbox for visualizing the robot simulation based on the iDyntree library. For further information, see [README](https://github.com/robotology/mex-wholebodymodel/blob/master/README.md).

### Using the new functions:
- To avoid conflicts with other libraries, the new utility-functions and also the Matlab-classes are put into own namespaces.
- For applying these new functions the namespace must be specified at first at top of the Matlab procedure or function with
  `import WBM.utilities.<function_name>`, or `import WBM.utilities.*;` to load **all** utility functions, or with
  a direct call of the function with the full namespace path `WBM.utilities.<function_name>`.

## Previous (alpha) version:
### [0.1.1]
[0.1.1]: https://github.com/robotology/mex-wholebodymodel/compare/v0.1...HEAD
#### Added
- All the functions take in input the complete state, including the base pose with respect to the world, 
  see [Issue 27](https://github.com/robotology/mex-wholebodymodel/issues/27).
- Please note that a bug occured before [Pull Request](https://github.com/robotology/mex-wholebodymodel/pull/57) that inverted
  the rotation passed (when passing the full state) to the `wbm_forwardKinematics`,`wbm_massMatrix`, `wbm_jacobian`, `wbm_centrodialMomentum`, `wbm_djdq`. 

#### Removed
- Removed `wbm_setWorldLink`, its behavior can be emulated with the `wbm_getWorldFrameFromFixedLink` and the new interfaces that take the complete
  state as input. 
  See [Issue 43](https://github.com/robotology/mex-wholebodymodel/issues/43) and [Pull Request 52](https://github.com/robotology/mex-wholebodymodel/pull/52).

### [0.1] - 2014-11-15
[0.1]: https://github.com/robotology/mex-wholebodymodel/compare/9fe87c...v0.1
#### Added
- First version of mex-wholebodymodel, compatible with wholebodyinterface v0.1. 
