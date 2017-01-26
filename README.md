
Matlab MEX interface to the wholeBodyModel C++ interface, implemented to be able to create forward dynamics on matlab for prototyping purposes.

## Installation
It is recommended to install the `mex-wholebodymodel` throught the [`codyco-superbuild`](https://github.com/robotology/codyco-superbuild/).
Once you installed the `codyco-superbuild`, the `mex-wholebodymodel` files should have been installed in
`${CODYCO_SUPERBUILD_ROOT}/build/install/mex` and its subdirectories (for the `mex-wholebodymodel`, this directories are `mexwbi-wrappers` and `mexwbi-utilities`).
To execute scripts that use `mex-wholebodymodel`, make sure that this directories are part of your [MATLAB search path](http://www.mathworks.com/help/matlab/ref/path.html). 
To this purpose, run only once the script `startup_mexWholeBodyModel.m` in `${CODYCO_SUPERBUILD_ROOT}/build/install/mex`. 
This should be enough to premanently add the required directories to your MATLAB path.

## Examples 

### Rigid Body Dynamics
An example on how to use mex-wholebodymodel to compute the dynamics quantities of 
a rigid body is available at [examples/rigidBodyDynamics.m](examples/rigidBodyDynamics.m).

## Controllers
In branch [WBMToolbox-controllers](https://github.com/robotology/mex-wholebodymodel/tree/WBMToolbox-controllers) the user can find different whole-body controllers implemented using the mex-wholebodymodel interface. The "official" version is
in the folder [controllers/torqueBalancing](https://github.com/robotology/mex-wholebodymodel/tree/WBMToolbox-controllers/controllers/torqueBalancing), but also other balancing controllers are available. 
Matlab tools for visualization, inverse kinematics and so on are available in [controllers/tools](https://github.com/robotology/mex-wholebodymodel/tree/WBMToolbox-controllers/controllers/tools) folder.
For more informations about the balancing controller check the relative [README](https://github.com/robotology/mex-wholebodymodel/tree/WBMToolbox-controllers/controllers/torqueBalancing).

### Simulations
Please note that simulations of inverse dynamics results are performed using [iDyntree](https://github.com/robotology/iDyntree) visualizer. In order to be able to use it, enable the `IDYNTREE_USES_MATLAB` and `IDYNTREE_USES_IRRLICHT` options in CMake.
If irrlicht library is not installed in your computer, install it using `sudo apt install libirrlicht-dev`.

## Mex-wholebodymodel
This folder contains matlab utilities and wrappers and the toolbox C++ library.

## Tests
To verify regression in the code when the user modifies it, some regression tests have been implemented in this repository.

### Run tests
Tests use `ctest` infrastructure.
To run the tests on this repository, after compiling the project, just follows these steps:

- Move (`cd`) to the build directory
- Run the `ctest` command. If the project supports multiples configurations (e.g. Xcode), the user also has to specify the configuration with `-C`, thus for example `ctest -C Debug`.

To print out the verbose output of the tests, launch the `ctest` command by adding the `-VV` option.

### Add new tests
To add new tests, just write a script that runs the test and raise and exception
if the test fails (for example using the `assert` command or the [tests/WBAssertEqual.m](tests/WBMAssertEqual.m) function).
Then modify the [tests/WBMTests.m](tests/WBMTests.m) file to call the testing script.

