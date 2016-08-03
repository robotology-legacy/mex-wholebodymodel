
Matlab MEX interface to the wholeBodyModel C++ interface, implemented to be able to create forward dynamics on matlab for prototyping purposes.

## Installation
It is recommended to install the `mex-wholebodymodel` throught the [`codyco-superbuild`](https://github.com/robotology/codyco-superbuild/).
Once you installed the `codyco-superbuild`, the `mex-wholebodymodel` files should have been installed in
`${CODYCO_SUPERBUILD_ROOT}/build/install/mex` and its subdirectories (for the `mex-wholebodymodel`, this directories are `mexwbi-wrappers` and `mexwbi-utilities`).
To execute scripts that use `mex-wholebodymodel`, make sure that this directories are part of your [MATLAB search path](http://www.mathworks.com/help/matlab/ref/path.html).

## Examples 

### Rigid Body Dynamics
An example on how to use mex-wholebodymodel to compute the dynamics quantities of 
a rigid body is available at [examples/rigidBodyDynamics.m](examples/rigidBodyDynamics.m).

## Matlab-src

### Controllers simulations
In [matlab-src/controllers](matlab-src/controllers/) the user can find different
simulations of whole-body controller implemented using the mex-wholebodymodel interface. The official controller version is
in the folder [matlab-src/controllers/torqueBalancing](matlab-src/controllers/torqueBalancing/), while in the folder
[matlab-src/controllers/experiments](matlab-src/controllers/experiments/) other balancing controllers are implemented. 
All the utility function for control are in [matlab-src/controllers/utilityMatlabFunctions](matlab-src/controllers/utilityMatlabFunctions/). For more informations about the balancing controller check the relative [README](matlab-src/controllers/torqueBalancing/).

## Mex-wholebodymodel
This folder contains all the matlab and C++ utilities used for tests and controllers.

## Tests
To verify regression in the code when the user modifies it, some
regression tests have been implemented in this repository.

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

