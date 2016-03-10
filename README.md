Matlab MEX interface to the wholeBodyModel C++ interface, implemented to be able to create forward dynamics on matlab for prototyping purposes.

## Controllers simulations
In [matlab-src/torqueBalancing_matlab](matlab-src/torqueBalancing_matlab/) the user can find two different
simulations of whole-body controller implemented using the mex-wholebodymodel interface. One uses the Stack of task approach while 
the other is a Joint Space controller. 
For more information please check the relative [README_SoT](matlab-src/torqueBalancing_matlab/StackOfTask_balancing/README) and
[README_Jc](matlab-src/torqueBalancing_matlab/JointSpace_balancing/README)

## Tests
To verify regression in the code when you modify the code, some
regression tests have been implemented in this repository.

### Run tests
To run the tests on this repository, just compile it and from the build directory launch
the `ctest` command. If you want to print out the verbose output of the tests, launch `ctest -VV`.

### Add new tests
To add new tests, just write a script that runs your test and raise and exception
if the test fails (for example using the `assert` command or the [tests/WBAssertEqual.m](tests/wBAAssertEqual.m) function). 
Then modify the [tests/WBMTests.m](tests/WBMTests.m) file to call your testing script.
