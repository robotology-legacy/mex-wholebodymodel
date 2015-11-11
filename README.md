Matlab MEX interface to the wholeBodyModel C++ interface, implemented
to be able to create forward dynamics on matlab for prototyping purposes.

## Controller simulation
In the [matlab-src/mex-wholebodymodel_balancing](matlab-src/mex-wholebodymodel_balancing/) and
[matlab-src/TorqueBalancing_js](matlab-src/TorqueBalancing_js/) directories you can find two different
simulations of a whole-body controller implemented using the mex-wholebodymodel interface. 
For more information please check the relative [README](matlab-src/mex-wholebodymodel_balancing/README),
[README](matlab-src/TorqueBalancing_js/README).

## Tests
To verify regression in the code when you modify the code, some
regression tests have been implemented in this repository.

### Run tests
Tests use `ctest` infrastructure.
To run the tests on this repository, after you compiled the project, just following the following steps:

- Move (`cd`) to the build directory
- Run the `ctest` command. If your project supports multiples configurations (e.g. Xcode), you also have to specify the configuration with `-C`, thus for example `ctest -C Debug`.

If you want to print out the verbose output of the tests, launch the `ctest` command by adding the `-VV` option.

### Add new tests
To add new tests, just write a script that runs your test and raise and exception
if the test fails (for example using the `assert` command or the [tests/WBAssertEqual.m](tests/wBAAssertEqual.m) function). 
Then modify the [tests/WBMTests.m](tests/WBMTests.m) file to call your testing script.
