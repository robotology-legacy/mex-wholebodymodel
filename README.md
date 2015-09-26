Project to create a mex C/C++ interface to the WBI WholeBodyModel components
in order to be able to create forward dynamics on matlab for prototyping purposes.

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
