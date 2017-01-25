# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/) 
and this project does not adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]
### Added
- All the functions take in input the complete state, including the base pose with respect to the world, 
  see [Issue 27](https://github.com/robotology/mex-wholebodymodel/issues/27).
- Please note that a bug occured before [Pull Request](https://github.com/robotology/mex-wholebodymodel/pull/57) that inverted
  the rotation passed (when passing the full state) to the `wbm_forwardKinematics`,`wbm_massMatrix`, `wbm_jacobian`, `wbm_centrodialMomentum`, `wbm_djdq`. 

### Removed
- Removed `wbm_setWorldLink`, its behavior can be emulated with the `wbm_getWorldFrameFromFixedLink` and the new interfaces that take the complete
  state as input. 
  See [Issue 43](https://github.com/robotology/mex-wholebodymodel/issues/43) and [Pull Request 52](https://github.com/robotology/mex-wholebodymodel/pull/52).

## [0.1] - 2014-11-15
### Added
- First version of mex-wholebodymodel, compatible with wholebodyinterface v0.1. 

## [0.2] - 2017-01-25
### Added
- New version of mex-wholebodymodel. A complete refactory of the toolbox has been done.

### Renamed
- The following WBM-wrappers have been renamed: 
- The output of the following functions changed order:

### Moved
- The controllers have been refactored and moved on a different branch, called `WBM-controllers`; 

[Unreleased]: https://github.com/robotology/mex-wholebodymodel/compare/v0.1...HEAD
[0.1]: https://github.com/robotology/mex-wholebodymodel/compare/9fe87c...v0.1
