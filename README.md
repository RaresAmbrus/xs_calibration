# xs_calibration


## xs_data_reader

Project for loading the raw data, including images, transforms and camera parameters.

### Library

 The file [helper_functions.h](xs_data_reader/include/helper_functions.h) contains the following methods:

* `parseDataFolder` - returns 4 vectors containing the 3x RGB images and 1x Depth images. The first RGB vector corresponds to `Xtion` images, the 2nd one to `XS1` images and the 3rd to `XS2` images. 
* `visualize_data` - debug method for viewing the images in opencv
* `initializeCamParamsFromFiles` - loads the camera parameters for the 3 cameras. Each set of camera parameters is a `double[6]` array containing (`k1` and `k2` are the 2nd and 4th order radial distortion coefficients):
  * fx
  * fy
  * cx
  * cy
  * k1
  * k2
* `loadPosesFromFile` - returns a vector of `Eigen::Matrix4f` transforms, 1st one from `Xtion -> XS1` and the 2nd one from `Xtion -> XS2`. 

This is compiled as a lib called `xs_data_reader`

### Executable

The executable [`xs_data_reader_main`](xs_data_reader/src/xs_data_reader_main.cpp) is meant to serve as an example on how to use the `xs_data_reader` library for loading the raw data. It also contains the method `testTransformsROS` which publishes the loaded camera poses so that they can be viewed in TF (with e.g. `rviz`).


