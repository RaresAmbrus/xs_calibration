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
* `createCloudFromImages` - creates a point cloud from an RGB image, a depth image and camera parameters. 

This is compiled as a lib called `xs_data_reader`.

**NOTE** `XS1` is on the left, `Xtion` is in the middle and `XS2` is on the right (in terms of the actual physical layout).

### Executables

#### `xs_data_reader_main`

The executable [`xs_data_reader_main`](xs_data_reader/src/xs_data_reader_main.cpp) is meant to serve as an example on how to use the `xs_data_reader` library for loading the raw data. It also contains the method `testTransformsROS` which publishes the loaded camera poses so that they can be viewed in TF (with e.g. `rviz`).

#### `xs_data_publisher`

The executable [`xs_data_publisher`](xs_data_reader/src/xs_data_publisher.cpp) loads the data and publishes it in ros. It can be used for debugging how well the camera transforms and paramters fit, by visualizing in `rviz` the projected `Xtion` point cloud in the two camera images. The topics published are:
* `/xs1/image` and `/xs1/camera_info`
* `/xs2/image` and `/xs2/camera_info`
* `/xtion_cloud`
 
By subscribing to `/xtion_cloud` and adding an object of type [Camera](http://wiki.ros.org/rviz/DisplayTypes/Camera) in `rviz` subscribed to one of the xs image topics, you can visualize the projection of the point cloud in the camera image.

