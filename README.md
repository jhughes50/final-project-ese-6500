## Tanqueray: ESE 6500 Final Project
By: Jason Hughes, Josh Caswell

Tanqueray is the first ever GLINS system: GPS-LiDAR-INS system. It offers extremely accurate localization without drift and at high rate.

In this project, we use factor-graphs to fuse GPS, LiDAR odometry and IMU measurements to get fast, accurate and efficient state estimates. We use Faster-LIO as a LiDAR odometry source, this is added to our factor graphs in GTSAM as a between factor so we do not suffer from traditional LiDAR odometry drift. The GPS is used as the main prior in the factor graph and is our coordinate frame. This allows us to correct GPS noise with LiDAR odometry without drift. GPS and LiDAR odometry update at slow rates, at 4 Hz and 10 Hz respectively. To get faster state estimates we use IMU preintegration to interpolate between GPS updates. This allows to get state updates at more than 100 Hz while using little compute.

### Running
Both Tanqueray and the visualizer can be built as ros packages in your ros workspace.
You can build both Tanqueray and the visualizer with `catkin build`.
Run Tanqueray with:
```
roslaunch tanqueray tanqueray.launch
```
You can set the argument: `use_sim_time:=true` if you are using a bag file or simulated sensor.

### Visualizer
We provide a flask visualization app that allows us to look at robots location in real time on a satellite map. 
To run this locally, just run
```
roslaunch visualzier viz.launch
```
To run this from a remote compute or robot, change the `ip_address` (and possibly) `port` parameters from `config/map.yaml`
or override them directly in the CLI:
```
roslaunch visualizer viz.launch ip:=<ip-address> port:=<port>
```

### Building and Running Unit Tests
We use GTest to run unit tests. You can build the tests with 
``` 
cmake -S . -B build
cmake --build build
```
and run with:
```
cd build 
ctest
```

