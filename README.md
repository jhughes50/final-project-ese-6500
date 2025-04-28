## ESE 6500 Final Project
By: Josh Caswell, Jason Hughes, Zac Ravichandran, Luying Zhang

Using factor graphs for sensor fusion...


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

