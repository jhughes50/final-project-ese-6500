## ESE 6500 Final Project
By: Josh Caswell, Jason Hughes, Zac Ravichandran, Luying Zhang

Using factor graphs for sensor fusion...

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
