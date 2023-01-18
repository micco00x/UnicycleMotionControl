# Unicycle Motion Control
## CoppeliaSim Plugin
Compile the plugin and copy it in the CoppeliaSim folder.
```bash
mkdir build
cd build
cmake ..
make
cp libSimExtUnicycleMotionControl.so ${COPPELIASIM_ROOT_DIR}
```

## Matlab
Run matlab from the `matlab` directory and run UnicycleMotionControl.m
by either clicking on the Run button or typing from the command window
the following:
```
UnicycleMotionControl
```