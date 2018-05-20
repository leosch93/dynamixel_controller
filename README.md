# dynamixel_controller
This repository contains the controller of the aerial manipulator.

## Parameters
In the parameter file the different topics are specified.

## Documentation
The code is commented in a way such that the documentation can autonomously be generated using Doxygen:

1. Install doxygen:
```
$ sudo apt-get install doxygen doxygen-doc doxygen-gui graphviz
```
For futher information, see: https://wiki.ubuntuusers.de/Doxygen/

2. Generate the documentation (html and latex)
The doxygen configuration files are included in each package under the directory $(find package)/Doxygen/..._doxy

--> Go to the root directory of the package and start generating the documentation from the configuration file
```
$ cd <path to root of package>
$ doxygen Doxygen/..._doxy
```

### Example for the catkinpkg_framework package:
--> Go to package the root of package catkinpkg_framework
```
$ cd ~/catkin_ws/src/catkinpkg_framework
$ doxygen Doxygen/catkinpkg_framework_doxygen_config
```

This will generate the html and latex documentation.
