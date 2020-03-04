utnbadrone_base_patch
=======================

Description
-----------

This package is focused on including the local ROS libraries to the source code of the utnbadrone base board.

Limitation
-------------

Currently, this code was only tested on the SW4STM32 toolchain using the STM32CubeMX HAL.

Generate code
----------------

$ cd _target_sw4stm32_workspace_  
$ rosrun utnbadrone_base_patch make_libraries.py .  
**Never forget to change the project type to _cpp project_ in SW4STM32!!**  
