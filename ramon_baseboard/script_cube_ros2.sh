xhost +local:root; docker run --mount type=bind,source=/home/fdominguez/ros2_ws,target=/home/ros2_ws -d -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $PWD/shared:/mnt/host:rw ebenolson/cubemx:latest sh -c '/usr/local/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX' --user "$(id -nu):$(id -ng)"
