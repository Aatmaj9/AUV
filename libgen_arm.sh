sudo cp -r ros2_ws/src/interfaces ~/Arduino/libraries/micro_ros_arduino/extras/library_generation/extra_packages/

cd ~/Arduino/libraries/micro_ros_arduino
sed -i 's/4_8-2014q1/7-2017-q4-major/g' extras/library_generation/library_generation.sh
export ARDUINO_PATH=[~/.arduino15/packages/per1234]

docker run -it --rm -v $ARDUINO_PATH/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/toolchain -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p cortex_m3 -t serial

sudo chown -R $(whoami) ~/Arduino/libraries/micro_ros_arduino