## docker ##

docker stop $(docker ps -q)         # stop all running containers

docker rm $(docker ps -aq)          # remove all containers
docker rmi $(docker images -q)      # remove all images
docker system prune -a --volumes    # remove everything

docker ps                           # list active containers
docker ps -a                        # list all containers
docker images -a                    # list images
docker volume ls                    # list volumes
docker network ls                   # list networks
docker start ros_jazzy              # start container ros_jazzy
docker exec -it ros_jazzy /entrypoint.sh  # enter container

docker update --restart=always ros-persefone # makes the container start on boot

## groups and users ##
getent group i2c # get the numeric group ID (GID) of the i2c group


## devices ## 
udevadm info -a -p $(udevadm info -q path -n /dev/ttyAMA0) # check for mathcing tag for device (ex /dev/serial0) for udev rules
# more info on UDEV RULES at https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Udev%20Rules.html

sudo dmesg | grep tty   # check serial ports

sudo fuser /dev/ttyACM0 # check processes running on port 
sudo kill xxxx          # fill process with PID xxxx 

sudo minicom -b 115200 -H -o  -D /dev/ttyAMA0     # show terminal output
sudo minicom -b 9600 -o  -D /dev/ttyACM0

## gpio ##
# gpio commands -> https://lloydrochester.com/post/hardware/libgpiod-intro-rpi/

gpioinfo # get info on all gpio pins used
gpiodetect


## i2c ##

i2cdetect -y 1          # check i2c device port number for i2c-1: 
i2cset -y 1 0x08 0x01   # send 1 byte through i2c at port 0x08


## gps ##
sudo apt-get install gpsd gpsd-clients     # install gps tools
gpsmon /dev/ttyACM0                     # gps terminal output

ros2 launch ublox_gps ublox_gps_node-launch.py
# sudo systemctl enable gpsd.socket # enable the socket (necessary only the first time)
# sudo systemctl start gpsd.socket # start the socket
# sudo systemctl status gpsd.socket # check gpds
# cgps -s                             # verify incoming data from gps: 


## memory ##
baobab      # verify memory usage 

## cameras
rpicam-hello --camera 0 --qt-preview -t 0 # get preview feed of the camera through ssh 

## ownership and permissions ##
sudo chown -R $(id -u):$(id -g) . # change ownership of current directory to current user 

## ROS - when in docker container ros-jazzy ##
# create cpp package in ros2
ros2 pkg create --build-type ament_cmake --license Apache-2.0 [name] --dependencies rclcpp [other dependencies]
ros2 pkg create --build-type ament_cmake --license Apache-2.0 manta_control --dependencies rclcpp std_msgs sensor_msgs geometry_msgs

# create py package in ros2
ros2 pkg create --build-type ament_python --license Apache-2.0 [name] --dependencies rclpy [other dependencies]
ros2 pkg create --build-type ament_python --license Apache-2.0 rpi-stereo --dependencies rclpy std_msgs sensor_msgs geometry_msgs stereo_msgs

# start a bag reading all topics 
ros2 bag record -o [name] -a

# start a bag reading only some topics
ros2 bag record -o [name] [topic1] [topici]
ros2 bag record -o heading_movements /imu/data /imu/mag_raw

# publish static tf
ros2 run tf2_ros static_transform_publisher [x y z y p r] [parent_link] [child_link]
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

# check arduino 
docker start ros_jazzy 
docker exec -it ros_jazzy /entrypoint.sh  # enter container
ros2 run arduino_comm ardu_i2c_wave_node
ros2 topic pub /ardu_data std_msgs/msg/Float32MultiArray '{data: [1.0, 0.0, 0.0, 0.0, 0.0]}'
ros2 topic pub /ardu_data std_msgs/msg/Float32MultiArray '{data: [1.0, 45.0, 45.0, 0.0, 0.0]}'

ros2 topic pub -r 2 /toBrunilde sail_msgs/msg/SerialMsg \
  "{id: 10,
    payload:{layout: {dim: [], data_offset: 0}, data: [0, 69]}}"

scp -r persefone@192.168.125.159:/home/persefone/2025_SOFTWARE/ros2_ws/rosbags C:\Users\tommy\Desktop\registrazioni