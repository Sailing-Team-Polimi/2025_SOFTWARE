#!/bin/bash

set -e  # Interrompe lo script in caso di errore

echo "🔄 Update iniziale"
sudo apt update && sudo apt -y upgrade

echo "🔐 Installo SSH, curl, git, etc..."
sudo apt install -y openssh-client openssh-server wget nano curl git

echo "📦 Aggiungo repository ROS 2 Jazzy"
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "🔄 Update & Upgrade post-repo"
sudo apt update && sudo apt -y upgrade

echo "🧰 Installo dev tools ROS"
sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools

echo "🐢 Installo ROS 2 Jazzy Desktop"
sudo apt install -y ros-jazzy-desktop

echo "📦 Extra ROS packages"
sudo apt install -y \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rviz-imu-plugin \
  ros-jazzy-tf-transformations \
  ros-jazzy-rosbridge-server \
  ros-jazzy-robot-localization



echo "🔧 Configuro Git"
sudo apt install -y git-all
git config --global user.email "tommaso.scudeletti@mail.polimi.it"
git config --global user.name "ScudeT"

echo "📁 Clono la repo"
cd ~
git clone https://github.com/Sailing-Team-Polimi/2025_SOFTWARE.git

echo "🧠 Configuro il .bashrc"
echo "export ROS_DISTRO=jazzy" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> ~/.bashrc
echo 'if [ -f ~/2025_SOFTWARE/ros2_ws/install/local_setup.bash ]; then' >> ~/.bashrc
echo '  source ~/2025_SOFTWARE/ros2_ws/install/local_setup.bash' >> ~/.bashrc
echo 'fi' >> ~/.bashrc
echo "cd ~/2025_SOFTWARE/ros2_ws" >> ~/.bashrc

echo "⚙️ Interfacce Raspberry (UART, GPIO, I2C, GPS)"
sudo apt install -y raspi-config
sudo adduser $USER dialout
sudo apt install -y gpiod libgpiod-dev
sudo apt install -y i2c-tools libi2c-dev
sudo adduser $USER i2c
sudo apt install -y gpsd gpsd-clients python3-gps
sudo apt install -y tree minicom

echo "✅ SETUP COMPLETATO! Riavvia o esegui:"
echo "source ~/.bashrc"
