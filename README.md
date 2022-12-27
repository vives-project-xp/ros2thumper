# ROS2 Thumper

## **Setup**

### **Install Ubuntu on the Raspberry Pi 4**

Install the Raspberry Pi Imager via [Raspberry Pi OS](https://www.raspberrypi.com/software/).

After installing, open the program and place an SD-card.
Select "Other general-purpose OS", then "Ubuntu" and choose Ubuntu Desktop 22.04.1 LTS.

![PiImager](/img/PiImager.png)

After waiting for the program to flash to your SD-card. Place the SD-Card into the slot on your Raspberry Pi 4.

### **Install ROS2 on Raspberry Pi 4**

1. **Set locale**

    Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

    ```bash
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```

2. **Setup sources**

    You will need to add the ROS 2 apt repository to your system. First, make sure that the Ubuntu Universe repository is enabled by checking the output of this command.

    ```bash
    apt-cache policy | grep universe
    ```

    This should output a line like the one below:

    ```bash
    500 http://us.archive.ubuntu.com/ubuntu jammy/universe amd64 Packages
        release v=22.04,o=Ubuntu,a=jammy,n=jammy,l=Ubuntu,c=universe,b=amd64
    ```

    If you don’t see an output line like the one above, then enable the Universe repository with these instructions.

    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    ```

    Now add the ROS 2 apt repository to your system. First authorize the GPG key with apt.

    ```bash
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```

    Then add the repository to your sources list.

    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3. **Install ROS2 packages**

    Update your apt repository caches after setting up the repositories.

    ```bash
    sudo apt update
    ```

    ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

    ```bash
    sudo apt upgrade
    ```

    The final step is to install ROS2

    ```bash
    sudo apt install ros-humble-desktop 
    ```

4. **Sourcing the setup script**

    Source the setup script so the terminal can use it.

    ```bash
    # Replace ".bash" with your shell if you're not using bash
    # Possible values are: setup.bash, setup.sh, setup.zsh
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    ```

5. **Install Colcon**

    You'll also have to install colcon to build the packages.

    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

(From https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### **Create workspace including ROS2 packages**

1. **Create a new directory**

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. **Download joy_linux**

    This will allow you to use a game controller to drive the thumper

    ```bash
    sudo apt install ros-humble-joy-linux
    ```

3. **Install thumper nodes**

    Copy the 3 thumper packages found in this directory to your workspace.
    When you have done this you can use:

    ```bash
    colcon build
    ```

    After building you can test this by doing:

    ```bash
    ros2 run thumper_drive thumper_drive_node
    ```

    This is an example with thumper_drive, just follow the same pattern for your other packages. 

## **Launch File with startup on boot**

### **Launch file**

First we make a new ROS2 package. Go into your ROS2 Workspace and pick a name for your package. We used start_robot but you can choose. You also need to delete the include and src directories. Replace these with a launch directory in which you will be adding your launch file.

```bash
cd ~/ros2_ws/src
ros2 pkg create start_robot
cd start_robot/
rm -rf include/
rm -rf src/
mkdir launch
touch launch/launch.py
```

After this we can start with the actual launch file itself. Underneath is an example for the launch file.

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
    )

    listener_node = Node(
        package="demo_nodes_py",
        executable="listener"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld
```

Inside each of the defined nodes there is package and executable. In our case an example would be:

```py
thumper_drive = Node(
    package="thumper_drive"
    executable="thumper_drive_node"
)
```

Our launch file can be found in our repository.
TODO

### **Install dependencies**

Because we use different packages than the one for our launch file we need to add some dependencies.

Open package.xml where you wrote the launch file.
To add the dependency, use an "exec_depend" tag for every package you want to launch with your launch file. I will once again use thumper_drive as an example:

```xml
<exec_depend>thumper_drive</exec_depend>
```

Go into the CMakeLists.txt of your package, and after find_package(ament_cmake REQUIRED), add:

```
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

Now go back into your workspace:

```bash
cd ..
```

And use build the package:

```bash
colcon build
```

After sourcing your workspace you can run the package and you will see that the nodes you wanted start running.

```bash
ros2 launch start_robot launch.py
```

### **Setup systemd**

Then we make the script to run the launch file.
We placed it in our user directory.

```bash
touch ~/startup.sh
```

```bash
#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/ros2thumper/ros2_ws/install/setup.bash
/opt/ros/humble/bin/ros2 launch start_robot launch.py
```

Make the script executable

```bash
chmod u+x ~/startup.sh
```

After this make the systemd service.

We first need to make a file and type the following code.

```bash
sudo nano /ect/systemd/system/ros2-thumper.service
```

```bash
[Unit]
Description=Run script on startup
After=default.target

[Service]
Type=simple
WorkingDirectory=/home/ros2thumper
RemainAfterExit=yes
ExecStart=/home/ros2thumper/startup.sh
TimeoutStartSec=0
User=ros2thumper
Environment="HOME=/home/ros2thumper"

[Install]
WantedBy=default.target
```

After this you will have to enable the service.

```bash
systemctl daemon-reload

systemctl enable ros2-thumper.service
```

To test if it works, you can do the following.

```bash
systemctl start ros2-thumper.service

systemctl status ros2-thumper.service
```

### Download GPIO library for Raspberry Pi

first you need to clone the library from github.

```bash
git clone https://github.com/joan2937/pigpio.git
```

Then you need to build the library.
You need to be in the repo you cloned from github to the following commands.

Need to check if this works on fresh install.

```bash
cmake -Bbuild .
cmake --build build --parallel
cmake --install build
sudo cmake --install build
```

To build the node you need the following command

```bash
colcon build --packages-select gpio_controller --cmake-args "-DCMAKE_INSTALL_PREFIX=/usr/local/"
```

After this you need to add the user to the dailout group and install the following.

```bash
sudo apt install rpi.gpio-common
```

## Physical Configuration

![Pinout for I²C and buck for Pi](/img/Pinout.png)

## 3D Drawings
The car that we use can be purchased [here](https://www.pololu.com/product/1563) and the drawings are all made for this version
### Frontbumper
![frontbumper](/img/3d_drawings/frontbumper.png)

This one has to be placed on the front of the car with m3 bolts and nuts. The 2 holes are for [neopixels](https://www.amazon.nl/AZDelivery-WS2812B-Neopixel-compatibel-Inclusief/dp/B07V1GGKHV/ref=sr_1_3?__mk_nl_NL=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=2BOZWYYQMQUCL&keywords=neopixel%2B12%2Bled%2Bring&qid=1666254029&qu=eyJxc2MiOiIwLjAwIiwicXNhIjoiMC4wMCIsInFzcCI6IjAuMDAifQ%3D%3D&sprefix=%2Caps%2C188&sr=8-3&th=1) and the 4 smaller holes are for the wires from the neopixel. There is a place for the camera on top, so make sure that the raspberry pi is not too far away.

### Backbumper
![frontbumper](/img/3d_drawings/backbumper.png)

This is the same as the frontbumper, but has to be installed on the back

### Battery holder and attachments
![batteryholder](/img/3d_drawings/batteryholder.png)

The batteryholder has to be placed on top of the car at the back. This is attached with m3 bolts and nuts. The battery is hold in place with 2 slide in doors. One of them has a hole so can let the wire come trough. The batteryholder also holds 2 buckconverters and these can be hold in place with a "clamp". There are also 4 of these to hold the doors and the battery in the holder 

![batteryholder door](/img/3d_drawings/batteryholder_schuif1.png)

![batteryholder door](/img/3d_drawings/batteryholder_schuif2.png)

![batteryholder clamp](/img/3d_drawings/batteryholder_klem.png)

![buckconverter clamp](/img/3d_drawings/buckconverter_klem.png)

### Pi holder and roof
![Pi holder](/img/3d_drawings/baseplate_pi.png)

The 4 smallest holes are to hold the pi on the baseplate. You need to use m2.5 nuts and bolts for this. The 4 outer holes are to hold the baseplate on top of the car. This needs to be close to the frontbumper so the camera can fit. This is also attached with m3 nuts and bolts. The other 4 holes are for the roof of the pi. 

![Pi holder](/img/3d_drawings/pi_roof.png)

The roof holds a neopixels that acts as a flashing light. The roof is attached with [spacers](https://www.allekabels.be/luidspreker-zelfbouw/237/1064473/afstandsbuis.html?gclid=CjwKCAiAzKqdBhAnEiwAePEjklzWyRlXLIAmq-iBt_muqQ_QlJY0f41R-OtYIHU7eHgD6yQ9KDLbIxoCVFYQAvD_BwE).

