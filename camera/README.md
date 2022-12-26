# Single camera setup

## **Enable the camera port**

First you need to tell to Ubuntu to accept the Raspberry Pi CameraV1. This is done by installing raspi-config. Follow these steps to do so:

```bash
sudo apt-get update && upgrade

sudo apt install curl

sudo curl -L --output /usr/bin/rpi-update https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update && sudo chmod +x /usr/bin/rpi-update

sudo rpi-update
reboot
sudo apt install raspi-config
```

After installing raspi-config, you can enter the Raspberry Pi Software Configuration Tool with the command

```bash
sudo raspi-config
```

You will be greeted by the following screen:
![Raspi_config](/img/RaspiConfigScreen.png)

To activate the legacy camera support, select option `3 Interface Options` with the arrow keys and press enter.
![InterfaceOptions](/img/InterfaceOption.png)

Press enter again on `1 Legacy Camera`.

![LegacyCamera](/img/LegacyCamera.png)

Press enter on `<Yes>`. If you get the following screen, you have successfully enabled the camera. Hit enter again.
![CameraEnabled](/img//CameraEnabled.png)

Press the right arrow key twice to select `<Finish>` and hit enter. Reboot the Pi to activate the changes.

## **Camera connection**

Put the cable in the camera. The blue patches should always be the closest to the black part of the connector. 

<img src="/img/CameraBack.jpg" width="50%" height="50%">
<img src="/img/CameraFront.jpg" width="50%" height="50%">

Connect the other side of the cable with the Raspberry Pi. Use the connector in the middle, not the one on the side cause that one is for a display.

It should look like this

<img src="/img/PiBack.jpg" width="50%" height="50%">
<img src="/img/PiFront.jpg" width="50%" height="50%">

## **Camera detection**

After connecting the camera, you have to make sure that Ubuntu detects it.

First open the firmware config file with the following command

```bash
sudo nano /boot/firmware/config.txt
```

At the bottom of the file, add the next lines of code:

```text
start_x=1
gpu_mem=128
```

Next you need to install the Video4Linux library. Run the following command

```bash
sudo apt-get install v4l-utils
```

After the installation, you should be able to run the following command 

```bash
v4l2-ctl --list-devices
```

It wil give a response something like this

![CameraDetection](/img/CameraDetection.png)

The bottom line is the path where the camera can be found. This is needed in the code for the camera.

## **Live video feed**

An option to implement the camera in this project is live video feed in a web browser. The down side is there must be a wifi network or the device that will be used has to be capable of creating a hotspot.

Start by installing python 3 and the library's needed to use the camera and to stream

```bash
sudo apt install python3
sudo apt install libopencv-dev python3-opencv flask
```

Create a new directory

```bash
mkdir video-server
```

Copy the files from the camera/server directory of this project.
By running `python3 server.py` the video feed will be streamed on the localhost and on the ip address in the network the pi is connected with.

## **Colordetection**

An other option to implement the camera is by publishing a ROS message with the color code of the center pixel of the frame.

In the src directory of the ROS workspace, create a new python package.

```bash
ros2 pkg create --build-type ament_python colordetection
```

The src directory should look like this

![Tree_src_camera_package](/img/tree_src_camera_package.png)

Go to src/colordetection/colordetection and copy the files from camera/colordetect of this project.
In the ROS workspace, run `colcon build`. After that, there should be a build, install and log directory next to the self made src directory.

In the src/colordetection, open the setup.py file. At the bottom: there is a block of code like this:

```text
entry_points={
        'console_scripts': [
        ],
```

It"s important to add the entry points of out nodes. So change to code to something like this:

```text
entry_points={
        'console_scripts': [
                'colordetection = colordetection.colordetection:main',
                'colorsub = colordetection.colorsubscriber:main'
        ],
```

The syntax is as follows:

```text
'name = directory.file:point'

name = the name used in the ros run command to run the node
directory = directory where the programs are located
file = filename of the program 
point = location in the program where it needs to start (in most cases it will be main)
```

To run the nodes in the ros workspace, source the install/setup file. According to the used shell, change the extension of the setup file. Possible files are: setup.bash, setup.zsh, setup.ps1 and setup.sh

Then by running `ros2 run colordetection colordetection` the publisher wil run.
By running `ros2 run colordetection colorsub` the subscriber wil run.
