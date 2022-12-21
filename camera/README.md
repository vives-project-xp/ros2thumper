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

![CameraBack](/img/CameraBack.jpg)
![CameraFront](/img/CameraFront.jpg)

Connect the other side of the cable with the Raspberry Pi. Use the connector in the middle, not the one on the side cause that one is for a display.

It should look like this

![PiFront](/img/PiFront.jpg)
![PiBack](/img/PiBack.jpg)

## **Camera detection**

After connecting the camera, you have to make sure that Ubuntu detects it.

First open the firmware config file with the following command
```bash
sudo gedit /boot/firmware/config.txt
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

## **Testing the camera**

It is possible to run a simple python script to take a picture. This will be used to see if the camera is working.

Start by installing python 3 and the library's needed to use the camera
```bash
sudo apt install python3
sudo apt install libopencv-dev python3-opencv
```

Make a file in a 



