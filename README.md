# ros2thumper

## Setup

### Install ubuntu on the raspberry pi.

### Install ros2 on raspberry pi.

### Download ros2 packages.

### Setup systemd.

First make the script to launch all the nodes at once.
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