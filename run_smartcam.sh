#!/bin/bash
export DISPLAY=:0
sudo chmod 777 /dev/ttyTHS1
#source /home/smartcam/archiconda3/bin/activate mindspore
#conda activate mindspore
cd /home/smartcam
python /home/smartcam/main.py
# sudo update-rc.d /etc/init.d/run_smartcam.sh defaults
