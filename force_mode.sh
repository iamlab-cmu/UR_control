#!/bin/bash

rostopic pub /ur_hardware_interface/script_command std_msgs/String --file="./force_mode.yaml"