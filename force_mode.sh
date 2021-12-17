#!/bin/bash

rostopic pub /ur_driver/URScript std_msgs/String --file="./force_mode.yaml"