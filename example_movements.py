#!/usr/bin/env python

import time
import signal
import sys
import rospy
import tf
import math
from ur_controller import URController

def exit_gracefully(signum, frame):
    # restore the original signal handler as otherwise evil things will happen
    # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
    signal.signal(signal.SIGINT, original_sigint)

    try:
        if raw_input("\nAre you sure you want to quit? (y/n)> ").lower().startswith('y'):
            sys.exit(1)

    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

    # restore the exit gracefully handler here    
    signal.signal(signal.SIGINT, exit_gracefully)

def run_main():

    ur_controller = URController()

    ur_controller.home_arm(10)

    ee_pose = ur_controller.get_ee_pose()

    new_ee_pose = [ee_pose[0], ee_pose[1]-0.01, ee_pos[2], ee_pos[3], ee_pos[4], ee_pos[5]]

    mbz_controller.move_ee(new_ee_pos, 5)


if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    run_main()
