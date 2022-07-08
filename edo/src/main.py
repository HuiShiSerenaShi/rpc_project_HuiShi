#!/usr/bin/env python3

import os
import sys
# Add current dir to path, otherwise importing other modules won't work
sys.path.insert(0, os.path.dirname(__file__))
from edo_move_group_interface import EdoMoveGroupInterface
import rospy

def main():
    try:
        # Initialize move group
        edo_move_group = EdoMoveGroupInterface()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()