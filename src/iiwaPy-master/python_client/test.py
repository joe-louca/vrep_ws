#!/usr/bin/env python

import rospy

def main():
    print(test)

if __name__ == '__main__':
    global test
    test = rospy.get_param('joint_angles')
    main()
