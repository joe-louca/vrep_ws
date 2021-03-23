#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Float32MultiArray

def ft_delay(ft):
    # Stores ft readings in an array. Retrieves delayed readings. Removes old readings

    # Store frame & timestamp
    global delayed_fts
    delayed_fts.insert(0, ft)

    # Initialse variables
    tbl_len = len(delayed_fts)
    ft_retrieved = False
    ft = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ft_total = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    elapsed_time = rospy.get_time()

    # Number of readings to smooth
    N = 10
    
    for i in range(tbl_len): 
        if elapsed_time > delayed_fts[i][6] + latency:       # For each row, check if time < t + latency
            if tbl_len >= N:                                 # if enough values for smoothing
                for j in range(i,i+N):                           # Get average of N readings after delay point
                    for k in range(6):                      
                        ft_total[k] += delayed_fts[j][k]
                ft = [ft_total[0]/N, ft_total[1]/N, ft_total[2]/N, ft_total[3]/N, ft_total[4]/N, ft_total[5]/N]
                delayed_fts = delayed_fts[:(-tbl_len+i+N-1)]  # remove readings later than delay + N
                ft_retrieved = True
                break


            #ft = [delayed_fts[i][0], delayed_fts[i][1], delayed_fts[i][2], delayed_fts[i][3], delayed_fts[i][4], delayed_fts[i][5]]
            #delayed_fts = delayed_fts[:(-tbl_len+i-1)]
            #ft_retrieved = True
            #break
        
    return ft, ft_retrieved


def ft_callback(msg):
    global pub
    t = rospy.get_time()
    ft_reading = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], t]
    ft, ft_retrieved = ft_delay(ft_reading)

    if ft_retrieved:
        # publish ft reading
        m = Float32MultiArray()
        m.data= ft
        pub.publish(m)
        print(ft)
    
        rospy.set_param('ft_delay/fx', ft[0])
        rospy.set_param('ft_delay/fy', ft[1])
        rospy.set_param('ft_delay/fz', ft[2])
        rospy.set_param('ft_delay/tx', ft[3])
        rospy.set_param('ft_delay/ty', ft[4])
        rospy.set_param('ft_delay/tz', ft[5])
    

    
def main():   
    # Subscribes to /tf and saves position and rotation arrays to global config variables 
    rospy.init_node('ft_sub_node', anonymous=True)
    rospy.Subscriber('ft_sensor', Float32MultiArray, ft_callback, queue_size=1)
    global pub
    pub = rospy.Publisher('delayed_ft', Float32MultiArray, queue_size=1)
    rospy.spin()

  
if __name__ == '__main__':
    global delayed_fts
    global latency
    delayed_fts = []
    latency = rospy.get_param('latency')
    main()
