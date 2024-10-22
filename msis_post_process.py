#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from math import nan
import matplotlib.pyplot as plt
import rosbag

class MSIS_Post_Process:
    def __init__(self):
        with rosbag.Bag('/home/soslab/Desktop/Allen_harbor_10_18_2024/2024-10-18-12-27-02.bag') as bag:
        # with rosbag.Bag('/home/soslab/Desktop/NB_09_12_2024/2024-09-12-12-55-51.bag') as bag:
        
            for topic, msg, t in bag.read_messages(['/alpha_rise/odometry/filtered/local','/alpha_rise/echo', '/alpha_rise/pointcloud']):
                if topic == "/alpha_rise/echo":
                    angle = np.degrees(msg.angle)
                
                if topic == "/alpha_rise/odometry/filtered/local":
                    depth = msg.pose.pose.position.z

                elif topic == "/alpha_rise/pointcloud":
                    bin = []
                    intensity=[]
                    for index,point in enumerate(pc2.read_points(msg, skip_nans=True)):
                        i = point[-1]
                        bin.append(index * 50/1200)
                        intensity.append(i)


                    print(angle)
                    plt.figure()
                    plt.plot(bin, intensity)

                    if 90 > round(angle) > 40:
                        plt.title(f'{angle}, depth:{depth}' )
                        plt.annotate(xy=(10,np.mean(intensity)), text=f"Mean:{np.mean(intensity)}", color='r', fontsize=20)
                        plt.axhline(y = np.mean(intensity), color = 'r')

                        plt.annotate(xy=(10,1*np.std(intensity)), text=f"Threshold: {4*np.std(intensity)}", color='b', fontsize=20)
                        plt.axhline(y = 1*np.std(intensity), color = 'b')
                        plt.waitforbuttonpress(30)
                        plt.close()

if __name__ == "__main__":
    # rospy.init_node("post_process")
    MSIS_Post_Process()
    # rospy.spin()