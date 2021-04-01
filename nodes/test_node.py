#!/usr/bin/env python

from __future__ import print_function

import rospy

from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import ColorRGBA
import project11

geoVizItem_pub = None
geoVizItem = GeoVizItem()
class TestPoint():
    geoVizItem = None
    def __init__(self):
        global geoVizItem 
        geoPoint = GeoPoint()
        geoPoint.longitude =  -70.75598
       
        geoPoint.latitude =  43.08110
        geoPoint.altitude = 1000
        pointList = GeoVizPointList()
        pointList.points = [geoPoint]
        pointList.color = ColorRGBA(0, 1, 0, 1)
        pointList.size = 100
        geoVizItem.point_groups = [pointList]
        geoVizItem.id = "THOMAS TEST"
     
        

    def publish(self):
        print(geoVizItem.id)    
        geoVizItem_pub.publish(geoVizItem)
        
def publish(data):
        testPoint = TestPoint()
        testPoint.publish()
   
if __name__ == '__main__':
    rospy.init_node('test')
    geoVizItem_pub = rospy.Publisher('/project11/display',GeoVizItem,queue_size = 10)
    
   
    rospy.Timer(rospy.Duration.from_sec(0.2),publish)

    rospy.spin()


    
