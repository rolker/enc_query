#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import argparse
from enc_query.srv import enc_query_srv
from geographic_msgs.msg import GeoPose


def enc_query_client(layers, points):
    print("initializing client...")
    rospy.init_node("enc_query_client")
    print("waiting for service")
    rospy.wait_for_service('enc_query_node')
    print("Done waiting for service...")
    try:
        enc_query = rospy.ServiceProxy('enc_query_node', enc_query_srv)
        # rosparam.set_param('enc_root', '/home/thomas/Downloads/ENC_ROOT')
        # rosparam.set_param('catalog_location', 'https://www.charts.noaa.gov/ENCs/NH_ENCProdCat_19115.xml')
        resp1 = enc_query(layers,points)
        print(resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    layers = ['boylat']
    p1 = GeoPose()
  

    p1.position.latitude = 43.123
    p1.position.longitude = -70.863
    p2 = GeoPose()
    p2.position.latitude = 43.12
    p2.position.longitude = -70.855
    p3 = GeoPose()
    p3.position.latitude = 43.12
    p3.position.longitude = -70.863
    p4 = GeoPose()
    p4.position.latitude = 43.123
    p4.position.longitude = -70.863
    points = [p1,p2,p3,p4]
    enc_query_client(layers,points)
    
    
