#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import argparse
from enc_query.srv import enc_query_srv



def enc_query_client(layers, longitude, latitude, azimuth, distance, view_angle):
    print("initializing client...")
    rospy.init_node("enc_query_client")
    print("waiting for service")
    rospy.wait_for_service('enc_query_node')
    print("Done waiting for service...")
    try:
        enc_query = rospy.ServiceProxy('enc_query_node', enc_query_srv)
        # rosparam.set_param('enc_root', '/home/thomas/Downloads/ENC_ROOT')
        # rosparam.set_param('catalog_location', 'https://www.charts.noaa.gov/ENCs/NH_ENCProdCat_19115.xml')
        resp1 = enc_query(layers, longitude, latitude, azimuth, distance, view_angle)
        print("Got Response")
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    layers = ['all']
    longitude = -70.863 
    latitude = 43.123
    bearing = 0
    distance = 1000
    view_angle = 40
    print("Requesting query...")
    features = enc_query_client(layers, longitude, latitude, bearing, distance, view_angle)
    print('Response Received...')
    print(features)
    
