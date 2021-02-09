#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import argparse
from enc_query.srv import enc_query_srv
parser = argparse.ArgumentParser()

# group = parser.add_mutually_exclusive_group()
# group.add_argument('-p', '--path',
#                     action='store', 
#                     help= 'ENC file input')
# group.add_argument('-d','--directory',
#                     action = 'store',
#                     help = 'Directory of ENC files to survey.')
parser.add_argument('-l','--layers',
                    nargs='+',
                    type = str,
                    help = 'Layers to query. "all" for all layers. --layer boylat wrecks') 
parser.add_argument('-long','--longitude',
                    action='store',
                    help = 'longitude')
parser.add_argument('-lat','--latitude',
                    action='store',
                    help = 'longitude')

parser.add_argument('-a','--azimuth',
                    action='store',
                    help = 'Degrees from due north')
parser.add_argument('-dist','--distance',
                    action='store',
                    help = 'distance in meters')
parser.add_argument('-v','--view_angle',
                    action='store',
                    help = 'View angle of FOV')



def enc_query_client(layers, longitude, latitude, azimuth, distance, view_angle):
    print("initializing client...")
    rospy.init_node("enc_query_client")
    print("waiting for service")
    rospy.wait_for_service('enc_query_node')
    print("Done waiting for service...")
    try:
        enc_query = rospy.ServiceProxy('enc_query_node', enc_query_srv)
        resp1 = enc_query(layers, longitude, latitude, azimuth, distance, view_angle)
        print("Got Response")
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    
    args = parser.parse_args()
    layers = args.layers
    longitude = float(args.longitude)
    latitude = float(args.latitude)
    azimuth = float(args.azimuth)
    distance = int(args.distance)
    view_angle = int(args.view_angle)
    print("Requesting query...")
    features = enc_query_client(layers, longitude, latitude, azimuth, distance, view_angle)
    print('Response Received...')
    print(features)
    
    # # args = parser.parse_args()
    # # path = args.path
    # directory = args.directory
    # layers = args.layers
    # longitude = args.longitude
    # latitude = args.latitude
    # azimuth = args.azimuth
    # distance = args.distance
    # view_angle = args.view_angle
    # print("Requesting query...")
    # features = enc_query_client(path, directory, layers, longitude, latitude, azimuth, distance, view_angle)
    # for feature in features:
    #     print(feature.name, feature.fid)
