#!/usr/bin/env python

from __future__ import print_function

import rospy
import math


from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPolygon
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPath, GeoPoseStamped
from marine_msgs.msg import NavEulerStamped, NavEuler
from std_msgs.msg import ColorRGBA
from enc_query.srv import enc_query
import project11

features = []

geoVizItem_pub = None
geoVizItem = GeoVizItem()
class FOVQuery():
    geoVizItem = None
    fov = None
    def __init__(self):
        global geoVizItem 
        fov = calculateFOV()
        print("Thomas")
        geoVizItem.id = "FOV" 
        fovVizPoly = geoPathToGeoVizPoly(fov)
        fovVizPoly.edge_color = ColorRGBA(1,0,0,1)
        geoVizItem.polygons.append(fovVizPoly)
       

    def publish(self):
        print(geoVizItem.id)    
       

    def iterate(self):
        self.fov = calculateFOV()

def calculateFOV():
    # global fov
    fov = GeoPath()
    """calculate the field of view for the class attributes

    Returns:
        ogr.Geometry : field of view
    """
    #          .
    #    .     |     .
    #     \    |10m /
    #      \   |   /
    # 10m   \  |  /  10m
    #        \ | /
    #         \|/ View angle = 40
    #          o boat
    #Define geometry
    # fovCoords = ogr.Geometry(ogr.wkbLinearRing)
    # TODO: Make distance and viewangle ROSPARAM
    distance = 100
    view_angle = 40
    long = asv_position.position.longitude
    lat = asv_position.position.latitude

    #Angles for direction of 
    lDeg = asv_heading - (view_angle/2)
    rDeg = asv_heading + (view_angle/2)

    #Calculate points of geometry
    lPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(lDeg),distance)
    mPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(asv_heading),distance)
    rPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(rDeg),distance)

    #Add points to geometry
    fov.poses.append(makePose(long, lat))
    fov.poses.append(makePose(math.degrees(lPoint[0]),math.degrees(lPoint[1])))
    fov.poses.append(makePose(math.degrees(mPoint[0]),math.degrees(mPoint[1])))
    fov.poses.append(makePose(math.degrees(rPoint[0]),math.degrees(rPoint[1])))
    return fov
def publish(data):
        testPoint = FOVQuery()
        testPoint.publish()

def makePose(long,lat):
    poseStamped = GeoPoseStamped()
    poseStamped.pose.position.latitude = lat
    poseStamped.pose.position.longitude = long
    return poseStamped

def geoPathToGeoVizPoly(self):
    fovPoly = GeoVizPolygon()
    for geoPostStamped in self.poses:
        fovPoly.outer.points.append(geoPostStamped.pose.position)
    fovPoly.fill_color = ColorRGBA(1,0,0,1)
    return fovPoly
        
def positionCallback(data):
    global asv_position
    asv_position = data

def headingCallback(data):
    global asv_heading
    asv_heading = data.orientation.heading

def iterate(data):
    global geoVizItem
    geoVizItem = GeoVizItem()
    geoVizItem.id = "FOV" 
    fovGeoViz = geoPathToGeoVizPoly(calculateFOV())
    if len(geoVizItem.polygons) == 0:
        geoVizItem.polygons.append(fovGeoViz)
    else:
        geoVizItem.polygons[0] = fovGeoViz
    geoVizItem_pub.publish(geoVizItem)
        
        
   
if __name__ == '__main__':
    rospy.init_node('fov_query')
    position_sub = rospy.Subscriber('/position', GeoPointStamped, positionCallback)
    heading_sub = rospy.Subscriber('/heading', NavEulerStamped, headingCallback)

    geoVizItem_pub = rospy.Publisher('/project11/display',GeoVizItem,queue_size = 10)
    
    rospy.Timer(rospy.Duration.from_sec(0.01),iterate)

    rospy.spin()


    
