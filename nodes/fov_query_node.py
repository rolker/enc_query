#!/usr/bin/env python

from __future__ import print_function

import rospy
import math


from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPolygon, GeoVizPointList
from geographic_msgs.msg import GeoPose, GeoPointStamped, GeoPath, GeoPoint
from marine_msgs.msg import NavEulerStamped
from std_msgs.msg import ColorRGBA
from enc_query.srv import enc_query_srv
import project11


geoVizItem_pub = None
asv_position = GeoPointStamped()
asv_heading = 0

# fov = None

class FOVQuery():
    fovPath = None
    fovVizItem = None
    distance = 0
    view_angle = 0
    enc_query = None
    def __init__(self, distance, view_angle):
        self.distance = distance
        self.view_angle = view_angle

        # init path and add points 
        self.fovPath = GeoPath()
        self.fovPath.poses.append(GeoPose())
        self.fovPath.poses.append(GeoPose())
        self.fovPath.poses.append(GeoPose())
        self.fovPath.poses.append(GeoPose())

        #init VizItem and add polygon
        self.fovVizItem = GeoVizItem()
        self.fovVizItem.polygons.append(GeoPath())
        self.fovVizItem.id = "FOV" 

        self.featureViz = GeoVizItem()
        self.featureViz.point_groups = [None]*2
        

        
        try:
            rospy.wait_for_service('enc_query_node')
            self.enc_query = rospy.ServiceProxy('enc_query_node', enc_query_srv)
        
        except rospy.ServiceException as e:
            rospy.loginfo("ERROR IN SERVICE INITIALIZATION")

       
    def iterate(self,data):
    
        rospy.loginfo("Running")

        #update fov
        self.calculateFOV()
        #update VizItem
        self.fovVizItem.polygons[0] = self.geoPathToGeoVizPoly(self.fovPath)

        #send Query
        enc_ret = self.enc_query(["boylat"],self.fovPath.poses)
        inViewList = GeoVizPointList()
        inViewList.color = ColorRGBA(0,1,0,1)
        inViewList.size = 30
        notInViewList = GeoVizPointList()
        notInViewList.color = ColorRGBA(0,0,1,1)
        notInViewList.size = 30
       
        for enc_feat in enc_ret.featuresInView:
            long = enc_feat.longitude
            lat=enc_feat.latitude
            inFov = enc_feat.inFov
            feat = GeoPoint(lat,long,0)
            if inFov:
                inViewList.points.append(feat)
            else:
                notInViewList.points.append(feat)

        self.featureViz.point_groups[0] = inViewList
        self.featureViz.point_groups[1] = notInViewList
            
        #pubish updates
        geoVizItem_pub.publish(self.fovVizItem)
        geoVizItem_pub.publish(self.featureViz)

    def calculateFOV(self):
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
        global asv_heading
        global asv_position
        
        long = asv_position.position.longitude
        lat = asv_position.position.latitude

        #Angles for direction of 
        lDeg = asv_heading - (self.view_angle/2)
        rDeg = asv_heading + (self.view_angle/2)

        #Calculate points of geometry
        lPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(lDeg),self.distance)
        mPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(asv_heading),self.distance)
        rPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(rDeg),self.distance)
     
        #update points to geometry
        pt0 = self.fovPath.poses[0]
        pt0.position.latitude = lat
        pt0.position.longitude = long
        pt1 = self.fovPath.poses[1]
        pt1.position.latitude = math.degrees(lPoint[1])
        pt1.position.longitude = math.degrees(lPoint[0])
        pt2 = self.fovPath.poses[2]
        pt2.position.latitude = math.degrees(mPoint[1])
        pt2.position.longitude = math.degrees(mPoint[0])
        pt3 = self.fovPath.poses[3]
        pt3.position.latitude = math.degrees(rPoint[1])
        pt3.position.longitude = math.degrees(rPoint[0])

    def geoPathToGeoVizPoly(self, path):
        poly = GeoVizPolygon()
        for geoPose in path.poses:
            poly.outer.points.append(geoPose.position)
        poly.fill_color = ColorRGBA(1,0,0,.2)
        return poly
        
def positionCallback(data):
    global asv_position
    asv_position = data

def headingCallback(data):
    global asv_heading
    asv_heading = data.orientation.heading
        
        
   
if __name__ == '__main__':
    rospy.init_node('fov_query')
    position_sub = rospy.Subscriber('/position', GeoPointStamped, positionCallback)
    heading_sub = rospy.Subscriber('/heading', NavEulerStamped, headingCallback)

    geoVizItem_pub = rospy.Publisher('/project11/display',GeoVizItem,queue_size = 10)
    fov_query = FOVQuery(100,45)
    rospy.Timer(rospy.Duration.from_sec(.1),fov_query.iterate)

    rospy.spin()


    
