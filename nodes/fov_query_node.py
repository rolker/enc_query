#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import time
import project11
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPolygon, GeoVizPointList
from geographic_msgs.msg import GeoPose, GeoPointStamped, GeoPath, GeoPoint
from marine_msgs.msg import NavEulerStamped
from std_msgs.msg import ColorRGBA
from enc_query.srv import enc_query_srv

class FOVQuery:
    distance = 0
    fovPath = None
    fovVizItem = None
    enc_query = None

    def __init__(self,distance):
        self.distance = distance
        self.timelog = [0,0]

        # init path and add points 
        self.fovPath = GeoPath()
        for i in range(9):
            self.fovPath.poses.append(GeoPose())

        #init VizItem and add polygon
        self.fovVizItem = GeoVizItem()
        self.fovVizItem.polygons.append(GeoPath())
        self.fovVizItem.id = "FOV" 
        try:
            rospy.wait_for_service('enc_query_node')
            self.enc_query = rospy.ServiceProxy('enc_query_node', enc_query_srv)
        except rospy.ServiceException as e: 
            rospy.loginfo("ERROR IN SERVICE INITIALIZATION")

    def iterate(self,data):
        #calculate fov for current position
        self.calculateFOV()
        
        #update VizItem
        self.fovVizItem.polygons[0] = self.geoPathToGeoVizPoly(self.fovPath)
        
        #query enc
        layers = ['BCNCAR', 'BCNISD', 'BCNLAT', 'BCNSAW', 'BCNSPP', 'BOYCAR', 'BOYINB', 'BOYISD', 'BOYLAT', 'BOYSAW', 'BOYSPP', 'CGUSTA', 'CTRPNT', 'CURENT', 'DAYMAR', 'DISMAR', 'FOGSIG', 'LIGHTS', 'LITFLT', 'LITVES', 'PILPNT', 'RADRFL', 'RADSTA', 'RDOSTA', 'RETRFL', 'RSCSTA', 'RTPBCN', 'SISTAT', 'SISTAW', 'SOUNDG', 'SPRING', 'TOPMAR', 'UWTROC']
        t=  time.time()
        enc_ret = self.enc_query(layers,self.fovPath.poses)
        t = time.time() - t
        self.timelog[0] += t
        self.timelog[1] += 1
        rospy.loginfo("FOVQUERY TIME:" + str(t))
        rospy.loginfo("TIMELOG:" + str(self.timelog))

        #draw points in view
        inViewList = GeoVizPointList()
        inViewList.color = ColorRGBA(0,1,0,1)
        inViewList.size = 20
        self.featureViz = GeoVizItem()
        self.featureViz.point_groups = [None]
        # rospy.loginfo(enc_ret.featuresInView)
        for enc_feat in enc_ret.featuresInView:
            long = enc_feat.longitude
            lat=enc_feat.latitude
            # rospy.loginfo(str(long) +" "+ str(lat))

            feat = GeoPoint(lat,long,0)
            inViewList.points.append(feat)
           

        self.featureViz.point_groups[0] = inViewList
            
        # #pubish updates
        geoVizItem_pub.publish(self.fovVizItem)
        geoVizItem_pub.publish(self.featureViz)

    def calculateFOV(self):
        global asv_heading
        global asv_position
        
        long = asv_position.position.longitude
        lat = asv_position.position.latitude

        octoPoints = []
        angle = asv_heading;
        for i in range(9):
            point = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(angle),self.distance)
            angle = (angle+45)%360
            pose = self.fovPath.poses[i]
            pose.position.longitude = math.degrees(point[0])
            pose.position.latitude = math.degrees(point[1])
        

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
        
if __name__ == "__main__":
    rospy.init_node('fov_query')
    position_sub = rospy.Subscriber('/position', GeoPointStamped, positionCallback)
    heading_sub = rospy.Subscriber('/heading', NavEulerStamped, headingCallback)
    rospy.wait_for_message('/heading',NavEulerStamped)
    rospy.wait_for_message('/position', GeoPointStamped)

    geoVizItem_pub = rospy.Publisher('/project11/display',GeoVizItem,queue_size = 10)

    fov_query = FOVQuery(100000)
    rospy.Timer(rospy.Duration.from_sec(.1),fov_query.iterate)

    rospy.spin()