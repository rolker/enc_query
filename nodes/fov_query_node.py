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
from nav_msgs.msg import Odometry
from enc_query.srv import enc_query_srv
import tf2_ros
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

class FOVQuery:
    distance = 0
    fovPath = None
    fovVizItem = None
    enc_query = None

    def __init__(self,distance):
        self.distance = distance
        self.timelog = [0,0]

        self.odometry = None
        rospy.Subscriber('odom', Odometry, self.odometryCallback, queue_size = 1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
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
        if not self.calculateFOV():
            return
        
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
        asv_heading_rad = self.heading()
        asv_position = self.position()

        if asv_heading_rad is None or asv_position is None:
            return False
        
        long = asv_position[1]
        lat = asv_position[0]

        octoPoints = []
        angle = asv_heading_rad;
        for i in range(9):
            point = project11.geodesic.direct(long, lat, angle, self.distance)
            angle = angle+math.radians(45)
            pose = self.fovPath.poses[i]
            pose.position.longitude = math.degrees(point[0])
            pose.position.latitude = math.degrees(point[1])
        return True
        

    def geoPathToGeoVizPoly(self, path):
        poly = GeoVizPolygon()
        for geoPose in path.poses:
            poly.outer.points.append(geoPose.position)
        poly.fill_color = ColorRGBA(1,0,0,.2)
        return poly

    def odometryCallback(self, msg):
        self.odometry = msg

    def position(self):
      if self.odometry is not None:
        try:
          odom_to_earth = self.tfBuffer.lookup_transform("earth", self.odometry.header.frame_id, rospy.Time())
        except Exception as e:
          print(e)
          return
        ecef = do_transform_pose(self.odometry.pose, odom_to_earth).pose.position
        return project11.wgs84.fromECEFtoLatLong(ecef.x, ecef.y, ecef.z)

    def heading(self):
      if self.odometry is not None:
        o = self.odometry.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        return math.radians(90)-euler_from_quaternion(q)[2]

        
if __name__ == "__main__":
    rospy.init_node('fov_query')

    geoVizItem_pub = rospy.Publisher('project11/display',GeoVizItem,queue_size = 10)

    fov_query = FOVQuery(2000)
    rospy.Timer(rospy.Duration.from_sec(.1),fov_query.iterate)

    rospy.spin()