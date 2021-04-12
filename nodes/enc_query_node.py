#!/usr/bin/env python
from __future__ import print_function
try:
    ROS = True
    import rospy
    import project11
    from enc_query.srv import enc_query_srv, enc_query_srvResponse
    from enc_query.msg import enc_feature_msg
except ImportError:
    ROS = False
from osgeo import ogr
from socket import *
import geopandas as gpd
import pandas as pd
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from osgeo import ogr
import pickle

class enc_query:
    """ Class for querying a ENC_ROOT folder of enc maps for features of type point within a input polygon. """
    
    fov = None
    # layerNames = []
    featureDataframe = None
    isROS = False;
    def __init__(self,enc_root, isROS):
        self.isROS = isROS
        self.enc_root = enc_root
        filenames = ['US2EC03M', 'US2EC04M', 'US3EC10M', 'US3EC11M', 'US4MA04M', 'US4MA19M', 'US4ME01M', 'US5MA1AM', 'US5MA04M', 'US5MA19M', 'US5ME01M', 'US5NH01M', 'US5NH02M']
        layerNames = ['BCNCAR', 'BCNISD', 'BCNLAT', 'BCNSAW', 'BCNSPP', 'BOYCAR', 'BOYINB', 'BOYISD', 'BOYLAT', 'BOYSAW', 'BOYSPP', 'CGUSTA', 'CTRPNT', 'CURENT', 'DAYMAR', 'DISMAR', 'FOGSIG', 'LIGHTS', 'LITFLT', 'LITVES', 'PILPNT', 'RADRFL', 'RADSTA', 'RDOSTA', 'RETRFL', 'RSCSTA', 'RTPBCN', 'SISTAT', 'SISTAW', 'SOUNDG', 'SPRING', 'TOPMAR', 'UWTROC']
        # filenames = ["US5NH01M"]

        features = []
        for filename in filenames:    
            ds = ogr.Open(self.enc_root + '/' + filename + '/' + filename + '.000')
            layers = self.getLayersByNames(ds,layerNames)
            # rospy.loginfo("layers: " + str(layers))
            features += self.getAllFeatures(layers)
        # rospy.loginfo("features:"+ str(features))
        df = pd.DataFrame.from_records(features, columns=['name', 'fid', 'longitude', 'latitude'])
        self.featureDataframe = gpd.GeoDataFrame(df,geometry=gpd.points_from_xy(df.longitude, df.latitude))

    @classmethod
    def ROS_Query(cls, enc_root):
        return cls(enc_root, True)
    @classmethod
    def UDP_Query(cls, enc_root):
        return cls(enc_root, False)

    def getLayers(self,ds):
        """Get all layers in input dataset

        Args:
            ds (ogr.DataSource): dataset to query

        Returns:
            ogr.Layer[]: List of layers 
        """
        numLayers = ds.GetLayerCount()
        layers = []
        for i in range(numLayers):
            layers.append(ds.GetLayerByIndex(i))
        return layers


    def getFeatures(self,layer):
        """Get all the features from the provided layer

        Args:
            layer (ogr.Layer): layer to pull features from

        Returns:
            ogr.Feature[]: featuers in layer
        """     
        numFeatures = layer.GetFeatureCount()
        features = []
        for i in range(numFeatures):
            feature = layer.GetNextFeature()
            if feature is not None:
                geomRef = feature.GetGeometryRef()
                if((geomRef is not None and geomRef.GetPointCount() != 0)):
                    features.append(self.getFeatureInfo(feature))
        return features


    def getAllFeatures(self,layers):
        """Get all features from all input layers 

        Args:
            layers (ogr.Layer[]): List of layers

        Returns:
            ogr.Feature[]: List of features with layers
        """ 
        features = []
        for layer in layers:
            features += self.getFeatures(layer)
        return features

    def getLayersByNames(self,ds, layerNames):
        """Get layers from datasource using specific layer names

        Args:
            ds (ogr.Datasource): datasource to query
            layerNames (String[]): List of strings

        Returns:
            ogr.Layer[]: List of layers from datasource
        """
        layers = []
        for layerName in layerNames:
            layer = ds.GetLayer(layerName)
            if layer is None:
                rospy.loginfo("Layer not found: " + layerName )
            else:
                layers.append(layer)
        return layers
        
    def getFeatureInfo(self,feature):
        """Given a feauture, get the featurename, featureID, longitude, and latitude of feature.

        Args:
            feature (ogr.Feature): feature 

        Returns:
            tuple: featureName, FID, longitude, latitude
        """
        geomRef = feature.GetGeometryRef()
        nameIndex = feature.GetFieldIndex("OBJNAM")
        featureName = "NO OBJNAM"
        if(nameIndex != -1 and feature.GetFieldAsString(nameIndex) != "" ):
            featureName = feature.GetFieldAsString(nameIndex)
        featureInfo = (featureName, feature.GetFID(), geomRef.GetX(), geomRef.GetY())
        # rospy.loginfo(featureInfo)
        return featureInfo


    def geoPathToGPD(self, inFOV):
        """Take a geoPath message and convert into GeoPandas Dataframe

        Args:
            inFOV (GeoPath): ros message GeoPath

        Returns:
            GeoDataFrame: dataframe containing only the polygon in geometry column
        """ 
        points = []
        for geoPose in inFOV:
            points.append((geoPose.position.longitude, geoPose.position.latitude))
        poly = Polygon(points)
        # rospy.logerr("FOVPoly:"+ str(poly))
        return gpd.GeoDataFrame({'geometry': [poly]})

    def pickleToGPD(self,inFOV):
        points = pickle.loads(inFOV)
        poly = Polygon(points)
        return gpd.GeoDataFrame({'geometry': [poly]})



    def run(self, req):
        """Find all features in polygon using GeoPandas sjoin spatial indexing feature.

        Returns:
            enc_feature_msg: ros feature msg containing feauture  name, longitude, latitude, fid
        """

        if(self.isROS):
            featureList = []
            self.fov = self.geoPathToGPD(req.fov)
        else:
            self.fov = self.pickleToGPD(req)


            
        response = []
    
        featuresInView = gpd.sjoin(self.featureDataframe, self.fov, op='within')     
        for index, feature in featuresInView.iterrows():
            if(self.isROS):
                response.append(enc_feature_msg(feature["name"], feature["longitude"],feature["latitude"], feature["fid"]))
            else:
                response.append((feature["name"], feature["longitude"],feature["latitude"], feature["fid"]))
        if(self.isROS):
            return enc_query_srvResponse(response)
        return response

    # def service_testing(self):
    #     featureList = []
    #     self.fov = Polygon([(-70.855,43.123),(-70.855,43.12),(-70.863,43.12),(-70.863,43.123)])
    #     featureList = self.run()
    #     print(featureList)
    #     # return enc_query_srvResponse(featureList)
           

def enc_query_server(isROS):
    """Initialize node for ros server or UDP server
    """
    if(isROS):
        rospy.init_node('enc_query_node')
        enc_root = rospy.get_param('enc_root')
        obj = enc_query.ROS_Query(enc_root)
        s = rospy.Service('enc_query_node', enc_query_srv, obj.run)
        rospy.spin()
    else:
        serverSocket = socket(AF_INET, SOCK_DGRAM)
        serverSocket.bind(('', 12000))
        obj = enc_query.UDP_Query("/home/thomas/Downloads/ENC_ROOT")

        # long, lat = -70.855,43.123
        print("Server running...")
        while True:
            fov, address = serverSocket.recvfrom(1024)
            response = obj.run(fov)
            print(response)
            response = pickle.dumps(response)
            serverSocket.sendto(response,address)



if __name__ == "__main__":
    ROS = False
    enc_query_server(ROS)
    