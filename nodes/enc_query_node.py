#!/usr/bin/env python
from __future__ import print_function
import rospy
import project11
from enc_query.srv import enc_query_srv, enc_query_srvResponse
from enc_query.msg import enc_feature_msg
from osgeo import ogr
import geopandas as gpd
import pandas as pd
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from osgeo import ogr

import requests
import xml.etree.ElementTree as ET 

from enc_query.srv import enc_query_srv

class enc_query:
    # enc_root = ""
    fov = None
    # layerNames = []
    featureDataframe = None
    def __init__(self,enc_root):
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


    def getLayers(self,ds):
        numLayers = ds.GetLayerCount()
        layers = []
        for i in range(numLayers):
            layers.append(ds.GetLayerByIndex(i))
        return layers


    def getFeatures(self,layer):
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
        features = []
        for layer in layers:
            features += self.getFeatures(layer)
        return features

    def getLayersByNames(self,ds, layerNames):
        layers = []
        for layerName in layerNames:
            layer = ds.GetLayer(layerName)
            if layer is None:
                rospy.loginfo("Layer not found: " + layerName )
            else:
                layers.append(layer)
        return layers
        
    def getFeatureInfo(self,feature):
        geomRef = feature.GetGeometryRef()
        nameIndex = feature.GetFieldIndex("OBJNAM")
        featureName = "NO OBJNAM"
        if(nameIndex != -1 and feature.GetFieldAsString(nameIndex) != "" ):
            featureName = feature.GetFieldAsString(nameIndex)
        featureInfo = (featureName, feature.GetFID(), geomRef.GetX(), geomRef.GetY())
        # rospy.loginfo(featureInfo)
        return featureInfo


    def geoPathToGPD(self, inFOV):
        points = []
        for geoPose in inFOV:
            points.append((geoPose.position.longitude, geoPose.position.latitude))
        poly = Polygon(points)
        # rospy.logerr("FOVPoly:"+ str(poly))
        return gpd.GeoDataFrame({'geometry': [poly]})


    def run(self):
        response = []
        # rospy.loginfo("FeatureDataframe: " + str(self.featureDataframe))  
        # rospy.loginfo("FOV: " + str(self.fov) )  

        featuresInView = gpd.sjoin(self.featureDataframe, self.fov, op='within')    

        # rospy.loginfo("Features in view: " + str(featuresInView))    
        for index, feature in featuresInView.iterrows():
            response.append(enc_feature_msg(feature["name"], feature["longitude"],feature["latitude"], feature["fid"]))
        # rospy.loginfo("response: "+str(response))
        return response

    def service_handler(self,req):
        """Handle incomming requests from ROS service

        Args:
            req (srv): attributes for enc queryls

        Returns:
            list[features]: list of features within FOV
        """
    
        featureList = []
    
        # if req.layers:
        #     self.layerNames = req.layers
        # else:
        #     # THIS NEEDS FIXING
        #     self.layerNames = ['all']
        self.fov = self.geoPathToGPD(req.fov)
        featureList = self.run()
        return enc_query_srvResponse(featureList)

    # def service_testing(self):
    #     featureList = []
    #     self.fov = Polygon([(-70.855,43.123),(-70.855,43.12),(-70.863,43.12),(-70.863,43.123)])
    #     featureList = self.run()
    #     print(featureList)
    #     # return enc_query_srvResponse(featureList)
           

def enc_query_server():
    """Initialize node for ros server
    """
    rospy.init_node('enc_query_node')
    enc_root = rospy.get_param('enc_root')
    obj = enc_query(enc_root)
    s = rospy.Service('enc_query_node', enc_query_srv, obj.service_handler)
    rospy.spin()


if __name__ == "__main__":
    enc_query_server()
    
