#!/usr/bin/env python
from __future__ import print_function
import rospy
import project11
from enc_query.srv import enc_query_srv, enc_query_srvResponse
from enc_query.msg import enc_feature_msg
import math
import os
from osgeo import ogr
import argparse
import fnmatch
import requests
import xml.etree.ElementTree as ET 
import traceback

#Quick fix for error:
#   UnicodeEncodeError: 'ascii' codec can't encode characters in position 88013-88015: ordinal not in range(128)
import sys  
reload(sys)  
sys.setdefaultencoding('utf-8')
  

class enc_feature:
    def __init__(self,fid, name = 'NoName'):
        self.fid = fid
        self.name = name

class enc_query:
    """ enc_query class  """
    def __init__(self, enc_root):
        """ Initialize an enc_query
        Args:
            ENC_filename (string): path into enc_root
        """
        self.enc_root = enc_root
        self.layers = []
        self.verbose = 0
        self.longitude = 0
        self.latitude = 0
        self.bearing = 0
        self.distance = 1000
        self.view_angle = 40
        self.fov = None
        self.enc_geometries = None

    def verbosePrint(self, message, count=1):
        """print function for when -v flag is used

        Args:
            message ( string ): message to be printed 
            count (int, optional): Level of verbosity. Defaults to 1.
        """
        if self.verbose>=count:
            print(message)

    def tempFOV(self):
        """temporary field of view for testing

        Returns:
            pgr.Geometry : fov with Hen island and Eight-Foot rock bouys
        """
        boxCoords = ogr.Geometry(ogr.wkbLinearRing)
        boxCoords.AddPoint(-70.863,43.123)
        boxCoords.AddPoint(-70.855,43.123)
        boxCoords.AddPoint(-70.855,43.12)
        boxCoords.AddPoint(-70.863,43.12)
        boxCoords.AddPoint(-70.863,43.123)
        box = ogr.Geometry(ogr.wkbPolygon)
        box.AddGeometry(boxCoords)
        return box
        
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
        fovCoords = ogr.Geometry(ogr.wkbLinearRing)
        long = self.longitude
        lat = self.latitude

        #Angles for direction of 
        lDeg = self.bearing - (self.view_angle/2)
        rDeg = self.bearing + (self.view_angle/2)

        #Calculate points of geometry
        lPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(lDeg),self.distance)
        mPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(self.bearing),self.distance)
        rPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(rDeg),self.distance)

        #Add points to geometry
        fovCoords.AddPoint(long, lat)
        fovCoords.AddPoint(math.degrees(lPoint[0]),math.degrees(lPoint[1]))
        fovCoords.AddPoint(math.degrees(mPoint[0]),math.degrees(mPoint[1]))
        fovCoords.AddPoint(math.degrees(rPoint[0]),math.degrees(rPoint[1]))
        fovCoords.AddPoint(long, lat)
        #Create Polygon
        fov = ogr.Geometry(ogr.wkbPolygon)
        fov.AddGeometry(fovCoords)

        return fov
    def queryLayers(self, ds):
        """Query through given layers

        Returns:
            enc_feature[]: list of enc_feature type msg
        """
        self.verbosePrint("Querying layers...")

        numLayers = ds.GetLayerCount()
        featureList = []

        if 'all' in self.layers:
            self.verbosePrint("numLayers: " + str(numLayers))
            for i in range(numLayers):
                layer = ds.GetLayerByIndex(i)
                featureList += self.queryFeatures(layer)
        else:
            for layerName in self.layers:
                try:
                    layer = ds.GetLayer(layerName)
                    featureList += self.queryFeatures(layer)
                except:
                    self.verbosePrint("Layer not found: " + layerName )
        # verbosePrint("FEATURELIST:" + str(featureList))
        return featureList

    def queryFeatures(self,layer):
        """Querys through specified layer

        Args:
            layer (org.Layer): Layer to be queried

        Returns:
            ogr.Feature: List of features within fov 
        """
        self.verbosePrint("Querying features...")

        desc = layer.GetDescription()
        numFeats = layer.GetFeatureCount()
        featureList = []
        
        self.verbosePrint('Layer: ' + desc)
        self.verbosePrint('\tNumFeats: ' + str(numFeats))
        print("NUMFEATS: ", numFeats)
        for i in range(numFeats):
            feat = layer.GetNextFeature()
            featGeomRef = feat.GetGeometryRef()
            fid = feat.GetFID()
            
            #Check if feature contains Geometry reference
            if featGeomRef is not None:
                #Check if Geometry reference is contained within FOV
                if self.fov.Contains(featGeomRef):
                    if feat.GetFieldIndex('OBJNAM') is not None:
                        featDesc = feat.GetFieldAsString('OBJNAM')
                    else:
                        featDesc = "NO OBJNAM"

                    self.verbosePrint("Feature IS in range of FOV",2)
                    self.verbosePrint("\tFeature: " + str(featDesc))
                    self.verbosePrint("\t\tFID: " + str(fid))

                    featureList.append(enc_feature_msg(featDesc,fid))
                else:
                    self.verbosePrint("Feature NOT within FOV",2)
            else:
                self.verbosePrint("Feature has no Geom Ref: " + desc + ", " + str(i))

        self.verbosePrint("---------------------------")

        return featureList

    def getEncGeometries(self, catalog_location):
        self.verbosePrint("Getting ENC Geometries")
        #get xml from web
        if(catalog_location[:4].lower() == 'http'):
            resp = requests.get(catalog_location).text 
            root = ET.fromstring(resp) 
        #get xml from local file
        else:
            tree = ET.parse(catalog_location)
            root = tree.getroot()

        #define xml namespace variables
        #This should be derrived from the file but ET does not directly support it, namespaces look to be 
        #consistant throughout all of the catalogs. Possible solution is to directly parse out from xml text file
        ns = {'d': 'http://www.isotc211.org/2005/gmd', "f":"http://www.opengis.net/gml/3.2"}

        #declare return dict
        encGeometries = {}

        #search though xml
        for map in root.findall("./d:composedOf/d:DS_DataSet/d:has/d:MD_Metadata/d:identificationInfo/d:MD_DataIdentification/d:extent/d:EX_Extent/d:geographicElement/d:EX_BoundingPolygon/d:polygon/f:Polygon",ns): 
            #key id of polygon i.e. filename
            key = map.attrib['{http://www.opengis.net/gml/3.2}id']
            self.verbosePrint(key,3)
            
            #declare array for latlong points
            points = []
            
            #get latlong positions of each point of polygon 
            for point in map.findall("./f:exterior/f:LinearRing/f:pos",ns):
                lat,long = point.text.split()
                long = float(long)
                lat = float(lat)
                points.append([long,lat])

            #declare new geometry for bountry
            encGeometry = ogr.Geometry(ogr.wkbLinearRing)

            #add all the points
            for point in points:
                encGeometry.AddPoint(point[0], point[1])
            #close the geometry
            encGeometry.AddPoint(points[0][0], points[0][1])
            #declare polygon
            encGeometryPoly = ogr.Geometry(ogr.wkbPolygon)

            #add geometry to polygon
            encGeometryPoly.AddGeometry(encGeometry) 

            if not encGeometryPoly.IsValid():
                print("GEOMETRY NOT VALID: ", key )
                exit()

            encGeometries[key] = encGeometryPoly
            # self.verbosePrint()
        return encGeometries
    def getENC(self):
        """Get the appropriate enc file based on current position
        """
        position = ogr.Geometry(ogr.wkbPoint)
        position.SetPoint_2D(0, self.longitude, self.latitude)
        enc_filenames = []
        for key in self.enc_geometries:
            if position.Within(self.enc_geometries[key]):
                self.verbosePrint("Position is within:" + key)
                enc_filenames.append(key[:-3]) #remove _P1 label to just get filename
        return enc_filenames
    def run(self):
        """Get correct ENC file, build FOV, then query

        Returns:
            enc_feature[]: list of features within query fov
        """
        self.verbosePrint("Running...")
        enc_filenames = self.getENC()
        # exit()
        fov = self.tempFOV()
        self.fov = fov
        response = []
        if self.fov.IsValid():
            if self.fov is not None:
                for enc_filename in enc_filenames:
                    ds = ogr.Open(self.enc_root + '/' + enc_filename + '/' + enc_filename + '.000')
                    fileResponse = self.queryLayers(ds)
                    self.verbosePrint(enc_filename+" response: " + str(fileResponse))
                    response += fileResponse
                return response
            else:
                self.verbosePrint('FOV is none')
        else:
            self.verbosePrint("FOV not valid geometry")
    def service_handler(self,req):
        """Handle incomming requests from ROS service

        Args:
            req (srv): attributes for enc query

        Returns:
            list[features]: list of features within FOV
        """
    
        featureList = []
       
        self.verbosePrint("Layers", req.layers)
        self.verbosePrint("Longitude", req.longitude)
        self.verbosePrint("Latitude", req.latitude)
        self.verbosePrint("Bearing", req.bearing)
        self.verbosePrint("Distance", req.distance)
        self.verbosePrint("View_angle", req.view_angle)
            
    
        if req.layers:
            self.layers = req.layers
        else:
            self.layers = ['all']
        if req.latitude and req.longitude:
            self.longitude = req.longitude
            self.latitude = req.latitude
        self.bearing = req.bearing
        self.distance = req.distance
        featureList = self.run()

        return enc_query_srvResponse(featureList)
def enc_query_server():
    """Initialize node for ros server
    """
    print("enc_query_server called...")
    rospy.init_node('enc_query_node')
    # enc_root = '/home/thomas/Downloads/ENC_ROOT'
    enc_root = rospy.get_param('enc_query/enc_root')
    catalog_location = rospy.get_param('enc_query/catalog_location')
    # catalog_location = 'https://www.charts.noaa.gov/ENCs/NH_ENCProdCat_19115.xml'
    verbose = 1
    obj = enc_query(enc_root)
    obj.verbose = verbose 
    obj.enc_geometries = obj.getEncGeometries(catalog_location)
    s = rospy.Service('enc_query_node', enc_query_srv, obj.service_handler)
    rospy.spin()


if __name__ == "__main__":
    enc_query_server()
    # #define the ENC_ROOT directory
    # enc_root = '/home/thomas/Downloads/ENC_ROOT'

    # #set up enc_query attributes
    # verbose = 1
    # layers = ['all']
    # longitude = -70.863 
    # latitude = 43.123
    # bearing = 0
    # distance = 1000

    # obj = enc_query(enc_root)
    # obj.verbose = verbose
    # obj.layers = layers
    # obj.longitude = longitude
    # obj.latitude = latitude
    # obj.bearing = bearing
    # obj.distance = distance
    # obj.enc_root = enc_root
    # obj.run()





    

