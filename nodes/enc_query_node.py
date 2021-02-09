#!/usr/bin/env python
from __future__ import print_function
import rospy
from enc_query.srv import enc_query_srv, enc_query_srvResponse
from enc_query.msg import enc_feature_msg
import math
import os
from osgeo import ogr
import argparse
import fnmatch

import project11

#parse manager
parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group()
group.add_argument('-f', '--file',
                    action='store', 
                    help= 'ENC file input')
group.add_argument('-d','--directory',
                    action = 'store',
                    help = 'Directory of ENC files to survey.')
parser.add_argument('-l','--layers',
                    nargs='+',
                    type = str,
                    help = 'Layers to query. "all" for all layers. --layer boylat wrecks') 
parser.add_argument('-v','--verbose',
                    action = 'count',
                    help = 'Verbose output, -vvv = more output')
parser.add_argument('-p','--position',
                    nargs='+',
                    type = float,
                    help = ' Longitude and latitude: -p -70.2 30.4') #long lat may not be the best way to do this because im using points
parser.add_argument('-a','--azimuth',
                    action='store',
                    help = 'Degrees from due north')
parser.add_argument('-m','--meters',
                    action='store',
                    help = 'distance in meters')

class enc_feature:
    def __init__(self,fid, name = 'NoName'):
        self.fid = fid
        self.name = name

class enc_query:
    """ enc_query class  """
    def __init__(self, ENC_filename):
        """ Initialize an enc_query
        Args:
            ENC_filename (string): path pointing to S-57 file to be queried
        """
        self.ENC_filename = ENC_filename
        self.ds = ogr.Open(self.ENC_filename)
        self.layers = []
        self.verbose = 0
        self.longlat = (0,0)#(-70.855229, 43.122453)
        self.azimuth = 0
        self.distance = 1000
        self.view_angle = 40
        self.fov = None
    
    # Function used for testing
    # def getBouyLat(self):
       
    #     layer = self.ds.GetLayer("BOYLAT")
    #     coords = []
    #     for i in range(layer.GetFeatureCount()):
    #         feat = layer.GetNextFeature()
    #         geom = feat.GetGeometryRef()
    #         coords.append((feat.GetFieldAsString('OBJNAM'), (geom.GetPoint()[1], geom.GetPoint()[0])))
    #         print((feat.GetFieldAsString('OBJNAM'), (geom.GetPoint()[1], geom.GetPoint()[0])))
    #     return coords

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
        long, lat = self.longlat

        #Angles for direction of 
        lDeg = self.azimuth - (self.view_angle/2)
        rDeg = self.azimuth + (self.view_angle/2)

        #Calculate points of geometry
        lPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(lDeg),self.distance)
        mPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(self.azimuth),self.distance)
        rPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(rDeg),self.distance)

        #Add points to geometry
        fovCoords.AddPoint(long, lat)
        fovCoords.AddPoint(math.degrees(lPoint[0]),math.degrees(lPoint[1]))
        fovCoords.AddPoint(math.degrees(mPoint[0]),math.degrees(mPoint[1]))
        fovCoords.AddPoint(math.degrees(rPoint[0]),math.degrees(rPoint[1]))

        #Create Polygon
        fov = ogr.Geometry(ogr.wkbPolygon)
        fov.AddGeometry(fovCoords)

        return fov

    def verbosePrint(self, message, count=1):
        """print function for when -v flag is used

        Args:
            message ( string ): message to be printed 
            count (int, optional): Level of verbosity. Defaults to 1.
        """
        if self.verbose>=count:
            print(message)

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
            
            # feat = layer.GetNextFeature()

        self.verbosePrint("---------------------------")

        return featureList
    def queryLayers(self):
        """Query through given layers

        Returns:
            enc_feature[]: list of enc_feature type msg
        """
        self.verbosePrint("Querying layers...")

        numLayers = self.ds.GetLayerCount()
        featureList = []

        if 'all' in self.layers:
            self.verbosePrint("numLayers: " + str(numLayers))
            for i in range(numLayers):
                layer = self.ds.GetLayerByIndex(i)
                featureList += self.queryFeatures(layer)
        else:
            for layerName in self.layers:
                try:
                    layer = self.ds.GetLayer(layerName)
                    featureList += self.queryFeatures(layer)
                except:
                    self.verbosePrint("Layer not found: " + layerName )
        print("FEATURELIST:",featureList)
        return featureList

            
    def run(self):
        """Check initization of FOV

        Returns:
            enc_feature[]: list of features within query fov
        """
        self.verbosePrint("Running...")
        fov = self.tempFOV()
        self.fov = fov
        if self.fov is not None:
            try:
                response = self.queryLayers()
                print("RESPONSE:", response)
                return response
            except:
                self.verbosePrint("ERROR IN queryLayers ")
        else:
            self.verbosePrint('FOV is none')

def enc_query_server():
    print("enc_query_server called...")
    rospy.init_node('enc_query_node')
    print("ENC NODE INITED")
    s = rospy.Service('enc_query_node', enc_query_srv, service_handler)
    rospy.spin()
    
def service_handler(req):
    enc_files = ['/home/thomas/Downloads/ENC_ROOT/US5NH01M/TestFile/US5NH01M.000']
    verbose = 1
    featureList = []
    if verbose:
        print("Path", req.path)
        print("Directory", req.directory)
        print("Layers", req.layers)
        print("Longitude", req.longitude)
        print("Latitude", req.latitude)
        print("Azimuth", req.azimuth)
        print("Distance", req.distance)
        print("View_angle", req.view_angle)

    if req.path:
        enc_files.append(req.path)
    if req.directory:
        directory = req.directory
        for root, dirnames, filesnames in os.walk(directory):
            for filename in fnmatch.filter(filesnames, 'US*.000'):
                enc_files.append(os.path.join(root,filename))

    for enc in enc_files:
        if verbose >= 2:
            print('Processing %s ' % enc)
            print('Input file: %s' % enc)
        obj = enc_query(ENC_filename = enc)
        obj.verbose = verbose 
        if req.layers:
            obj.layers = req.layers
        if req.latitude and req.longitude:
            obj.longlat = (req.longitude, req.latitude)
        obj.azimuth = req.azimuth
        obj.distance = req.distance
        featureList += obj.run()
    print("FINAL FEATURELIST:",featureList)
    return enc_query_srvResponse(featureList) 

if __name__ == "__main__":
    enc_query_server()
    # enc_files = ['/home/thomas/Downloads/ENC_ROOT/US5NH01M/TestFile/US5NH01M.000']
    
    # args = parser.parse_args()
    # verbose = args.verbose

    # if verbose >= 2:
    #     print("Arguments:")
    #     arguments = vars(args)
    #     for key, value in arguments.iteritems():
    #         print("\t%s:\t\t%s" % (key,str(value)))

    # if args.file:
    #     enc_files.append(args.file)
    # if args.directory:
    #     directory = args.directory
    #     for root, dirnames, filesnames in os.walk(directory):
    #         for filename in fnmatch.filter(filesnames, 'US*.000'):
    #             enc_files.append(os.path.join(root,filename))

    # for enc in enc_files:
    #     if verbose >= 2:
    #         print('Processing %s ' % enc)
    #         print('Input file: %s' % enc)
    #     obj = enc_query(ENC_filename = enc)
    #     obj.verbose = verbose 
    #     if args.layers:
    #         obj.layers = args.layers
    #     if args.position:
    #         obj.longlat = tuple(args.position)
    #     obj.azimuth = args.azimuth
    #     obj.distance = args.meters
    #     obj.run()        