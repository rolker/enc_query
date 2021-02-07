import math
import os
from osgeo import ogr
import argparse
import fnmatch

import project11
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

class enc_query:
    """ Playing with ENC files """
    def __init__(self, ENC_filename):
        self.ENC_filename = ENC_filename
        self.ds = ogr.Open(self.ENC_filename)
        self.layers = ['all']
        self.verbose = 0
        self.longlat = (0,0)#(-70.855229, 43.122453)
        self.azimuth = 0
        self.distance = 1000
        self.view_angle = 40
        self.fov = None

    def getBouyLat(self):
       
        layer = self.ds.GetLayer("BOYLAT")
        coords = []
        for i in range(layer.GetFeatureCount()):
            feat = layer.GetNextFeature()
            geom = feat.GetGeometryRef()
            coords.append((feat.GetFieldAsString('OBJNAM'), (geom.GetPoint()[1], geom.GetPoint()[0])))
            print((feat.GetFieldAsString('OBJNAM'), (geom.GetPoint()[1], geom.GetPoint()[0])))
        return coords

    def tempFOV(self):
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
        if self.verbose>=count:
            print(message)
    def queryFeatures(self,layer):
        self.verbosePrint("Querying features...")
        desc = layer.GetDescription()
        numFeats = layer.GetFeatureCount()
        self.verbosePrint('Layer: ' + desc)
        self.verbosePrint('\tNumFeats: ' + str(numFeats))
        feat = layer.GetNextFeature()
        for i in range(numFeats):
            featGeomRef = feat.GetGeometryRef()
            
            if featGeomRef is not None:
                if self.fov.Contains(featGeomRef):
                    if feat.GetFieldIndex('OBJNAM') is not None:
                        featDesc = feat.GetFieldAsString('OBJNAM')
                    else:
                        featDesc = "NO OBJNAM"
                    fid = feat.GetFID()
                    self.verbosePrint("Feature IS in range of FOV")
                    self.verbosePrint("\tFeature: " + str(featDesc))
                    self.verbosePrint("\t\tFID: " + str(fid))
                else:
                    self.verbosePrint("Feature NOT within FOV")
            else:
                self.verbosePrint("Feature has no Geom Ref: " + desc + ", " + str(i))
            feat = layer.GetNextFeature()

        self.verbosePrint("---------------------------")
    def queryLayers(self):
        self.verbosePrint("Querying layers...")
        numLayers = self.ds.GetLayerCount()
        # boxCoords = ogr.Geometry(ogr.wkbLinearRing)
        # boxCoords.AddPoint(-70.863,43.123)
        # boxCoords.AddPoint(-70.855,43.123)
        # boxCoords.AddPoint(-70.855,43.12)
        # boxCoords.AddPoint(-70.863,43.12)
        # boxCoords.AddPoint(-70.863,43.123)

        # box = ogr.Geometry(ogr.wkbPolygon)
        # box.AddGeometry(boxCoords)
       
        if 'all' in self.layers:
            self.verbosePrint("numLayers: " + str(numLayers))
            for i in range(numLayers):
                layer = self.ds.GetLayerByIndex(i)
                # try:
                self.queryFeatures(layer)
                # except:
                    # print("ERROR on layer %d" % i)
        else:
            for layerName in self.layers:
                try:
                    layer = self.ds.GetLayer(layerName)
                    self.queryFeatures(layer)
                except:
                    self.verbosePrint("Layer not fount: " + layerName )
                
            # print("Layer: %s, NumFeats: %d" % (self.layerName, self.ds.GetLayer(self.layerName).GetFeatureCount()))
        # # if(layerName)
        # 
            
    def run(self):
        self.verbosePrint("Running...")
        fov = self.tempFOV()
        self.fov = fov
        if self.fov is not None:
            self.queryLayers()
        else:
            self.verbosePrint('FOV is none')



if __name__ == "__main__":
    enc_files = ['/home/thomas/Downloads/ENC_ROOT/US5NH01M/TestFile/US5NH01M.000']
    
    args = parser.parse_args()
    verbose = args.verbose

    if verbose >= 2:
        print("Arguments:")
        arguments = vars(args)
        for key, value in arguments.iteritems():
            print("\t%s:\t\t%s" % (key,str(value)))

    if args.file:
        enc_files.append(args.file)
    if args.directory:
        directory = args.directory
        for root, dirnames, filesnames in os.walk(directory):
            for filename in fnmatch.filter(filesnames, 'US*.000'):
                enc_files.append(os.path.join(root,filename))

    for enc in enc_files:
        if verbose >= 2:
            print('Processing %s ' % enc)
            print('Input file: %s' % enc)
        obj = enc_query(ENC_filename = enc)
        obj.verbose = verbose 
        if args.layers:
            obj.layers = args.layers
        if args.position:
            obj.longlat = tuple(args.position)
        obj.azimuth = args.azimuth
        obj.distance = args.meters
        obj.run()        