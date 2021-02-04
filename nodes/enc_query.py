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
parser.add_argument('-l','--layer',
                    action='store',
                    help = 'Layer to Extract, or "all" for all of them.')
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
parser.add_argument('-m','--meters ',
                    action='store',
                    help = 'distance in meters')

class enc_query:
    """ Playing ENC files """
    def __init__(self, ENC_filename):
        self.ENC_filename = ENC_filename
        # self.ds = ogr.Open(self.ENC_filename)
        self.layerName = 'all'
        self.verbose = 0
        self.longlat = (0,0)
        self.azimuth = 0
        self.distance = 10
    def calculateFOV(self):
    # periods signify points
    #          .
    #    .     |     .
    #     \    |10m /
    #      \   |   /
    # 10m   \  |  /  10m
    #        \ | /
    #         \|/
    #          o

        fovCoords = ogr.Geometry(ogr.wkbLinearRing)
        long, lat = self.longlat
        lDeg = self.azimuth - 5
        rDeg = self.azimuth + 5
        lPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(lDeg),self.distance)
        mPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(self.azimuth),self.distance)
        rPoint = project11.geodesic.direct(math.radians(long),math.radians(lat),math.radians(rDeg),self.distance)
        fovCoords.AddPoint(long, lat)
        fovCoords.AddPoint(math.degrees(lPoint[0]),math.degrees(lPoint[1]))
        fovCoords.AddPoint(math.degrees(mPoint[0]),math.degrees(mPoint[1]))
        fovCoords.AddPoint(math.degrees(rPoint[0]),math.degrees(rPoint[1]))
        fov = ogr.Geometry(ogr.wkbPolygon)
        fov.AddGeometry(fovCoords)

        return fov
        
    def run(self):
        ds = ogr.Open(self.ENC_filename)
        numLayers = ds.GetLayerCount()
        boxCoords = ogr.Geometry(ogr.wkbLinearRing)
        boxCoords.AddPoint(-70.863,43.123)
        boxCoords.AddPoint(-70.855,43.123)
        boxCoords.AddPoint(-70.855,43.12)
        boxCoords.AddPoint(-70.863,43.12)
        boxCoords.AddPoint(-70.863,43.123)

        box = ogr.Geometry(ogr.wkbPolygon)
        box.AddGeometry(boxCoords)

        if verbose >=2:
            print("Found %d Layers" % numLayers)
        # if(layerName)
        for i in range(numLayers):
            layer = ds.GetLayerByIndex(i)
            try:
                desc = layer.GetDescription()
                # print(desc)
                numFeatures = layer.GetFeatureCount()
                # print(numFeatures)
                if verbose >= 2:
                    print("Found %d features in layer %s" % (numFeatures,desc))
                for i in range(numFeatures):
                    feat = layer.GetNextFeature()
                    if feat is not None and box.Contains(feat.GetGeometryRef()):
                        if feat.GetFieldAsString('OBJNAM') is not None:
                            print("-----Feature %d : %s FID : %s"  % (i,feat.GetFieldAsString('OBJNAM'),str(feat.GetFID())))
                        else:        
                            print("-----FID : %s" % str(feat.GetFID()) )


            except:
                print("Error on layer %d" % i)
            
            

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
        obj.layerName = args.layer
        obj.longlat = tuple(args.position)
        obj.azimuth = args.azimuth
        obj.distance = args.distance
        obj.calculateFOV()
        # obj.run()