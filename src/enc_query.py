import os
from osgeo import ogr
import argparse
import fnmatch
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
class enc_query:
    """ Playing ENC files """
    def __init__(self, ENC_filename):
        self.ENC_filename = ENC_filename
        # self.ds = ogr.Open(self.ENC_filename)
        self.layerName = 'all'
        self.verbose = 0
    # def printObjectByIndex(self, layerIdx, objIdx):
    #     print("LayerIdx: %d - ObjectIdx: %d" % (layerIdx,objIdx))
    #     layer = self.ds.GetLayerByIndex(layerIdx)
    #     print("Layer Name : %s" % layer.GetName())
    #     print("Feature Count : %d" % layer.GetFeatureCount())
    #     # feat = layer.GetNextFeature()
    #     # print("Feature : %s" % feat.GetFieldAsString('OBJNAM'))
    #     # print("FID : %d" % feat.GetFID())
    #     # geom = feat.GetGeometryRef()
    #     # print("Coords: %d, %d" %( geom.GetX(), geom.GetY()) )
    #     # print("Coords", ( geom.GetPoint())) 
        
    #     boxCoords2 = ogr.Geometry(ogr.wkbLinearRing)
    #     boxCoords2.AddPoint(-70.863,43.123)
    #     boxCoords2.AddPoint(-70.855,43.123)
    #     boxCoords2.AddPoint(-70.855,43.12)
    #     boxCoords2.AddPoint(-70.863,43.12)
    #     boxCoords2.AddPoint(-70.863,43.123)

    #     box2 = ogr.Geometry(ogr.wkbPolygon)
    #     box2.AddGeometry(boxCoords2)

       
    #     # print("isValid: %s"% box2.IsValid())
    #     # print("Area: %d" % box2.GetArea())
    #     # print("overlap: %s"% box2.Contains(geom)) 
    #     # # print(thomas)

    #     self.checkLayer(layer,box2)
    # def checkLayer(self, layer, box):
    #     numFeats = layer.GetFeatureCount()
    #     print(numFeats)
    #     for i in range(numFeats):
    #         feat = layer.GetNextFeature()
    #         if feat is not None and box.Contains(feat.GetGeometryRef()):
    #             print("Feature %d : %s" % (i,feat.GetFieldAsString('OBJNAM')))

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
                            print("-----Feature %d : %s" % (i,feat.GetFieldAsString('OBJNAM')))
                            print("-----FID : %s" % str(feat.GetFID()) )
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
        obj.run()