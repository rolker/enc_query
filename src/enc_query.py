import os
from osgeo import ogr
import argparse

parser = argparse.ArgumentParser()

group = parser.add_mutually_exclusive_group()
group.add_argument('-f', '--file', action='store', help= 'ENC file input')

class enc_query:
    """ Playing ENC files """
    def __init__(self, ENC_filename):
        self.ENC_filename = ENC_filename
        self.ds = ogr.Open(self.ENC_filename)
    def printObjectByIndex(self, layerIdx, objIdx):
        print("LayerIdx: %d - ObjectIdx: %d" % (layerIdx,objIdx))
        layer = self.ds.GetLayerByIndex(layerIdx)
        print("Layer Name : %s" % layer.GetName())
        print("Feature Count : %d" % layer.GetFeatureCount())
        # feat = layer.GetNextFeature()
        # print("Feature : %s" % feat.GetFieldAsString('OBJNAM'))
        # print("FID : %d" % feat.GetFID())
        # geom = feat.GetGeometryRef()
        # print("Coords: %d, %d" %( geom.GetX(), geom.GetY()) )
        # print("Coords", ( geom.GetPoint())) 
        
        boxCoords2 = ogr.Geometry(ogr.wkbLinearRing)
        boxCoords2.AddPoint(-70.863,43.123)
        boxCoords2.AddPoint(-70.855,43.123)
        boxCoords2.AddPoint(-70.855,43.12)
        boxCoords2.AddPoint(-70.863,43.12)
        boxCoords2.AddPoint(-70.863,43.123)

        box2 = ogr.Geometry(ogr.wkbPolygon)
        box2.AddGeometry(boxCoords2)

       
        # print("isValid: %s"% box2.IsValid())
        # print("Area: %d" % box2.GetArea())
        # print("overlap: %s"% box2.Contains(geom)) 
        # # print(thomas)

        self.checkLayer(layer,box2)
    def checkLayer(self, layer, box):
        numFeats = layer.GetFeatureCount()
        print(numFeats)
        for i in range(numFeats):
            feat = layer.GetNextFeature()
            if feat is not None and box.Contains(feat.GetGeometryRef()):
                print("Feature %d : %s" % (i,feat.GetFieldAsString('OBJNAM')))



    def run(self):
        #print file overview
        print("Layer Count: %d" % self.ds.GetLayerCount())
        # for i in range(ds.GetLayerCount()):
        #     print("Index: %d" % i)
        #     layer = ds.GetLayerByIndex(i)
        #     print(layer.GetName(), layer.GetFeatureCount(), layer.GetDescription())
           
        #     feat = layer.GetNextFeature()
        #     print(feat.GetFieldCount())
        #     for feat_attribute in feat.keys():  
        #        print('       ATTR:' + feat_attribute + ':' + feat.GetFieldAsString(feat_attribute))
        self.printObjectByIndex(6,1)
        # self.checkBox(())
            # if geom is not None:
            #     print(geom.GetX(), geom.GetY())
  
        

if __name__ == "__main__":
    enc_files = ['/home/thomas/Downloads/ENC_ROOT/US5NH01M/TestFile/US5NH01M.000']
    
    args = parser.parse_args()

    if args.file:
        enc_files.append(args.file)
    for enc in enc_files:
        print('Input file: %s' % enc)
        obj = enc_query(ENC_filename = enc)
        obj.run()

        #43 7.4 70 51.8   43 7.4 70 51.3
        #43 7.2 70 51.3   43 7.2 70 51.8

        # 43.123 -70.863    43.123 -70.855
        # 43.12  -70.855    43.12  -70.863

     