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
    def run(self):
        ds = ogr.Open(self.ENC_filename)
        print("Layer Count = %d" % ds.GetLayerCount())
        for i in range(ds.GetLayerCount()):
            layer = ds.GetLayerByIndex(i)
            print(layer.GetName(), layer.GetFeatureCount())
            feat = layer.GetNextFeature()
            geom = feat.GetGeometryRef()
            if geom is not None:
                print(geom.GetX(), geom.GetY())

if __name__ == "__main__":
    enc_files = ['/home/thomas/Downloads/ENC_ROOT/US5NH01M/US5NH01M.000']
    
    args = parser.parse_args()

    if args.file:
        enc_files.append(args.file)
    for enc in enc_files:
        print('Input file: %s' % enc)
        obj = enc_query(ENC_filename = enc)
        obj.run()