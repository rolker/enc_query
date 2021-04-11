#!/usr/bin/env python
import pickle
import time
import socket
import math

def calculateFOV(long, lat, distance):
    poly = []
    angle = 0;
    for i in range(9):
        point = direct(math.radians(long),math.radians(lat),math.radians(angle),distance)
        angle = (angle+45)%360
        poly.append((math.degrees(point[0]),math.degrees(point[1])))
    return poly




def direct(lon1,lat1,azimuth,distance):
    '''Calculates the postion P2 from azimuth and distance from P1 on the WGS84 ellipsoid.
    @param lon1,lat1: Position P1 in rads
    @param azimuth: Clockwise angle in rads relative to true north.
    @param distance: Distance in meters
    @return: Position P2 in rads (lon2,lat2)
    '''
    
    phi1 = lat1
    alpha1 = azimuth
    
    a = 6378137.0        # length of major axis of the ellipsoid (radius at equator in meters)
    b = 6356752.314245   # length of minor axis of the ellipsoid (radius at the poles)
    f = 1/298.257223563  # flattening of the ellipsoid

    epsilon = 1e-12;

    #U is 'reduced latitude'
    tanU1 = (1.0-f)*math.tan(phi1)
    cosU1 = 1/math.sqrt(1+tanU1*tanU1)
    sinU1 = tanU1*cosU1

    cosAlpha1 = math.cos(alpha1)
    sinAlpha1 = math.sin(alpha1)

    sigma1 = math.atan2(tanU1, cosAlpha1) # angular distance on sphere from equator to P1 along geodesic
    sinAlpha = cosU1*sinAlpha1
    cos2Alpha = 1.0-sinAlpha*sinAlpha

    u2 = cos2Alpha*(a*a-b*b)/(b*b)

    k1 = (math.sqrt(1.0+u2)-1.0)/(math.sqrt(1.0+u2)+1.0)
    A = (1.0+k1*k1/4.0)/(1.0-k1)
    B = k1*(1.0-3.0*k1*k1/8.0)

    sigma = distance/(b*A)
    
    while True:
        cos2Sigmam = math.cos(2.0*sigma1+sigma);
        sinSigma = math.sin(sigma)
        cosSigma = math.cos(sigma)
        
        deltaSigma = B*sinSigma*(cos2Sigmam+.25*B*(cosSigma*(-1.0+2.0*cos2Sigmam*cos2Sigmam)-(B/6.0)*cos2Sigmam*(-3.0+4.0*sinSigma*sinSigma)*(-3.0+4.0*cos2Sigmam*cos2Sigmam)))
        last_sigma = sigma;
        sigma = (distance/(b*A))+deltaSigma;
        if abs(last_sigma-sigma) <= epsilon:
            break

    cos2Sigmam = math.cos(2.0*sigma1+sigma);
            
    phi2 = math.atan2(sinU1*math.cos(sigma)+cosU1*math.sin(sigma)*cosAlpha1,(1-f)*math.sqrt(sinAlpha*sinAlpha+pow(sinU1*math.sin(sigma)-cosU1*math.cos(sigma)*cosAlpha1,2)))
    l = math.atan2(math.sin(sigma)*sinAlpha1,cosU1*math.cos(sigma)-sinU1*math.sin(sigma)*cosAlpha1)
    C = (f/16.0)*cos2Alpha*(4.0+f*(4.0-3.0*cos2Alpha))
    L = l-(1.0-C)*f*sinAlpha*(sigma+C*math.sin(sigma)*(cos2Sigmam+C*math.cos(sigma)*(-1+2.0*cos2Sigmam*cos2Sigmam)));

    lat2 = phi2
    lon2 = lon1 + L
    return lon2,lat2

if __name__ == "__main__":
    while True:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.settimeout(1.0)
        # long, lat = -70.855,43.123
        val = input("Enter long,lat,distance\n")
        long,lat,distance = val
        message = calculateFOV(long,lat,distance)
        message = pickle.dumps(message)
        addr = ("127.0.0.1", 12000)

        start = time.time()
        client_socket.sendto(message, addr)
        try:
            data, server = client_socket.recvfrom(1024)
            end = time.time()
            elapsed = end - start
            print("TIME: ",elapsed)
            data=pickle.loads(data)
            print(data)
        except socket.timeout:
            print('REQUEST TIMED OUT')