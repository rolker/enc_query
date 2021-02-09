
from enc_query import enc_query
from tkinter import * 
# import enc_query
from osgeo import ogr
canvas_width = 1000
canvas_height = 1000

root = Tk()
root.title("Points")
canvas = Canvas(root,
           width=canvas_width,
           height=canvas_height)
origX = canvas.xview()[0]
origY = canvas.yview()[0]
def center(event=None):
    canvas.xview_moveto(origX)
    canvas.yview_moveto(origY)

def zoom_in(event=None):
    factor = 2
    x = root.winfo_pointerx()
    y = root.winfo_pointery()
    abs_coord_x = root.winfo_pointerx() - root.winfo_rootx()
    abs_coord_y = root.winfo_pointery() - root.winfo_rooty()
    canvas.scale(ALL, abs_coord_x,abs_coord_y, factor, factor)
def zoom_out(event=None):
    factor = .5
    x = root.winfo_pointerx()
    y = root.winfo_pointery()
    abs_coord_x = root.winfo_pointerx() - root.winfo_rootx()
    abs_coord_y = root.winfo_pointery() - root.winfo_rooty()
    canvas.scale(ALL, abs_coord_x,abs_coord_y, factor, factor)
def latlong_to_coord(latlong):
    x,y = latlong
    x_i = (( 43.211667 - x )/ 0.248334) * canvas_width
    y_i = ((-70.9783333 - y  )/ -0.263333 * canvas_height)
    return (x_i,y_i)
def placeBouy(bouy):
    r = 3
    name = bouy[0]
    x,y = latlong_to_coord(bouy[1])
    x1, y1 = (x - r), (y - r)
    x2, y2 = (x + r), (y + r)
    canvas.create_oval(x1, y1, x2, y2, fill='green')
    canvas.create_text(x, y+r+2, text=name, font=("Courier",10))
def points_from_geometry(geom):
    points = []
    # for i in range(geom.GetGeometryCount()):
    geomRef = geom.GetGeometryRef(0)
    print("PC:", geomRef.GetPointCount())
    for i in range(geomRef.GetPointCount()):
        # print(i)
        point = latlong_to_coord((geomRef.GetPoint(i)[1],geomRef.GetPoint(i)[0]))
        points.append(point)
    return points
def getBox():
    boxCoords = ogr.Geometry(ogr.wkbLinearRing)
    boxCoords.AddPoint(-70.863,43.123)
    boxCoords.AddPoint(-70.855,43.123)
    boxCoords.AddPoint(-70.855,43.12)
    boxCoords.AddPoint(-70.863,43.12)
    # boxCoords.AddPoint(-70.863,43.123)
    box = ogr.Geometry(ogr.wkbPolygon)
    box.AddGeometry(boxCoords)
    return box  
enc = enc_query('/home/thomas/Downloads/ENC_ROOT/US5NH01M/TestFile/US5NH01M.000')
bouys = enc.getBouyLat()

for bouy in bouys:
    placeBouy(bouy)

# points = [100, 100, 100, 200, 200, 200, 200,100]
box = enc.calculateFOV()

points = points_from_geometry(box)
print("POINTS",points)

canvas.create_polygon(points, outline='#f11',fill='red', width=2)
canvas.pack(expand=YES)
root.bind("<Up>", zoom_in)
root.bind("<Down>", zoom_out)
root.bind("<space>", center)
# canvas.create_polygon(10,10,20,20,30,30,fill='blue')
canvas.bind('<ButtonPress-1>', lambda event: canvas.scan_mark(event.x, event.y))
canvas.bind("<B1-Motion>", lambda event: canvas.scan_dragto(event.x, event.y, gain=1))

mainloop()