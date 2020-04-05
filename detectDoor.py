import math
import matplotlib.pyplot as plt
WallDistanceLimit = 1000    # 1000mm = 1m distance drone would like to keep away from walls

def main():
    # read data from Collected Data as .txt file
    lidar_file = input("Enter Lidar Data File Name: ")
    with open(lidar_file, "r") as filename:
        allLines = filename.readlines();
        headers = allLines[0:3]
        allLines = allLines[3:] # Data from file, minus the headers
        allLines = [x for x in allLines if len(x) > 1]
        polarData = []
        degrees = []
        radii = []
        cartData = []
        cartDataX = []
        cartDataY = []
        for line in allLines:
            line = line.strip("\n").split(" ") # Separate strings into columns
            line = line[0:2] # Remove Quality = 188 column, because we don't really need it
            line = [float(line[0]),float(line[1])]
            degrees.append(float(line[0]))
            radii.append(float(line[1]))
            polarData.append(line)
        polarData.sort(key = lambda x: x[0])
        listOfNones = findGaps(polarData)
        print("ListOfNones: ", listOfNones)
        # print(polarData)

        maxDistanceOnLeft = strafeLeft(polarData)
        print("MaxLeftDistance: " , maxDistanceOnLeft)

        filename.close()
        # print("SortedData: ", newData)
        for point in polarData:
            newCartPoint = convertToCartesian(point)
            cartData.append(newCartPoint) # append cartesian (rectangular) coordinates to cartData list
            cartDataX.append(newCartPoint[0])
            cartDataY.append(newCartPoint[1])


       #  print("CARTESIANDATA:\n", cartData)
       # # print("X: ", x)
       # # print("Y: ", y)
        plt.plot(cartDataX,cartDataY)
        plt.show()

# Converts polar point (theta, r) to cartesian point (x,y)
# Parameter: point: polarPoint as [theta, r]
# Returns [x,y] point equivalent of [theta, r] input
def convertToCartesian(point):
    x = point[1] * math.cos(math.radians(point[0])) # calculate x coordinate of point
    y = point[1] * math.sin(math.radians(point[0])) # calculate y coordinate of point
    return [x,y]

# Determines how many mm(s) the drone can strafe to the Left
# Parameter: points: PolarData as a list of polar points [[degree, radius],...]
# Returns ["Left", maxDistance] if maxDistance to Left > 1500 mm
# Else Returns ["Turn CCW"] for MAVLink to take care of turn.
def strafeLeft(points):
    # Scan For points
    maxDistance = 0
    for i in points:
        if (i[0] >= 260 and i[0] <= 280):
            # print("ANGLE: ", i[0], " DISTANCE: ", i[1])
            if (i[1] > maxDistance):
                maxDistance = i[1]
                # print("MaxDIsNow: ", max)
    maxDistance = maxDistance - WallDistanceLimit
    if (maxDistance > 500):
        return ["Left", maxDistance]
    else:
        return ["Turn CCW"]

# Experimental Code
# Finds the Gaps in the collected data points
# Parameter: points: PolarData as a list of polar points [[degree, radius],...]
# Returns list of Gaps/Holes in terms of which degree 1 to degree 2 [(degree1, degree2),...]
#   Also returns list of ranges where possibleDoors may lie as
#   rangeData = {
#            'range': r,
#            'polarCoordinates': (point1, point2),
#            'cartesianCoordinates': (point1Cart, point2Cart),
#            'rangeDistance': rangeDistance (in inches)
#            }
# rangeData may be used for future development for aligning with doorframes
def findGaps(points):
    degrees = []
    listOfNones = []
    listOfJumps = []
    rangeOfJumps = []

    for p in points:
        # p[0] = degree measured
        # p[1] = distance measured
        # if degree of point is not in list of degrees, add it
        if (int(p[0])) not in degrees:
            degrees.append(int(p[0]))
        # if distance is greater than 2.5 meters treat as Hallway or open space
        if (p[1] > 2500 and int(p[0]) not in listOfJumps):
            listOfJumps.append(p)

    # sort degrees in increasing order 0 to 360
    degrees.sort(key = lambda x: x)
    for d in range(0, len(degrees)-1):
        nxt = degrees[d+1]
        # if next degree in list and current degree are
        # greater than two degrees apart, there is a hole of data
        # between these two points
        if (nxt-degrees[d] > 2):
            listOfNones.append([degrees[d], nxt])

    for a in range(0, 360):
        if a not in degrees:
            # adds a [theta, radius] point to listOfJumps to account for degrees not found
            listOfJumps.append([a, None])

    listOfJumps.sort(key = lambda x: x)

    jumpSequence = [listOfJumps[0][0]]
    for j in range(0, len(listOfJumps)-1):
        nxt = listOfJumps[j+1][0]

        if (nxt-listOfJumps[j][0] <= 5):
            jumpSequence.append(nxt)
        else:
            # find first and last data points in jumpSequence with distance != None
            jumpSequence = [p for p in jumpSequence if isinstance(p, float) == True]

            if (len(jumpSequence) > 1):
                rangeOfJumps.append([jumpSequence[0],jumpSequence[-1]])
            jumpSequence = [nxt]

        # j has reached the end of the list
        if (j == len(listOfJumps)-2):
            # find first and last data points in jumpSequence with distance != None
            jumpSequence = [p for p in jumpSequence if isinstance(p, float) == True]

            if (len(jumpSequence) > 1):
                rangeOfJumps.append([jumpSequence[0],jumpSequence[-1]])

    # print("previous rangeOfJumps: ", rangeOfJumps)
    rangeOfJumps = mergeCircularityRange(rangeOfJumps)

    possibleDoors = []

    # print("New rangeOfJumps: ", rangeOfJumps)
    for r in rangeOfJumps:

        point1index = findPointFromList(r[0], points)[1]
        point2index = findPointFromList(r[1], points)[1]
        if (point1index == 0 and point2index == len(points)-1):
            possibleDoors.append("Full 0 to 360 Range")
        else:
            point1 = points[point1index - 1]
            point2 = points[point2index + 1]
            point1Cart = convertToCartesian(point1)
            point2Cart = convertToCartesian(point2)
            rangeDistance = distanceBetweenTwoPoints(point1Cart, point2Cart)
            # print("Range: ", r)
            # print("PolarPoint1: ", point1)
            # print("PolarPoint2: ", point2)
            # print("CartPoint1: ", point1Cart)
            # print("CartPoint2: ", point2Cart)
            # print("RangeDistance: ", rangeDistance)
            rangeData = {
                        'range': r,
                        'polarCoordinates': (point1, point2),
                        'cartesianCoordinates': (point1Cart, point2Cart),
                        'rangeDistance': rangeDistance
                        }
            possibleDoors.append(rangeData)

    print("Ranges Where No Data Points Found: ", listOfNones)
    print("Possible Doors: ", possibleDoors)
    return listOfNones + possibleDoors

# Merges the first and last pair of angles
#   if they are within 5 degrees of each other
# Parameter: rangeOfJumps, a list of ranges of JumpsInData
#   where distance is greater than 2.5m
# Returns a reformatted merged or non-merged rangeOfJumps
def mergeCircularityRange(rangeOfJumps):
    # lastAngle from rangeOfJumps may be around 360 or 0 deg
    # firstAngle may be around 360 or 0 deg
    lastAngle = rangeOfJumps[-1][1]
    firstAngle = rangeOfJumps[0][0]

    # determine if last and first angle are within 5 deg of each other
    if ((359-lastAngle) + firstAngle <= 5):
        rangeOfJumps = [[rangeOfJumps[-1][0], rangeOfJumps[0][1]]] + rangeOfJumps[1:-1]

        return rangeOfJumps
    return rangeOfJumps

# Converts millimeters to inches
# Parameter: mm: distance in millimeters to be converted
# Returns the converted distance in inches
def convertMMToInch(mm):
    return mm * .0393701

# Finds the distance between two Cartesian coordinates
# Parameters: p1, p2 , both points in format [x1,y1], [x2, y2] respectively
# Returns the distance between the two in inches
def distanceBetweenTwoPoints(p1, p2):
     x1 = p1[0]
     x2 = p2[0]
     y1 = p1[1]
     y2 = p2[1]
     mm = math.sqrt((x2-x1)**2 + (y2-y1)**2)
     inches = convertMMToInch(mm)
     return inches

# Finds the point from list of points
#   where degree matches specified degree
# Parameters: degree = (value between 0 and 360),
#   lst = list of points to search from
# Returns the point from the list matching specified degree
def findPointFromList(degree, lst):
    for p in lst:
        if (p[0] == degree):
            return [p, lst.index(p)]
    return None


if __name__ == '__main__':
    main()
