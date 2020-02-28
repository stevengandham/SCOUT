import math
import matplotlib.pyplot as plt
WallDistanceLimit = 1000    # 1000mm = 1m distance drone would like to keep away from walls

def main():
    # read data from Collected Data
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
        # print(listOfNones)
        # print(polarData)

        maxDistanceOnLeft = strafeLeft(polarData)
        print("MaxLeftDistance: " , maxDistanceOnLeft)

        filename.close()
        # print("SortedData: ", newData)
        for point in polarData:
            x = point[1] * math.cos(math.radians(point[0])) # calculate x coordinate of point
            y = point[1] * math.sin(math.radians(point[0])) # calculate y coordinate of point
            cartData.append([x,y]) # append cartesian (rectangular) coordinates to cartData list
            cartDataX.append(x)
            cartDataY.append(y)
       #
       #  print("CARTESIANDATA:\n", cartData)
       # # print("X: ", x)
       # # print("Y: ", y)
        plt.plot(cartDataX,cartDataY)
        plt.show()

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
# Returns list of Gaps in terms of which degree 1 to degree 2 [(degree1, degree2),...]
def findGaps(points):
    degrees = []
    listOfNones = []
    listOfJumps = []

    for p in points:
        if (int(p[0])) not in degrees:
            degrees.append(int(p[0]))
        if (p[1] > 2500):
            listOfJumps.append(p[0])
    #print("ListOfJumps: ", listOfJumps)
    degrees.sort(key = lambda x: x)
    for d in range(0, len(degrees)-1):
        nxt = degrees[d+1]
        if (nxt-degrees[d] > 2):
            listOfNones.append([degrees[d], nxt])

    jumpSequence = [listOfJumps[0]]
    for j in range(0, len(listOfJumps)-1):
        nxt = listOfJumps[j+1]

        if (nxt-listOfJumps[j] < 2):
            jumpSequence.append(nxt)
        else:
            listOfJumps.append(jumpSequence)
            jumpSequence = [nxt]
    return listOfNones

# Finds the distance between two points
def distanceBetweenTwoPoints(p1, p2):
     x1 = p1[0]
     x2 = p2[0]
     y1 = p1[1]
     y2 = p2[1]
     d = math.sqr((x2-x1)**2 + (y2-y1)**2)
     return d


if __name__ == '__main__':
    main()
