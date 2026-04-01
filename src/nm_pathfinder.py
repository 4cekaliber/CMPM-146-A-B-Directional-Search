#Bi-Directional A* Implementation by Elijah M.
"""
How it works(For future reference):
- Code starts at top by finding source and destination Box by iterating through Navmesh and checking if any of the navmesh boxes contain the two points
- Goes to the bottom calling biDirectionalAStar() which keeps calling queueAdjaneciesForward() Until one of the boxes queue elements being heappopped is already in the oppoisite prevDictionary
    -(There are two prevDictionaries which track the parent of each box)
 - queueAjdacenciesForward() -> getCost -> getEntryPoint() -> backTrackToStart()
 - backTrackToStart() gets the path of boxes that either lead back to start or destination point depending on currentDestination argument passed in
 - getEntryPoint() essentially utilizes the path of boxes to determine the shortest path to the box passed in, then the function returns that point
 - getCost() compares the cost(euclidian Distance to an adjacent box + each that points euclidian distance to the destination point depending on currentDestination argument passed in
"""
from math import sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    sourceBox = None
    destinationBox = None
    cost_so_far_forward = {}
    cost_so_far_backward = {}
    queue = [] # node queue
    discoveredBoxes = [] # boxes in which the adjancies have already been queued
    path = [] #point path from start to destination
    forward_prev = {}#key = box, value = key's parent from source path
    backward_prev = {}#key = box, value = key's parent from destination path
    boxPath = [] #list of box path from start to destination
    #note: box = (y1,y2,x1,x2)

    for box in mesh['boxes']:
        if source_point[1] < box[3] and source_point[1] > box[2]:
            if source_point[0] < box[1] and source_point[0] > box[0]:
                sourceBox = box
                cost_so_far_forward[sourceBox] = 0
                heappush(queue,(0,sourceBox,destination_point))
                forward_prev[source_point] = None
    for destBox in mesh['boxes']:
        if destination_point[1] < destBox[3] and destination_point[1] > destBox[2]:
            if destination_point[0] <destBox[1] and destination_point[0] > destBox[0]:
                destinationBox = destBox
                cost_so_far_backward[destinationBox] = 0
                heappush(queue,(0,destinationBox,source_point))
                backward_prev[destinationBox] = None

    if sourceBox == None or destinationBox == None:
        print("ERROR: PATH DOES NOT EXIST")
        
    def backTrackToStart(lastBox,currentDestination):
        #makes a list of boxes from the start to destination
        if currentDestination == source_point:
            prevDict = backward_prev
            goalBox = destinationBox
        elif currentDestination == destination_point:
            prevDict = forward_prev
            goalBox = sourceBox
        boxList = []
        boxList.insert(0,lastBox)
        prevBox = None
        while prevBox != goalBox:
            currentBox = boxList[0]
            if currentBox == goalBox:
                return boxList
            prevBox = prevDict[currentBox]
            boxList.insert(0,prevBox)
        return boxList
    
    def getEntryPoint(endBox,currentDestination):
        #gets point that entered endBox
        boxPath = backTrackToStart(endBox,currentDestination)
        samplePath = []
        if currentDestination == source_point:
            samplePath.append(destination_point)
        elif currentDestination == destination_point:
            samplePath.append(source_point)
        
        i = 0
        while i  < len(boxPath)-1:
            currentBox = boxPath[i]
            nextBox = boxPath[i + 1]
            prevPoint = samplePath[len(samplePath) - 1]
            nextPoint = [0,0]
            minX = max(currentBox[2],nextBox[2])
            maxX = min(currentBox[3],nextBox[3])
            minY = max(currentBox[0],nextBox[0])
            maxY = min(currentBox[1],nextBox[1])
            xRange = [minX,maxX]
            yRange = [minY,maxY]
            i += 1

            if minX == maxX:
                nextPoint[1] = minX
            elif prevPoint[1] < minX:
                nextPoint[1] = minX
            elif prevPoint[1] > minX and prevPoint[1] < maxX :
                nextPoint[1] = prevPoint[1]
            elif prevPoint[1] > maxX:
                nextPoint[1] = maxX
            elif prevPoint[1] == minX:
                nextPoint[1] = minX
            elif prevPoint[1] == maxX:
                nextPoint[1] = maxX

            if minY == maxY:
                nextPoint[0] = minY
            elif prevPoint[0] < minY:
                nextPoint[0] = minY
            elif prevPoint[0] > minY and prevPoint[0] < maxY :
                nextPoint[0] = prevPoint[0]
            elif prevPoint[0] > maxY:
                nextPoint[0] = maxY
            elif prevPoint[0] == minY:
                nextPoint[0] = minY
            elif prevPoint[0] == maxY:
                nextPoint[0] = maxY
            
            if nextPoint != destination_point or nextPoint != source_point:
                samplePath.append(nextPoint)
        return samplePath[len(samplePath) - 1]

    def getCost(firstBox,secondBox,currentDestination):
        #returns distance from entry point of box to entry point of secondBox
        firstPoint = getEntryPoint(firstBox,currentDestination)
        if currentDestination == source_point:
            goalPoint = source_point
        elif currentDestination == destination_point:
            goalPoint = destination_point
        secondPoint = [0,0]
        minX = max(firstBox[2],secondBox[2])
        maxX = min(firstBox[3],secondBox[3])
        minY = max(firstBox[0],secondBox[0])
        maxY = min(firstBox[1],secondBox[1])
        xRange = [minX,maxX]
        yRange = [minY,maxY]

        if minX == maxX:
            secondPoint[1] = minX
        elif firstPoint[1] < minX:
            secondPoint[1] = minX
        elif firstPoint[1] > minX and firstPoint[1] < maxX :
            secondPoint[1] = firstPoint[1]
        elif firstPoint[1] > maxX:
            secondPoint[1] = maxX
        elif firstPoint[1] == minX:
            secondPoint[1] = minX
        elif firstPoint[1] == maxX:
            secondPoint[1] = maxX

        if minY == maxY:
            secondPoint[0] = minY
        elif firstPoint[0] < minY:
            secondPoint[0] = minY
        elif firstPoint[0] > minY and firstPoint[0] < maxY :
            secondPoint[0] = firstPoint[0]
        elif firstPoint[0] > maxY:
            secondPoint[0] = maxY
        elif firstPoint[0] == minY:
            secondPoint[0] = minY
        elif firstPoint[0] == maxY:
            secondPoint[0] = maxY

        
        entryToExitDist = distFormula(firstPoint,secondPoint)
        heuristic = distFormula(secondPoint,goalPoint)
        totalEstimate = entryToExitDist + heuristic
        return entryToExitDist, heuristic

    def queueBoxesForward(currentBox,currentDestination):
        for boxContact in mesh["adj"][currentBox]:
        #for each touching box, find it's adjacencies
            if currentDestination == source_point:
                prevDict = backward_prev
                costDict = cost_so_far_backward
            elif currentDestination == destination_point:
                prevDict = forward_prev
                costDict = cost_so_far_forward
            nextBoxCost, heuristic = getCost(currentBox,boxContact,currentDestination)
            newCost = costDict[currentBox] + nextBoxCost
            
            if boxContact not in prevDict or newCost < costDict[boxContact] :
                
                costDict[boxContact] = newCost
                totalEstimate = newCost + heuristic
                heappush(queue,(totalEstimate,boxContact,currentDestination))
                prevDict[boxContact] = currentBox

    def biDirectionalAStar():
        while len(queue) > 0:
        #when there is at least 1 item in queue
            currentPriority,currentBox,currentDestination = heappop(queue)
            if currentDestination == source_point:
                oppositeDict = forward_prev
            if currentDestination == destination_point:
                oppositeDict = backward_prev
            discoveredBoxes.append(currentBox)
            
            #when a box from the opposite side appers in the opposite dictionary, make path from middle to start, then a path from middle to end
            if currentBox in oppositeDict:
            #if the current node is destination box then break
                fromSourcePath = makePath(currentBox,source_point)
                for element2 in fromSourcePath:
                    path.append(element2)
                fromDestinationPath = makePath(currentBox,destination_point)
                fromDestinationPath.reverse()
                for element in fromDestinationPath:
                    path.append(element)
                
                return
            queueBoxesForward(currentBox,currentDestination)    
    
    def makePath(commonBox,currentDestination):
        #makes full path of points from start to destination point
        tempPath = []
        if currentDestination == source_point and destination_point not in tempPath:
            tempPath.append(destination_point)
        elif currentDestination == destination_point and source_point not in tempPath:
            tempPath.append(source_point)
        boxPath = backTrackToStart(commonBox,currentDestination)

        i = 0
        while i  < len(boxPath)-1:
            currentBox = boxPath[i]
            nextBox = boxPath[i + 1]
            prevPoint = tempPath[len(tempPath) - 1]
            nextPoint = [0,0]
            minX = max(currentBox[2],nextBox[2])
            maxX = min(currentBox[3],nextBox[3])
            minY = max(currentBox[0],nextBox[0])
            maxY = min(currentBox[1],nextBox[1])
            xRange = [minX,maxX]
            yRange = [minY,maxY]
            i += 1

            if minX == maxX:
                nextPoint[1] = minX
            elif prevPoint[1] < minX:
                nextPoint[1] = minX
            elif prevPoint[1] > minX and prevPoint[1] < maxX :
                nextPoint[1] = prevPoint[1]
            elif prevPoint[1] > maxX:
                nextPoint[1] = maxX
            elif prevPoint[1] == minX:
                nextPoint[1] = minX
            elif prevPoint[1] == maxX:
                nextPoint[1] = maxX

            if minY == maxY:
                nextPoint[0] = minY
            elif prevPoint[0] < minY:
                nextPoint[0] = minY
            elif prevPoint[0] > minY and prevPoint[0] < maxY :
                nextPoint[0] = prevPoint[0]
            elif prevPoint[0] > maxY:
                nextPoint[0] = maxY
            elif prevPoint[0] == minY:
                nextPoint[0] = minY
            elif prevPoint[0] == maxY:
                nextPoint[0] = maxY
            
            if nextPoint != destination_point or nextPoint != source_point:
                tempPath.append(nextPoint)

        return tempPath

    def distFormula(pointOne,pointTwo):
        return sqrt((pointOne[0]-pointTwo[0])**2+(pointOne[1]-pointTwo[1])**2)
    
    
    biDirectionalAStar()
    
    return path, discoveredBoxes #try to find what comes out from the boxes dict if you use destination box as keys