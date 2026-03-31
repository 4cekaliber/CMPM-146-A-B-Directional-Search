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
    cost_so_far = {}
    queue = [] # node queue
    discoveredBoxes = [] # boxes in which the adjancies have already been queued
    #note: box = (y1,y2,x1,x2)
    for box in mesh['boxes']:
        if source_point[1] < box[3] and source_point[1] > box[2]:
            if source_point[0] < box[1] and source_point[0] > box[0]:
                sourceBox = box
                cost_so_far[sourceBox] = 0
                heappush(queue,(0,sourceBox))
    for destBox in mesh['boxes']:
        if destination_point[1] < destBox[3] and destination_point[1] > destBox[2]:
            if destination_point[0] <destBox[1] and destination_point[0] > destBox[0]:
                destinationBox = destBox

    def backTrack(lastBox):
        #makes a list of boxes from the destination to start
        boxList = []
        boxList.insert(0,lastBox)
        prevBox = None
        while prevBox != sourceBox:
            currentBox = boxList[0]
            if currentBox == sourceBox:
                return boxList
            prevBox = boxes[currentBox]
            boxList.insert(0,prevBox)
        return boxList
    
    def getBoxPrevPoint(endBox):
        #gets point that entered endBox
        boxPath = backTrack(endBox)
        samplePath = []
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
            """
            print(f"xrange: {xRange}")
            print(f"yrange: {yRange}")
            print(f"next Point: {nextPoint}")
            """
            samplePath.append(nextPoint)
        return samplePath[len(samplePath) - 1]

    def getCost(firstBox,secondBox):
        #returns distance from entry point of box to entry point of secondBox
        firstPoint = getBoxPrevPoint(firstBox)
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
        heuristic = distFormula(secondPoint,destination_point)
        totalEstimate = entryToExitDist + heuristic
        return entryToExitDist, heuristic

    def queueAdjacencies(currentBox):
        for boxContact in mesh["adj"][currentBox]:
        #for each touching box, find it's adjacencies
            nextBoxCost, heuristic = getCost(currentBox,boxContact)
            newCost = cost_so_far[currentBox] + nextBoxCost
            adjBoxInDiscovered = False
            adjBoxInQueue = False
            """
            for each2 in discovered:
                if boxContact == each2:
                    adjBoxInDiscovered = True
                    break

            for adjBox in queue:
                if boxContact == adjBox:
                    adjBoxInQueue = True
                    break

            if adjBoxInDiscovered or adjBoxInQueue:
                continue
            """
            """
            print(f"xrange: {xRange}")
            print(f"yrange: {yRange}")
            print(f"next Point: {nextPoint}")
            """
            if boxContact not in boxes or newCost < cost_so_far[boxContact] :
                if boxContact == (673, 768, 450, 769):
                    print(f'''2ND to Last Bottom Box Cost: {newCost} 
{cost_so_far[currentBox]} + {getCost(currentBox,boxContact)}''')
                
                cost_so_far[boxContact] = newCost
                priority = newCost + heuristic
                heappush(queue,(priority,boxContact))
                boxes[boxContact] = currentBox

    def checkDiscovered(currentBox):
        #checks if box exists within discovered list
        boxDiscovered = False
        for each in discoveredBoxes:
        #if current box already exists in disovered list then break
            if currentBox == each:
                boxDiscovered = True
                break
        if boxDiscovered == True:
            return
        discoveredBoxes.append(currentBox)
    def bfsFunction():
        while len(queue) > 0:
        #when there is at least 1 item in queue
            currentBox = heappop(queue)[1]
            discoveredBoxes.append(currentBox)
            if currentBox == destinationBox:
            #if the current node is destination box then break
                print("FOUND PATH!")
                makePath()
                return
            
            #checkDiscovered(currentBox[1])
            queueAdjacencies(currentBox)
        #!!!Issue: if leave line 192 and 200 as is then disjktra's work but if you switch it so that 192 is queue.pop()[1] instead to allow makePath to work then it just does the first path it finds not the most efficient 
    
    def makePath():
        #makes full path of points from start to destination point
        boxPath = backTrack(destinationBox)
        path.append(source_point)
        

        i = 0
        while i  < len(boxPath)-1:
            currentBox = boxPath[i]
            nextBox = boxPath[i + 1]
            prevPoint = path[len(path) - 1]
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
            """
            print(f"xrange: {xRange}")
            print(f"yrange: {yRange}")
            print(f"next Point: {nextPoint}")
            """
            path.append(nextPoint)
        path.append(destination_point)
        """pseudo code
        iterate through boxPath from start to end
            - use for loop method by just doing "for len of boxPath" then just do ++ at end of each loop
            - get x range and y range for the next connection point
            - I think keep rest of code that assigns nextPoint??
            - Actually no change source_point parts to the previous point that was created
        """

    def distFormula(pointOne,pointTwo):
        return sqrt((pointOne[0]-pointTwo[0])**2+(pointOne[1]-pointTwo[1])**2)
    
    path = [] #point path from start to destination
    boxes = {}#came from dict
    boxPath = [] #list of box path from start to destination
    bfsFunction()
    print(f'''Source Point: {source_point}
        Box: {sourceBox}
        adjacent Boxes: {mesh["adj"][sourceBox]}''')
    print(f'''Destination Point: {destination_point}
        Box: {destinationBox}''') 
    #boxPath = backTrack(destinationBox)
    #print(f"boxes path list: {boxPath}")
    print(f"boxes path len: {len(boxPath)}")
    """
    print(f"dist between start and end: {distFormula(source_point,destination_point)}")
    print(f'''cost between start and finish: {getCost(sourceBox,destinationBox)}''')
    print(f'''3RD Top Left Box Cost: {cost_so_far[(193, 385, 0, 129)]}
    2ND Top Left Box Cost: {cost_so_far[(385, 577, 0, 226)]}
    2ND to last bottom Box Cost: {cost_so_far[(673, 768, 450, 769)]}''')
    """
    return path, discoveredBoxes #try to find what comes out from the boxes dict if you use destination box as keys