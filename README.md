# CMPM-146-A-B-Directional-Search

- Code starts at top by finding source and destination Box by iterating through the Navmesh and checking if any of the coordinates of the navmesh boxes contain the two points
- Goes to the bottom calling biDirectionalAStar() which keeps calling queueAdjaneciesForward() Until one of the boxes queue elements being heappopped is already in the oppoisite prevDictionary
    -(There are two prevDictionaries which track the parent of each box)
 - queueAjdacenciesForward() -> getCost -> getEntryPoint() -> backTrackToStart()
 - backTrackToStart() gets the path of boxes that either lead back to start or destination point depending on currentDestination argument passed in
 - getEntryPoint() essentially utilizes the path of boxes to determine the shortest path to the box passed in, then the function returns that point
 - getCost() compares the cost(euclidian Distance to an adjacent box + each that points euclidian distance to the destination point depending on currentDestination argument passed in
 - To Run Code type in terminal: python src/nm_interactive.py test_image.png test_image.png.mesh.pickle 2
