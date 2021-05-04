#RUN TESTS
#function used to run algorithms and print measured metrics for specified map data
###########################################################################################
from csv_access import*
from search import*
from performance_metrics import*

#finds euclidean distance between two points
#pass in start/end node ids and node csv
def e_distance(start_id, end_id, node_csv):
    #retrieve coordinates from node csv
    #note: [0] = x, [1] = y
    start_coords = getNodeCoords(node_csv, start_id)
    goal_coords = getNodeCoords(node_csv, end_id)

    #calculate euclidean distance
    direct_dist = math.sqrt(((start_coords[0]-goal_coords[0])**2) + ((start_coords[1]-goal_coords[1])**2))

    return direct_dist

#runs both algorithms and all heurisitcs on all three sets of points for given set of map data
#pass in node/edge csv files, map name, and three sets of start/goal pairs
def run_tests(node_csv, edge_csv, map_name, pt_pair1, pt_pair2, pt_pair3):
    #retrieve total number of nodes
    num_nodes = getNumRows(node_csv)
    num_edges = getNumRows(edge_csv)

    #print initial information
    print("------------------------------", map_name, "------------------------------")
    print("Map Size:")
    print("Total number of nodes->", num_nodes)
    print("Total number of edges->", num_nodes)

    #print direct route distances between start and goal (i.e. Euclidean distance)
    print("Direct route distances start/goal distances:")
    #shortest
    short_dist = e_distance(pt_pair1[0], pt_pair1[1], node_csv)
    print("Shortest->", short_dist)
    #middle
    mid_dist = e_distance(pt_pair2[0], pt_pair2[1], node_csv)
    print("Middle->", mid_dist)
    #furthest
    far_dist = e_distance(pt_pair3[0], pt_pair3[1], node_csv)
    print("Furthest->", far_dist)

    #assign neighbors and store in 2D list of tuples w/ edge lengths
    neighbors = assignNeighbors(edge_csv, num_nodes)

    #note: start and goal nodes stored in passed in pt pairs, (start_id, goal_id)
    #1=shortest, 2=middle, 3=furthest

    #RUN UCS (and measure time)
    print("UNIFORM COST SEARCH")
    #shortest distance
    #set-up perfomance tracker object to track performance metrics
    ptracker_ucs = PerformanceTracker()
    
    #START TIME
    ptracker_ucs.startTimer()

    parents_short_ucs = ucs(pt_pair1[0], pt_pair1[1], neighbors, ptracker_ucs)
    path_short_ucs = getPath(pt_pair1[0], pt_pair1[1], parents_short_ucs)

    #END TIME
    time_elapsed = ptracker_ucs.endTimer()

    #display path and metrics
    print("+++++++++++++++++++++++Closest pair of points+++++++++++++++++++++++")
    print("Path found:", path_short_ucs)
    print("________________________________________________")
    findEdgeCount(path_short_ucs, ptracker_ucs)
    print("Edges covered:", ptracker_ucs.getEdgesTraversed())
    findDistCovered(path_short_ucs, neighbors, ptracker_ucs)
    print("Distance covered:", ptracker_ucs.getDistCovered())
    print("Nodes visited:", ptracker_ucs.getNodesVisited())
    findLTurnsTaken(path_short_ucs, node_csv, ptracker_ucs)
    print("Left turns made:", ptracker_ucs.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker and time elapsed var to be reused
    del ptracker_ucs
    del time_elapsed

    #middle distance
    #set-up perfomance tracker object to track performance metrics
    ptracker_ucs = PerformanceTracker()
    
    #START TIME
    ptracker_ucs.startTimer()

    parents_mid_ucs = ucs(pt_pair2[0], pt_pair2[1], neighbors, ptracker_ucs)
    path_mid_ucs = getPath(pt_pair2[0], pt_pair2[1], parents_mid_ucs)

    #END TIME
    time_elapsed = ptracker_ucs.endTimer()

    #display path and metrics
    print("+++++++++++++++++++++++Middle pair of points+++++++++++++++++++++++")
    print("Path found:", path_mid_ucs)
    print("________________________________________________")
    findEdgeCount(path_mid_ucs, ptracker_ucs)
    print("Edges covered:", ptracker_ucs.getEdgesTraversed())
    findDistCovered(path_mid_ucs, neighbors, ptracker_ucs)
    print("Distance covered:", ptracker_ucs.getDistCovered())
    print("Nodes visited:", ptracker_ucs.getNodesVisited())
    findLTurnsTaken(path_mid_ucs, node_csv, ptracker_ucs)
    print("Left turns made:", ptracker_ucs.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker and time elapsed var to be reused
    del ptracker_ucs
    del time_elapsed

    #furthest distance
    #set-up perfomance tracker object to track performance metrics
    ptracker_ucs = PerformanceTracker()

    #START TIME
    ptracker_ucs.startTimer()

    parents_far_ucs = ucs(pt_pair3[0], pt_pair3[1], neighbors, ptracker_ucs)
    path_far_ucs = getPath(pt_pair3[0], pt_pair3[1], parents_far_ucs)

    #END TIME
    time_elapsed = ptracker_ucs.endTimer()

    #display path and metrics
    print("+++++++++++++++++++++++Furthest pair of points+++++++++++++++++++++++")
    print("Path found:", path_far_ucs)
    print("________________________________________________")
    findEdgeCount(path_far_ucs, ptracker_ucs)
    print("Edges covered:", ptracker_ucs.getEdgesTraversed())
    findDistCovered(path_far_ucs, neighbors, ptracker_ucs)
    print("Distance covered:", ptracker_ucs.getDistCovered())
    print("Nodes visited:", ptracker_ucs.getNodesVisited())
    findLTurnsTaken(path_far_ucs, node_csv, ptracker_ucs)
    print("Left turns made:", ptracker_ucs.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker and time var
    del ptracker_ucs
    del time_elapsed

    #################################################################################
    #set-up perfomance tracker objects to track performance metrics
    ptracker_astar1 = PerformanceTracker()
    ptracker_astar2 = PerformanceTracker()
    ptracker_astar3 = PerformanceTracker()

    #RUN A STAR (and measure time)
    print("A STAR:")
    #shortest distance
    print("+++++++++++++++++++++++Closest pair of points+++++++++++++++++++++++")
    #HEURISTIC 1
    print("Heuristic 1 Test: Euclidean Distance")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()

    #START TIME
    ptracker_astar.startTimer()

    parents_short_astar = astar(pt_pair1[0], pt_pair1[1], neighbors, node_csv, 1, ptracker_astar)
    path_short_astar = getPath(pt_pair1[0], pt_pair1[1], parents_short_astar)

    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_short_astar)
    print("________________________________________________")
    findEdgeCount(path_short_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_short_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_short_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_short_astar
    del path_short_astar

    #HEURISTIC 2
    print("Heurisitic 2 Test: Added Non-Left Turn Bonus")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()

    #START TIME
    ptracker_astar.startTimer()

    parents_short_astar = astar(pt_pair1[0], pt_pair1[1], neighbors, node_csv, 2, ptracker_astar)
    path_short_astar = getPath(pt_pair1[0], pt_pair1[1], parents_short_astar)

    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_short_astar)
    print("________________________________________________")
    findEdgeCount(path_short_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_short_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_short_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_short_astar
    del path_short_astar

    #HEURISTIC 3
    print("Heurisitic 3: Added Goal Direction Bonus")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()
    
    #START TIME
    ptracker_astar.startTimer()

    parents_short_astar = astar(pt_pair1[0], pt_pair1[1], neighbors, node_csv, 3, ptracker_astar)
    path_short_astar = getPath(pt_pair1[0], pt_pair1[1], parents_short_astar)

    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_short_astar)
    print("________________________________________________")
    findEdgeCount(path_short_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_short_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_short_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_short_astar
    del path_short_astar

    #middle distance
    print("+++++++++++++++++++++++Middle pair of points+++++++++++++++++++++++")
    #HEURISTIC 1
    print("Heuristic 1 Test: Euclidean Distance")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()
    
    #START TIME
    ptracker_astar.startTimer()

    parents_mid_astar = astar(pt_pair2[0], pt_pair2[1], neighbors, node_csv, 1, ptracker_astar)
    path_mid_astar = getPath(pt_pair2[0], pt_pair2[1], parents_mid_astar)
    
    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_mid_astar)
    print("________________________________________________")
    findEdgeCount(path_mid_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_mid_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_mid_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_mid_astar
    del path_mid_astar

    #HEURISTIC 2
    print("Heuristic 2 Test: Left Turn Penalty")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()
    
    #START TIME
    ptracker_astar.startTimer()

    parents_mid_astar = astar(pt_pair2[0], pt_pair2[1], neighbors, node_csv, 2, ptracker_astar)
    path_mid_astar = getPath(pt_pair2[0], pt_pair2[1], parents_mid_astar)
    
    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_mid_astar)
    print("________________________________________________")
    findEdgeCount(path_mid_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_mid_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_mid_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_mid_astar
    del path_mid_astar

    #HEURISTIC 3
    print("Heurisitic 3: Added Goal Direction Bonus")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()

    #START TIME
    ptracker_astar.startTimer()

    parents_mid_astar = astar(pt_pair2[0], pt_pair2[1], neighbors, node_csv, 3, ptracker_astar)
    path_mid_astar = getPath(pt_pair2[0], pt_pair2[1], parents_mid_astar)

    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_mid_astar)
    print("________________________________________________")
    findEdgeCount(path_mid_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_mid_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_mid_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_mid_astar
    del path_mid_astar

    #furthest distance
    print("+++++++++++++++++++++++Furthest pair of points+++++++++++++++++++++++")
    #HEURISTIC 1
    print("Heuristic 1 Test: Manhattan Distance")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()

    #START TIME
    ptracker_astar.startTimer()
    
    parents_far_astar = astar(pt_pair3[0], pt_pair3[1], neighbors, node_csv, 1, ptracker_astar)
    path_far_astar = getPath(pt_pair3[0], pt_pair3[1], parents_far_astar)

    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_far_astar)
    print("________________________________________________")
    findEdgeCount(path_far_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_far_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_far_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_far_astar
    del path_far_astar

    #HEURISTIC 2
    print("Heuristic 2 Test: Left Turn Penalty")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()

    #START TIME
    ptracker_astar.startTimer()

    parents_far_astar = astar(pt_pair3[0], pt_pair3[1], neighbors, node_csv, 2, ptracker_astar)
    path_far_astar = getPath(pt_pair3[0], pt_pair3[1], parents_far_astar)

    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_far_astar)
    print("________________________________________________")
    findEdgeCount(path_far_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_far_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_far_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_far_astar
    del path_far_astar

    #HEURISTIC 3
    print("Heurisitic 3: Added Goal Direction Bonus")
    #set-up perfomance tracker object to track performance metrics
    ptracker_astar = PerformanceTracker()

    #START TIME
    ptracker_astar.startTimer()

    parents_far_astar = astar(pt_pair3[0], pt_pair3[1], neighbors, node_csv, 3, ptracker_astar)
    path_far_astar = getPath(pt_pair3[0], pt_pair3[1], parents_far_astar)

    #END TIME
    time_elapsed = ptracker_astar.endTimer()

    #display path and metrics
    print("Path found:", path_far_astar)
    print("________________________________________________")
    findEdgeCount(path_far_astar, ptracker_astar)
    print("Edges covered:", ptracker_astar.getEdgesTraversed())
    findDistCovered(path_far_astar, neighbors, ptracker_astar)
    print("Distance covered:", ptracker_astar.getDistCovered())
    print("Nodes visited:", ptracker_astar.getNodesVisited())
    findLTurnsTaken(path_far_astar, node_csv, ptracker_astar)
    print("Left turns made:", ptracker_astar.getLTurnsMade())
    print(f"Total time taken: {time_elapsed:0.5f} seconds")
    print("________________________________________________")
    print("\n")

    #delete performance tracker, and time elapsed/path variables to be reused
    del ptracker_astar
    del time_elapsed
    del parents_far_astar
    del path_far_astar
