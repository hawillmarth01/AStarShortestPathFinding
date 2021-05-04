#SEARCH HELPER
#functions used to perform search algorithms

#algorithms included:
#uniform cost search
#a star
###########################################################################################
from queue import PriorityQueue
from csv_access import*
from performance_metrics import*
import math

#displays resulting path
#pass in start/goal node ids and parent dictionary, prints node ids of path
def displayPath(start_node, goal_node, parents):
    #build path by working backward from goal
    #initialize
    at_start = False #keeps track of whether or not start node found
    curr_node = goal_node #start at goal node
    path = [] #stores final path
    path.append(goal_node) #add goal node

    #repeat until goal found
    while(not at_start):
        next_node = parents[curr_node]
        #check if start node
        if(next_node == start_node): #exit loop
            at_start = True
            #break
        else: #insert next node at front of list
            path.insert(0, next_node)
            #move node
            curr_node = next_node
    
    #print path
    print("START:", start_node, "-> ", end="")
    for node in path:
        if node == goal_node:
            print("GOAL:", node)
        else:
            print(node, "-> ", end="")

#forms resulting path, returns list of nodes
#pass in start/goal node ids and parent dictionary
def getPath(start_node, goal_node, parents):
    #build path by working backward from goal
    #initialize
    at_start = False #keeps track of whether or not start node found
    curr_node = goal_node #start at goal node
    path = [] #stores final path
    path.append(goal_node) #add goal node

    #repeat until goal found
    while(not at_start):
        next_node = parents[curr_node]
        #check if start node
        if(next_node == start_node): #exit loop
            at_start = True
            #break
        else: #insert next node at front of list
            path.insert(0, next_node)
            #move node
            curr_node = next_node
    
    return path

#counts number of roads used in path, (i.e. edges traversed), stores in performance tracker
#pass in path (list of nodes) and performace tracker object
def findEdgeCount(path, performance_tracker):
    edges = len(path) - 1
    performance_tracker.setEdges(edges)

#finds total distance covered by taking resulting path, stores in performance trackerß
#pass in path (list of nodes) and performance tracker object
def findDistCovered(path, neighbors, performance_tracker):
    distance_covered = 0.0
    #iterate thru path, add distance of each road segment
    for i in range(len(path)-1): #dont need to iterate on goal node, all edges have already been covered
        curr_node_id = path[i]
        next_node_id = path[i+1] #won't go out of bounds because stops at 2nd to last node

        #l2 distance between nodes stored in neighbors list
        curr_neighbors = neighbors[curr_node_id] #neighbors of curr

        #find curr->next tuple in curr neighbors list
        for neighbor in curr_neighbors:
            if neighbor[0] == next_node_id:
                segment_dist = neighbor[1] #second element in tuple
                #add segment distance to distance covered
                distance_covered += segment_dist
                break


    #distance calculated, set dist_covered w/ performance_tracker object
    performance_tracker.setDistCovered(distance_covered)

#counts number of left turns made in path, stores in performance tracker
#pass in path (list of nodes), node csv file, and performace tracker object
def findLTurnsTaken(path, node_csv, performance_tracker):
    left_turns = 0
    #iterate thru path, incrementing left turn count when left turn encountered
    #start at 2nd element since use previous node. stop before last element since use next node.
    #by skipping first and last indexes, still covers all road segments
    for i in range(1,len(path)-2):
        #retrieve coordinates of all needed nodes (i.e. current node along with previous and next nodes)
        #note: [0] = x, [1] = y
        prev_coords = getNodeCoords(node_csv, path[i-1])
        curr_coords = getNodeCoords(node_csv, path[i])
        next_coords = getNodeCoords(node_csv, path[i+1])
        
        #form vectors
        #vector to store current direction of movement
        curr_dir = ((curr_coords[0] - prev_coords[0]), (curr_coords[1] - prev_coords[1]))
        #vector to store direction that would be taken if given neighbor picked
        next_dir = ((next_coords[0] - curr_coords[0]), (next_coords[1] - curr_coords[1]))

        #calculate counterclockwise angle between vectors
        #note: + angle = counterclockwise from curr direction
        #      - angle = clockwise from curr direction
        elem1 = det(curr_dir, next_dir) 
        elem2 = dot(curr_dir, next_dir) 
        rad_angle = math.atan2(elem1, elem2) 

        #convert to degrees
        deg_angle = rad_angle * (180 / math.pi)

        #if angle = positive, requires left turn off current road
        if deg_angle > 0: #left turn, increment count
            left_turns += 1
        
    #update performance tracker
    performance_tracker.setLTurns(left_turns)

    

#UNIFORM COST SEARCH###########################################################################
#based off of https://github.com/DeepakKarishetti/Uniform-cost-search/blob/master/find_route.py
#performs uniform cost search, finds minimum cost path
#pass in ids of start/goal nodes and 2D list of node neighbors
#returns dictionary of node parents in resulting path
def ucs(start_node, goal_node, neighbors, performance_tracker):
    #set-up data structures
    parents = {} #dictionary to store minimum cost path to goal (i.e. parents of nodes in path)
    visited = set() #stores all visited nodes
    PQ = PriorityQueue() #priority queue to store fringe

    #initialize priority queue
    #note: elements of PQ = (cost, vertex)
    PQ.put((0, start_node)) #add start node, cost is 0

    #repeat until find goal
    while(PQ):
        #print("still looking...")
        curr_cost, curr_node = PQ.get()
        #note: "node visit" = everytime a node is popped off of queue, i.e. path is explored
        performance_tracker.addNodeVisit()

        #check if node has been visited
        if curr_node not in visited:
            #print("explore new node...")
            #visiting new node, increment node count
            #performance_tracker.addNodeVisit()

            visited.add(curr_node) #has now been visited, add to set

            #check if node is the goal node
            if curr_node == goal_node: #is goal, return parents dict
                return parents 
            #else:

            #neighbor tuple list for current node
            curr_neighbors = neighbors[curr_node]

            #iterate thru neighbors of current node
            #note: next index layer = tuple (i.e. 'neighbor'), loop is iterating thru tuples
            #tuple= (neighbor id, l2 distance)
            for neighbor_tuple in curr_neighbors:
                #print("going through neighbors...")
                curr_neighbor = neighbor_tuple[0]

                #check if neighbor has been visited
                if curr_neighbor not in visited: 
                    #calculate new cost
                    node_neighbor_dist = neighbor_tuple[1]
                    total_cost = curr_cost + node_neighbor_dist

                    #update data structures
                    # print("adding neighbor:",curr_neighbor, ", Cost:", total_cost) #CHECK
                    PQ.put((total_cost, curr_neighbor))
                    parents[curr_neighbor] = curr_node ##CHECK

#A STAR SEARCH#######################################################################################
#----------------------------------------------------------------------------------------------------
#heuristic 1: [euclidean distance] sqrt((x1 - y1)^2 + (x2 - y2)^2)
#note: is underestimate, i.e. admissible
#pass in neighbor/goal nodes and node csv filename
def heuristic1(neighbor_node, goal_node, node_filename):
    #retrieve coordinates from node csv
    #note: [0] = x, [1] = y
    goal_coords = getNodeCoords(node_filename, goal_node)
    neighbor_coords = getNodeCoords(node_filename, neighbor_node)

    #calculate euclidean distance
    result = math.sqrt(((neighbor_coords[0]-goal_coords[0])**2) + ((neighbor_coords[1]-goal_coords[1])**2))

    return result
#----------------------------------------------------------------------------------------------------
#helper functions for heuristic 2
#finds magnitude of a vector
#pass in x, y components of vector, returns magnitude
def magnitude(vect):
    mag = math.sqrt((vect[0] * vect[0]) + (vect[1] * vect[1]))
    return mag

#finds dot product btwn two vectors
#pass in two vectors as tuples, returns dot product
def dot(vect1, vect2):
    dot = (vect1[0] * vect2[0]) + (vect1[1] * vect2[1])
    return dot

#finds determinant btwn two vectors
#pass in two vectors as tuples, returns dot product
def det(vect1, vect2):
    det = (vect1[0] * vect2[1]) - (vect1[1] * vect2[0])
    return det

#heuristic 2: [left turn penalty] (i.e. non-left turn "bonus")
#-25 travelling to neighbor does not require a left turn.
#(ensures the heurisitc remains admissible)
#pass in current/neighbor nodes, current parent dict, node csv filename and performance tracker object
def heuristic2(curr_node, neighbor_node, parents, node_filename, performance_tracker):
    #get parent node of current node
    #note: if curr_node is start node, return 0
    if parents[curr_node] == None: #is start node
        return 0
    else:
        parent_node = parents[curr_node]
    
    #retrieve coordinates of all needed nodes
    #note: [0] = x, [1] = y
    curr_coords = getNodeCoords(node_filename, curr_node)
    parent_coords = getNodeCoords(node_filename, parent_node)
    neighbor_coords = getNodeCoords(node_filename, neighbor_node)

    #form vectors
    #vector to store current direction of movement
    curr_dir = ((curr_coords[0] - parent_coords[0]), (curr_coords[1] - parent_coords[1]))
    #vector to store direction that would be taken if given neighbor picked
    next_dir = ((neighbor_coords[0] - curr_coords[0]), (neighbor_coords[1] - curr_coords[1]))

    #calculate counterclockwise angle between vectors
    #note: + angle = counterclockwise from curr direction
    #      - angle = clockwise from curr direction
    elem1 = det(curr_dir, next_dir) 
    elem2 = dot(curr_dir, next_dir) 
    rad_angle = math.atan2(elem1, elem2) 

    #convert to degrees
    deg_angle = rad_angle * (180 / math.pi)

    #if angle = positive, requires left turn off current road
    bonus = -50 #TUNE
    if deg_angle > 0: #left turn, return 0
        return 0
    else: #will be going straight or turning right, get negative "bonus"
        return bonus
#----------------------------------------------------------------------------------------------------
#heuristic 3: [penalty for straying from goal direction] (i.e. "bonus" for traveling close to goal direction)
#-25 for being within 45 degrees of vector between current node and goal node
#(ensures the heurisitc remains admissible)
#pass in current/neighbor/goal nodes and node csv filename
def heuristic3(curr_node, neighbor_node, goal_node, node_filename):
    #retrieve coordinates of all needed nodes
    #note: [0] = x, [1] = y
    curr_coords = getNodeCoords(node_filename, curr_node)
    goal_coords = getNodeCoords(node_filename, goal_node)
    neighbor_coords = getNodeCoords(node_filename, neighbor_node)

    #form vectors
    #vector to store direction of goal from current node
    goal_dir = ((goal_coords[0] - curr_coords[0]), (goal_coords[1] - curr_coords[1]))
    #vector to store direction that would be taken if given neighbor picked
    next_dir = ((neighbor_coords[0] - curr_coords[0]), (neighbor_coords[1] - curr_coords[1]))

    #calculate counterclockwise angle between vectors
    #note: + angle = counterclockwise from curr direction
    #      - angle = clockwise from curr direction
    elem1 = det(goal_dir, next_dir) 
    elem2 = dot(goal_dir, next_dir) 
    rad_angle = math.atan2(elem1, elem2) 

    #convert to degrees
    deg_angle = rad_angle * (180 / math.pi)

    #if |angle| > 45, strays too much from goal direction
    bonus = -25 #TUNE
    if abs(deg_angle) > 30: #is more than 45 degrees from goal direction, bonus
        return 0
    else: #will be traveling wihtin 45 degrees of goal direction, get negative "bonus"
        return bonus
#----------------------------------------------------------------------------------------------------

#performs A* search
#pass in ids of start/goal nodes, 2D list of node neighbors and node csv filename
#returns dictionary of node parents in resulting path
def astar(start_node, goal_node, neighbors, node_filename, heuristic, performance_tracker):
    #set-up data structures
    parents = {} #dictionary to store minimum cost path to goal (i.e. parents of nodes in path)
    cost_so_far = {} #dictionary to store current path segment cost
    PQ = PriorityQueue() #priority queue to store fringe

    #initialize
    parents[start_node] = None #start node does not have a parent
    cost_so_far[start_node] = 0 #cost of path to start = 0
    PQ.put((0, start_node)) #add start node, priority is 0

    #repeat until priority queue is empty
    while not PQ.empty():
        curr_priority, curr_node = PQ.get()
        #visiting node, increment node count
        #note: "node visit" = everytime a node is popped off of queue, i.e. path is explored
        performance_tracker.addNodeVisit()

        #check for goal
        if curr_node == goal_node:
            break #exit loop
        
        #neighbor tuple list for current node
        curr_neighbors = neighbors[curr_node]

        #iterate thru neighbors of current node
        #note: next index layer = tuple (i.e. 'neighbor'), loop is iterating thru tuples
        #tuple= (neighbor id, l2 distance)
        for neighbor_tuple in curr_neighbors:
            curr_neighbor = neighbor_tuple[0]
            distance = neighbor_tuple[1]

            if (curr_neighbor != -1): #not neighbor if -1
                new_cost = cost_so_far[curr_node] + distance #new g value
                #TRY W/O if curr_node in cost_so_far:, else:
                #if neighbor is not in cost dict or new cost is less than current cost
                if (curr_neighbor not in cost_so_far) or (new_cost < cost_so_far.get(curr_neighbor)):
                    #if (curr_neighbor not in cost_so_far):
                        #[if only want to count newly visited nodes] visiting new node, increment node count 
                        #performance_tracker.addNodeVisit()

                    #use specified heuristic
                    #note: heuristics 2 & 3 are added on top of "standard" heuristic 1
                    if heuristic == 1: #standard, euclidean dist)
                        #new h value
                        heuristic_val = heuristic1(curr_neighbor, goal_node, node_filename)
                        #print("Node:", curr_neighbor, "=", heuristic_val) #CHECK
                    elif heuristic == 2: #left turn penalty
                        standard = heuristic1(curr_neighbor, goal_node, node_filename)
                        #new h value
                        heuristic_val = heuristic2(curr_node, curr_neighbor, parents, node_filename, performance_tracker) + standard
                        #print("Node:", curr_neighbor, "=", heuristic_val) #CHECK
                    elif heuristic == 3: #non-goal direction penalty
                        standard = heuristic1(curr_neighbor, goal_node, node_filename)
                        #new h value
                        heuristic_val = heuristic3(curr_node, curr_neighbor, goal_node, node_filename) + standard
                        #print("Node:", curr_neighbor, "=", heuristic_val) #CHECK

                    #update cost dict with improved cost
                    cost_so_far[curr_neighbor] = new_cost
                    
                    #update priority queue
                    priority = new_cost + heuristic_val #priority-> f = g + h
                    PQ.put((priority, curr_neighbor))

                    #update parent dict for neighor node
                    parents[curr_neighbor] = curr_node

    #priority queue is empty, return parents dict
    return parents

                
########################################################################################################
#FOR TESTING    
def main():
    #performance metric objects
    pt1 = PerformanceTracker()
    pt2 = PerformanceTracker()
    pt3 = PerformanceTracker()
    pt4 = PerformanceTracker()

    #get neighbors
    test_num_nodes = getNumRows("test_nodes.csv")      
    test_neighbors = assignNeighbors("test.csv", test_num_nodes)
    ol_num_nodes = getNumRows("/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cnode.csv")      
    ol_neighbors = assignNeighbors("/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cedge.csv", ol_num_nodes)
    sf_num_nodes = getNumRows("/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cnode.csv")      
    sf_neighbors = assignNeighbors("/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cedge.csv", sf_num_nodes)

    #UCS TEST
    #call ucs w/ start node 4 & goal node 5
    test_parents = ucs(4, 5, test_neighbors, pt1)
    #test with actual map data
    # start = 0
    # end = 4224
    # test_parents = ucs(start, end, ol_neighbors, pt1)
    #test_parents = ucs(start, end, sf_neighbors, pt1)
    #DISPLAY PATH TEST
    # displayPath(start, end, test_parents)
    displayPath(4, 5, test_parents)
    #displayPath(start, end, test_parents)
    # path1 = getPath(start, end, test_parents)
    path1 = getPath(4, 5, test_parents)
    #path1 = getPath(start, end, test_parents)
    print("PATH:", path1)

    #TEST: look for nodes that could work
    # curr_max = 0
    # new_max = -1
    # for i in range(100,150):
    #     #test = astar(i, 5406, ol_neighbors,"/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cnode.csv", 1, pt2)
    #     test = astar(0, i, sf_neighbors,"/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cnode.csv", 1, pt2)
    #     curr_len = len(test)
    #     print(i, curr_len)
    #     if curr_len > curr_max:
    #         curr_max = len(test)
    #         print("NEW:", curr_max)
    #         print("id:", i)
    #         new_max = i
    # print("LONGEST NEIGHBOR LIST", new_max)
    # print("Length:", curr_max)

    #A* TEST
    #heuristic 1
    #call astar w/ start node 4 & goal node 5
    test_parents2 = astar(4, 5, test_neighbors,"test_nodes.csv", 1, pt2)
    # start = 1000
    # end = 4808
    # test_parents2 = astar(start, end, ol_neighbors,"/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cnode.csv", 1, pt2)
    # test_parents2 = astar(start, end, sf_neighbors,"/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cnode.csv", 1, pt2)
    #print("PARENTS", test_parents2)
    #print("length of parents:", len(test_parents2))
    #DISPLAY PATH TEST
    displayPath(4, 5, test_parents2)
    #displayPath(start, end, test_parents2)
    path2 = getPath(4,5,test_parents2)
    #path2 = getPath(start, end, test_parents2)
    print("PATH:", path2)

    #heuristic 2 [left turn penalty]
    #call astar w/ start node 4 & goal node 5
    test_parents3 = astar(4, 5, test_neighbors,"test_nodes.csv", 2, pt3)
    #DISPLAY/FIND PATH TEST
    displayPath(4, 5, test_parents3)
    path3 = getPath(4,5,test_parents3)
    print("PATH:", path3)

    # #heuristic 3 [direction penalty]
    #call astar w/ start node 4 & goal node 5
    test_parents4 = astar(4, 5, test_neighbors,"test_nodes.csv", 3, pt4)
    #DISPLAY/FIND PATH TEST
    displayPath(4, 5, test_parents4)
    path4 = getPath(4,5,test_parents3)
    print("PATH:", path4)

    #PERFORMANCE TEST
    print("1. UCS ------------")
    findEdgeCount(path1, pt1)
    print("Edges:", pt1.getEdgesTraversed())
    findDistCovered(path1, test_neighbors, pt1)
    #findDistCovered(path1, ol_neighbors, pt1)
    #findDistCovered(path1, sf_neighbors, pt1)
    print("Distance covered:", pt1.getDistCovered())
    findLTurnsTaken(path1, "test_nodes.csv", pt1)
    #findLTurnsTaken(path1, "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cnode.csv", pt1)
    #findLTurnsTaken(path1, "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cnode.csv", pt1)
    print("Left turns made:", pt1.getLTurnsMade())
    print("Nodes visited:", pt1.getNodesVisited())

    print("2. A Star, Heuristic 1 ------------")
    findEdgeCount(path2, pt2)
    print("Edges:", pt2.getEdgesTraversed())
    findDistCovered(path2, test_neighbors, pt2)
    #findDistCovered(path2, ol_neighbors, pt2)
    #findDistCovered(path2, sf_neighbors, pt2)
    print("Distance covered:", pt2.getDistCovered())
    # findLTurnsTaken(path2, "test_nodes.csv", pt2)
    #findLTurnsTaken(path2, "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cnode.csv", pt2)
    #findLTurnsTaken(path1, "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cnode.csv", pt2)
    print("Left turns made:", pt2.getLTurnsMade())
    print("Nodes visited:", pt2.getNodesVisited())

    print("3. A Star, Heuristic 2 ------------")
    findEdgeCount(path3, pt3)
    print("Edges:", pt3.getEdgesTraversed())
    findDistCovered(path3, test_neighbors, pt3)
    print("Distance covered:", pt3.getDistCovered())
    findLTurnsTaken(path3, "test_nodes.csv", pt3)
    print("Left turns made:", pt3.getLTurnsMade())
    print("Nodes visited:", pt3.getNodesVisited())

    print("4. A Star, Heuristic 3 ------------")
    findEdgeCount(path4, pt4)
    print("Edges:", pt4.getEdgesTraversed())
    findDistCovered(path4, test_neighbors, pt4)
    print("Distance covered:", pt4.getDistCovered())
    findLTurnsTaken(path4, "test_nodes.csv", pt4)
    print("Left turns made:", pt4.getLTurnsMade())
    print("Nodes visited:", pt4.getNodesVisited())

if __name__ == '__main__' :
    main()
            

        