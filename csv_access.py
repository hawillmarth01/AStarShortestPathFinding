#CSV HELPER
#functions used to get data from two csv files

#relevant files:
#SF.cedge.csv- road edges
#SF.cnode.csv- road start/end nodes
###########################################################################################
import numpy as np
import csv

#gets number of nodes in map by counting rows in node file
def getNumRows(filename):
    #open node csv to retrieve node coords
    csv_file = open(filename)
    csv_helper = csv.reader(csv_file)

    #count number of rows
    num_rows = sum(1 for row in csv_helper)

    csv_file.close() #closer node csv

    return num_rows

#different roads will have different #s of neighbors, ragged array is expected.
# np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning) #ignore warning

#assign node neighbors, returns neighbor list (list of tuple lists)
#node id = index into outer list, (ex. neighbors[x] = neighbor tuple list for node x)
#given node neighbor tuple = (neighbor, L2 dist between node & neighbor)
def assignNeighbors(edge_filename, num_nodes):
    #open edge csv file
    edge_file = open(edge_filename) #open csv to use for outer loop
    #edge_fileV2 = open(edge_filename) #open csv to use for inner loop

    #note: file should not have column headers
    csv_helper1 = csv.reader(edge_file)
    #csv_helper2 = csv.reader(edge_fileV2)
    #col_names = next(csv_helper1)
    #col_names2 = next(csv_helper2)

    #list to store neighbors of all nodes
    #note: index in array = node id
    neighbors = [[(-1,0.0)]] * num_nodes #initialize neighbor ids to -1, distance to 0.0
    #2D numpy array to store neighbors of all nodes
    #note: index in array = node id
    #edge_neighbors = []

    #iterate thru all edges
    #start node = 2nd column, end/neighbor node = 3rd column, l2 distance = 4th column
    for edge_row in csv_helper1: #iterate thru all edges, end nodes specifically
        #retrieve data
        curr_node = int(edge_row[1]) #start node
        neighbor = int(edge_row[2]) #end node
        dist_btwn = float(edge_row[3]) #l2 dist between start & end nodes
        
        #add (neighbor, dist) pair to node's neighbors list
        curr_neighbors = neighbors[curr_node] #list of node's current neighbors
        if (curr_neighbors[0] == (-1, 0.0)): #no neighbors so far, delete default tuple
            neighbors[curr_node] = [(neighbor, dist_btwn)]
        else: #append to current neighbors
            curr_neighbors.append((neighbor, dist_btwn))
    
    #USED TO FIND EDGE NEIGHBORS:
    #nested loop-> iterate through all end nodes and compare to all start nodes
    #start node = 2nd column, end node = 3rd column
    # for endpt_row in csv_helper1: #iterate thru all edges, end nodes specifically
    #     curr_neighbors = [] #list to store neighbors of current edge
    #     curr_endpt = endpt_row[2] #store current endpt

    #     #CHECK
    #     #print("currently looking at edge " + endpt_row[0] + " w/ endpt: " + endpt_row[2])

    #     for startpt_row in csv_helper2: #iterate thru all edges, check for matching start node.
    #         #CHECK
    #         #print("comparing to startpt: " + startpt_row[1])

    #         curr_startpt = startpt_row[1]
    #         if (curr_startpt == curr_endpt): #neighbor found, store in list
    #             #CHECK
    #             #print("NEIGHBOR FOUND")
    #             curr_neighbors.append(int(startpt_row[0])) #append edge_id of neighbor
                
    #     #have appended all neighboring edge_ids to current neighbor array
    #     #curr_neighbors = np.array(curr_neighbors) #NEED? convert to np array
    #     edge_neighbors.append(curr_neighbors) #append to 2D neighbor array
    #     edge_fileV2.seek(0) #rewind file for next outerloop iteration
    
    
    edge_file.close() #close edge csv
    # edge_fileV2.close()

    return neighbors

#retrieve coordinates of node id from node csv
#pass in node csv, node id, returns coords as (float, float)
def getNodeCoords(node_filename, node_id):
    #open node csv to retrieve node coords
    node_file = open(node_filename)
    csv_helper = csv.reader(node_file)

    #go to row corresponding to desired node id
    node_rows = list(csv_helper)
    node_row = node_rows[node_id]

    #get node coords
    #x coord = 2nd column, y coord = 3rd column
    x_coord = float(node_row[1])
    y_coord = float(node_row[2])

    node_file.close() #close node csv

    return (x_coord,y_coord)

###########PROBABLY ONLY NEED BELOW FOR EDGE VERSION###########
#retrieve length of road using edge_id, returns l2 distance as a float
#l2 distance = 4th column
# def getLength(edge_filename, edge_id):
#     #open edge csv to retrieve needed length
#     edge_file = open(edge_filename)
#     csv_helper = csv.reader(edge_file)
    
#     #go to row corresponding to given edge_id
#     rows = list(csv_helper)
#     edge_row = rows[edge_id]

#     #get length of edge (l2 distance)
#     edge_len = float(edge_row[3])

#     edge_file.close()

#     return edge_len


#retrieve start or end node coordinates from node csv
#pass in edge csv, node csv, edge id, start||end, returns coords as (float, float)
#start=0, end=1
# def getCoordsEdge(edge_filename, node_filename, edge_id, start_or_end):
#     #open edge csv to retrieve needed length
#     edge_file = open(edge_filename)
#     csv_helper = csv.reader(edge_file)

#     #go to row corresponding to given edge_id
#     edge_rows = list(csv_helper)
#     edge_row = edge_rows[edge_id]
    
#     #get desired node id
#     #start node = 2nd column, end node = 3rd column
#     node_id = -1 #initialize
#     if start_or_end == 0: #need start node
#         node_id = int(edge_row[1])
#     elif start_or_end == 1: #need end node
#         node_id = int(edge_row[2])

#     edge_file.close() #close edge csv

#     #get node coords
#     (x_coord, y_coord) = getNodeCoords(node_filename, node_id)

#     return (x_coord,y_coord)


###########################################################################################
#FOR TESTING    
def main():
    #NUM NODES TEST
    test_num_nodes = getNumNodes("test_nodes.csv")
    print("Number of nodes:", test_num_nodes)
    #NEIGHBOR TEST
    #actual file path: /Users/hannahwillmarth/Desktop/4511WFinalProject/data(current)/ + __
    test_neighbors = assignNeighbors("test.csv", test_num_nodes)
    print("All neighbors:", test_neighbors)
    print("Neighbors of node 2:", test_neighbors[2])
    print("Distance between node 2 and its first neighbor:", test_neighbors[2][0][1])

    #GET COORDS TEST
    #get coords of node w/ id 4
    test_coords = getNodeCoords("test_nodes.csv", 4)
    print("Coords of node 8:", test_coords)

    #GET LENGTH TEST
    # #get l2 dist of edge w/ id 4
    # test_len = getLength("test.csv", 4)
    # print("Length of edge 4:", test_len)

    # #GET COORDS BASED ON EDGE TEST
    # #get coords of start node of edge w/ id 7 (which is node id 4)
    # test_edge_coords = getCoordsEdge("test.csv", "test_nodes.csv", 7, 0)
    # print("Coords of edge 2 start node:", test_edge_coords)

if __name__ == '__main__' :
    main()

        
