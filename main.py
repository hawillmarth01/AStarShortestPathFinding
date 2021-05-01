#MAIN FUNCTION
#Maps tested:
#San Francisco, CA, United States (large amount of nodes)
#Oldenburg, Germany (smaller amount of nodes)
#Data from: https://www.cs.utah.edu/~lifeifei/SpatialDataset.htm
###########################################################################################
from maps import*
from run_tests import*
from performance_metrics import*

def main():
    ###########################MAP 1: SAN FRANCISCO###################################
    # #filenames for csv files that store map data
    # sf_node_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cnode.csv"
    # sf_edge_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cedge.csv"

    # #test start/goal pairs (start_node_id, goal_node_id)
    # sf_short = (100076, 174347)
    # sf_mid = (49520, 174130)
    # sf_ far = (48580, 30272)

    # # #make new map object
    # # map1 = Map("SAN FRANCISCO, CA", short, mid, far, sf_node_csv, sf_edge_csv)

    # #run tests
    # run_tests(sf_node_csv, sf_edge_csv, "SAN FRANCISCO, CA", short, mid, far)

    ##################################################################################
    
    ###########################MAP 2: OLDENBURG###################################
    #filenames for csv files that store map data
    ol_node_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cnode.csv"
    ol_edge_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cedge.csv"

    #test start/goal pairs (start_node_id, goal_node_id)
    ol_short = (1250, 238)#(5618, 3698)
    ol_mid = (4624, 6077)
    ol_far = (5335, 3981)

    #run tests
    run_tests(ol_node_csv, ol_edge_csv, "OLDENBURG, DE", ol_short, ol_mid, ol_far)

if __name__ == '__main__' :
    main()