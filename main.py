#MAIN FUNCTION
#Maps tested:
#San Francisco, CA, United States (large amount of nodes)
#Oldenburg, Germany (smaller amount of nodes)
#Data from: https://www.cs.utah.edu/~lifeifei/SpatialDataset.htm
######################################################################################
from maps import*
from run_tests import*
from performance_metrics import*

def main():
    ###########################MAP 1: SAN FRANCISCO###################################
    # #filenames for csv files that store map data
    # sf_node_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cnode.csv"
    # sf_edge_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/SF.cedge.csv"

    # #test start/goal pairs (start_node_id, goal_node_id)
    # sf_short = (118, 173166)
    # sf_mid = (44, 65370)
    # sf_ far = (0, 75156)

    # # #make new map object
    # # map1 = Map("SAN FRANCISCO, CA", short, mid, far, sf_node_csv, sf_edge_csv)

    # #run tests
    # run_tests(sf_node_csv, sf_edge_csv, "SAN FRANCISCO, CA", short, mid, far)

    ##################################################################################
    
    #############################MAP 2: OLDENBURG#####################################
    #filenames for csv files that store map data
    ol_node_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cnode.csv"
    ol_edge_csv = "/Users/hannahwillmarth/Desktop/4511WFinalProject/data/OL.cedge.csv"

    #test start/goal pairs (start_node_id, goal_node_id)
    ol_short = (2, 2565)
    ol_mid = (1000, 4808)
    ol_far = (309, 2652)

    #run tests
    run_tests(ol_node_csv, ol_edge_csv, "OLDENBURG, DE", ol_short, ol_mid, ol_far)

if __name__ == '__main__' :
    main()