#PERFORMANCE METRICS CLASS
###########################################################################################
import time

#class with variables and functions needed to measure performance of algorithms/heuristics
class PerformanceTracker:
    #constructor
    def __init__(self): #shouldn't need to pass anything in 
        self.edges_traversed = 0 #edges traversed on path, i.e. roads taken
        self.dist_covered = 0.0 #total distance travelled on path
        self.nodes_visited = 0 #nodes visited while searching
        self.lturns_made = 0 #left turns made on path

        #used to store start and end times, initialize to current time
        self.start = None
        self.end = None

    #Getters:
    #gets # of edges traversed
    def getEdgesTraversed(self):
        return self.edges_traversed
    
    #get distance covered on path
    def getDistCovered(self):
        return self.dist_covered

    #get # of nodes visited so far
    def getNodesVisited(self):
        return self.nodes_visited

    #get # of turns made so far
    def getLTurnsMade(self):
        return self.lturns_made

    #Setters:
    #sets edges_traversed
    def setEdges(self, edges):
        self.edges_traversed = edges
    
    #sets edges_traversed
    def setDistCovered(self, dist):
        self.dist_covered = dist
    
    #increments sets lturns_made 
    def setLTurns(self, lturns):
        self.lturns_made = lturns
    
    #increments nodes_visited by 1 (new node has been visited)
    def addNodeVisit(self):
        self.nodes_visited += 1
    
    #Timer functions:
    #"start" timer (precision = microseconds)
    def startTimer(self):
        self.start = time.perf_counter()

    #"end" timer (precision = microseconds)
    def endTimer(self):
        elapsed = time.perf_counter() - self.start
        self.start = None
        return elapsed

    #calculate elasped
    def timeElapsed(self):
        elasped = self.end - self.start
        return elasped

###########################################################################################
#FOR TESTING 
def main():
    test_tracker = PerformanceTracker()
    #start time
    test_tracker.startTimer()

    #getters
    print("Edges traversed: " + str(test_tracker.getEdgesTraversed()))
    print("Turns made: " + str(test_tracker.getLTurnsMade()))

    #setters
    print("*New turn made*")
    test_tracker.setLTurns(1)
    print("Left turns made: " + str(test_tracker.getLTurnsMade()))

    #end time
    test_tracker.endTimer()
    print("Time taken: " + str(test_tracker.timeElapsed()))

if __name__ == '__main__' :
    main()
    