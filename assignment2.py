#RRT Star algorithm
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams
np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 15

#tree Node class
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        
#RRT Star Algorithm class
class RRTStarAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])          #The RRT (root position) (has 0 cost)
        self.goal = treeNode(goal[0], goal[1])                  #goal position (initialize to a high cost)
        self.nearestNode = None                                 #nearest node            
        self.iterations = min(numIterations, 700)               #number of iterations to run
        self.grid = grid                                        #the map
        self.rho = stepSize                                     #length of each branch   
        self.nearestDist = 10000                                #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                   #number of waypoints
        self.Waypoints = []                                     #the waypoints
        self.searchRadius = self.rho*2                          #the radius to search for finding neighbouring vertices 
        self.neighbouringNodes = []                             #neighbouring nodes  
        self.goalArray = np.array([goal[0],goal[1]])            #goal as an array
        self.goalCosts = [10000]                                #the costs to the goal (ignore first value)
            
    #add the node to the nearest node, and add goal if necessary (TODO--------)    
    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            #append goal to nearestNode's children
            #and set goal's parent to nearestNode
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            #create a tree node from locationX, locationY
            #append this node to nearestNode's children
            #set the parent to nearestNode
            node = treeNode(locationX, locationY)
            self.nearestNode.children.append(node)
            node.parent = self.nearestNode
            print(f"Parent for {node.locationX}, {node.locationY} :{node.parent.locationX}, {node.parent.locationY}")

        
    #sample random point within grid limits (DONE)
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point
    
    #steer a distance stepSize from start location to end location (keep in mind the grid limits) (DONE)
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]-1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0]-1
        return point
    
    #check if obstacle lies between the start and end point of the edge (DONE)
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        dist = self.distance(locationStart, locationEnd)
        for i in range(int(dist)):
            testPoint[0] = min(grid.shape[1]-1, locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(grid.shape[0]-1,locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[1]),round(testPoint[0])] == 1:
                return True
        return False

    #find the unit vector between 2 locations (DONE)
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    #find the nearest node from a given (unconnected) point (Euclidean distance) (TODO--------)
    def findNearest(self, root, point):
        # print(f"Root:{root},{type(root)}")
        if not root:
            return
        # plt.plot(point[0],point[1],'bo')
        # print(f"root:{root.locationX}, {root.locationY}")
        distance = self.distance(root, point)
        # print(f"distance:{distance}")
        if distance <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = distance

        #find distance between root and point use distance method,
        #if it's lower than or equal to nearestDist then
        #update nearestNode to root
        #update nearestDist to the distance from line 84
        for child in root.children:
            self.findNearest(child, point)
            
    #find neighbouring nodes (TODO--------)         
    def findNeighbouringNodes(self,root,point):
        if not root:
            return
        #find distance between root and point (dist)
        dist = self.distance(root, point)
        #add root to neighbouringNodes if dist is less than or equal to searchRadius
        if dist <= self.searchRadius:
            self.neighbouringNodes.append(root)
        #recursive call
        for child in root.children:
            self.findNeighbouringNodes(child, point)        

    #find euclidean distance between a node and an XY point (DONE)
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist
    
    #check if the goal is within stepsize (rho) distance from point, return true if so otherwise false (TODO--------)
    def goalFound(self, point):
        # Calculate the distance between the point and the goal center
        distance = np.sqrt((self.goal.locationX - point[0])**2 + (self.goal.locationY - point[1])**2)
        
        # Check if the distance is within the radius
        if distance <= self.rho:
            print(f" GOAL REACHED")
            return True
        return False
    
    #reset: set nearestNode to None and nearestDistance to 10000 (TODO--------)
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        pass #delete this when you're done
    
    #trace the path from goal to start, since have to reset if called many times, do this iteratively (TODO--------)
    def retracePath(self):
        self.numWaypoints = 0
        self.Waypoints = []
        goalCost = 0
        goal = self.goal
        while goal.locationX != self.randomTree.locationX:
            print(f"compare:::::{goal.locationX},{goal.locationY}")
            #add 1 to numWaypoints
            self.numWaypoints += 1
            self.Waypoints.append(np.array([goal.locationX, goal.locationY]))
            #extract the X Y location of goal in a numpy array 
            #insert this array to waypoints (from the beginning)
            #add distance between the node and it's parent to goalCost (goalCost keeps increasing)        
              #set the node to it's parent
            print(f"goal======={goal.parent.locationX}")
            goalCost += self.distance(goal, np.array([goal.parent.locationX, goal.parent.locationY]))
            goal = goal.parent
        self.goalCosts.append(goalCost)    
        
    #find unique path length from root of a node (cost) (DONE)
    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        while currentNode.locationX != self.randomTree.locationX:
            if currentNode.parent == None:
                print(currentNode.locationX, currentNode.locationY)
                plt.plot(currentNode.locationX, currentNode.locationY,'bo')
                plt.pause(5)
            costFromRoot += self.distance(currentNode, np.array([currentNode.parent.locationX, currentNode.parent.locationY])) 
            currentNode = currentNode.parent   
        return costFromRoot    
        
#end of class definitions
#----------------------------------------------------------------------------------------------------------------------------#
        
#load the grid, set start and goal <x, y> positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([300.0, 300.0])
goal = np.array([1400.0, 775.0])
numIterations = 700
stepSize = 75
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Star Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0],start[1],'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
    
#Begin
rrtStar = RRTStarAlgorithm(start, goal, numIterations, grid, stepSize)
plt.pause(2)

#RRT Star algorithm (TODO-------)
#iterate
for i in range(rrtStar.iterations):
    
    #Reset nearest values, call the resetNearestValues method
    print("Iteration: ",i)
    rrtStar.resetNearestValues()
    
    #algorithm begins here-------------------------------------------------------------------------------
    
    #sample a point (use the appropriate method)
    point = rrtStar.sampleAPoint()
    #find the nearest node w.r.t to the point (just call the method do not return anything)
    rrtStar.findNearest(rrtStar.randomTree, point)
    #steer to point, set the returned variable to ('new')
    new = rrtStar.steerToPoint(rrtStar.nearestNode, point)
    #if not in obstacle    
    if not rrtStar.isInObstacle(rrtStar.nearestNode, new):
        rrtStar.findNeighbouringNodes(rrtStar.randomTree, new)
        min_cost_node = rrtStar.nearestNode
        min_cost = rrtStar.findPathDistance(min_cost_node)
        min_cost = min_cost + rrtStar.distance(rrtStar.nearestNode, new)
        
        #connect along minimum cost path (TODO-------)

        #for each node in neighbouringNodes
            #find the cost from the root (findPathDistance)
            #add the distance between the node and the new point ('new') to the above cost (use the relevant method)
        if len(rrtStar.neighbouringNodes) > 0:
            for node in rrtStar.neighbouringNodes:
                g_node = rrtStar.findPathDistance(node)
                c_node = rrtStar.distance(node, new)
                cost_node = g_node + c_node
            #if node and new are obstacle free AND the cost is lower than min_cost (use the relevant method)
                if (not rrtStar.isInObstacle(node, new)) and (cost_node < min_cost): 
                #set the min_cost_node to this node
                    min_cost_node = node
                #set the min_cost to this cost
                    min_cost = cost_node
        else:
            print(f"heeeeeeeeeeeeeeeeeeeeeeeeeeereeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
        #update nearest node to min_cost_node, create a treeNode object from the new point - call this newNode ('new[0],new[1]')
        rrtStar.nearestNode = min_cost_node
        newNode = treeNode(new[0], new[1])
        #SIDE NOTE: if neighbouringNodes is empty, it'll add to the original nearest node (obstacle free)  
        #addChild (add newNode to the nearest node - which is now updated and is the minimum cost node)
        rrtStar.addChild(new[0], new[1])
        newNode.parent = rrtStar.nearestNode   
        #plot for display
        plt.pause(0.01)
        plt.plot([rrtStar.nearestNode.locationX, new[0]], [rrtStar.nearestNode.locationY, new[1]],'go', linestyle="--")  
        
        #rewire tree (TODO-------)    
        #for each node in neighbouringNodes
        for node in rrtStar.neighbouringNodes:
            #set a variable: 'cost' to min_cost
            # node_arr = np.array([node.locationX, node.locationY])
            cost = min_cost + rrtStar.distance(node, new)
            #add the distance between 'new' and node to cost
            # g_node = min_cost + rrtStar.distance(node, new)
            
            #if node and new are obstacle free AND the cost is lower than the distance from root to vertex (use findPathDistance method)
            if (not rrtStar.isInObstacle(node, new)) and (cost < rrtStar.findPathDistance(node)):
            #set the parent of node to 'newNode' (see line 190)
                print(f"\n\n\nheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\n\n\n\n")
                print(f"Parent second before check for {newNode.locationX}, {newNode.locationY}, {newNode.parent.locationX} ")
                node.parent = newNode
                print(f"Parent third check for {newNode.locationX}, {newNode.locationY}")


        
            
        #if goal found, and the projected cost is lower, then append to path let it sample more (DONE)
        point = np.array([newNode.locationX, newNode.locationY])
        if rrtStar.goalFound(point):
            print(f"Error about to happpenn")
            print(f"Parent :{newNode.parent}")
            projectedCost = rrtStar.findPathDistance(newNode) + rrtStar.distance(rrtStar.goal, point)
            print(f"error happened")
            if projectedCost < rrtStar.goalCosts[-1]:
                rrtStar.addChild(rrtStar.goal.locationX, rrtStar.goal.locationY)
                rrtStar.goal.parent = rrtStar.nearestNode
                print(f"==============================={rrtStar.goal.parent.locationX}")
                plt.plot([rrtStar.nearestNode.locationX, rrtStar.goalArray[0]], [rrtStar.nearestNode.locationY, rrtStar.goalArray[1]],'go', linestyle="--") 
                #retrace and plot, this method finds waypoints and cost from root
                rrtStar.retracePath()
                print("Goal Cost: ", rrtStar.goalCosts)
                plt.pause(0.25)
                rrtStar.Waypoints.append(start)
                #plot the waypoints
                for i in range(len(rrtStar.Waypoints)-1):
                    plt.plot([rrtStar.Waypoints[i][0], rrtStar.Waypoints[i+1][0]], [rrtStar.Waypoints[i][1], rrtStar.Waypoints[i+1][1]],'ro', linestyle="--")
                    plt.pause(0.01)

print("Goal Costs: ", rrtStar.goalCosts) 
plt.pause(100)
