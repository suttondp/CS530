from __future__ import (print_function, absolute_import,
                       division, unicode_literals)

try:
    from future_builtins import *   # @UnusedWildImport
    import codecs                   # python-2
    open = codecs.open              # @ReservedAssignment
    input = raw_input               # @ReservedAssignment
    range = xrange                  # @ReservedAssignment
except ImportError:
    pass                            # python-3

try:
    from org.cs540.team1.BlockWorldSim import BlockWorld
except ImportError:
    from BlockWorldSim import BlockWorld
#import numpy as np
import StateCreator
import copy
#import random
#import math
import time

# Global variable nNodes is used to to track the number of nodes visited for multiple iterations of the algorithms.
nNodes = 0
#nodes = []

# "Node" class which represents the current state of a puzzle/problem in terms of it's current state,
#  path cost so far, heuristic cost to goal and total estimated cost to goal
class Node:
    def __init__(self, state, f=0, g=0 ,h=0):
        self.state = state
        self.path = []
        self.f = f
        self.g = g
        self.h = h
    def __repr__(self):
        return "Node(" + repr(self.state) + ", f=" + repr(self.f) + \
               ", g=" + repr(self.g) + ", h=" + repr(self.h) + ")"
    def equals(self, nodeB):
        return self.state == nodeB.state
'''
Purpose: Runs the aStarHelper function and could be used to adjust fmax if needed - used to run AStarHelper for pickup and dropoff routes
Arguments:

startState: the current BlockWorld object
goal: A tuple of ((X1,Y1,Z1),(X2,Y2,Z2)) that represents the start and finish positions of a block to move
bw: The current block world  
actionsF: a function that is given a state and returns a list of valid actions from that state,
takeActionF: a function that is given a state and an action and returns the new state that results from applying the action to the state,
goalTestF: a function that tests a given state to determine if it equals the goal 
hF: the heuristic function to apply in determining future cost at each state
fmax: the maximum cost to search before declaring a path a failure; provides a limit to the search space (1000 only good for very small spaces)
Result: Returns the path to the goal or 'failure' if not found along with the depth at which the goal was found.
'''
def aStar_low_level(startState, goal, fmax= 1000):
    global nNodes

    h = pickup_distance_heuristic(startState.dronePos, goal)
    startNode = Node(state=startState.dronePos, f=0+h, g=0, h=h)
    stateCopy = copy.deepcopy(startState)

    path = []
    moves, pickup_cost = astarHelper(stateCopy , startNode, goal, bw_actionsF,takeActionF_bw,pickup_goalTest,
                              pickup_distance_heuristic, float('inf'))
    if moves == 'failure':
        print("Search failed")
        return 'failure'

    nodes1 = nNodes
    nNodes = 0
    moves.pop(0)
    for move in moves:
        path.append(move[0])
    #moves = bw_movements(bw)

    stateCopy = copy.deepcopy(startState)

    #print(moves, cost)

    #for node in nodes:
        #print(node.state)
    for move in moves:
        stateCopy.move(*move[0])

    stateCopy.attach()
    path.append('attach')
    h = dropoff_distance_heuristic(stateCopy.dronePos, goal)
    startNode = Node(state=stateCopy.dronePos, f=0+h, g=0, h=h)


    moves, cost2 = astarHelper(stateCopy, startNode, goal, bw_actionsF,takeActionF_bw,dropoff_goalTest,
                              dropoff_distance_heuristic, float('inf'))
    #path.append(moves)
    #print(moves, cost)
    moves.pop(0)

    for move in moves:
        path.append(move[0])
        #if move != []:
        #print(move[0])
        #bw.move(*move[0])

    #bw.detach()
    path.append('detach')

    return path, len(path), nodes1 + nNodes
    #print ("Cost for best path is: {0}, number of nodes explored = {1}, path is:".format(cost + cost2, nNodes))
    #print(moves)
    #gt_locs = get_tower_locations(goal, startState.Xsize, startState.Zsize)
    #h = hF(startState.dronePos, goal)
    #startNode = Node(state=startState.dronePos, f=0+h, g=0, h=h)
    #return astarHelper(copy.deepcopy(startState), startNode, goal, actionsF, takeActionF, goalTestF, hF, float('inf'))

'''
Function: aStarHelper(startState, actionsF, takeActionF, goalTestF, hF, fmax)

Purpose: Implements A* search recursively using the recursive best-first search (RBFS) algorithm from CS440 class lecture notes,
 slightly modified to avoid an infinite loop if the goal is not found. The algorithm moves through the possible states and at each one:

    Checks to see if the goal has been found
    Determines what successor (child) states are
    Calculates the estimated cost to the goal for each child using f(x) = g(x) + h(x)
    Sorts the children by f value and then calls them in order of best to worst
    In each iteration where the goal is not found, returns 'failure' and the current f value so that it's parent will be able to be able to 
    appropriately prioritize which nodes to expand in the future.
    
NOTE: This implementation has been adjusted slightly such that the state is represented primarily by the drone position instead of 
creating thousands of copies of the entire BlockWorld object.  Instead, a single BlockWorld object is edited in each call to reflect the 
information provided by the drone position and it's connected block (if applicable)

Arguments:

startState: the starting state for the chosen problem
actionsF: a function that is given a state and returns a list of valid actions from that state,
takeActionF: a function that is given a state and an action and returns the new state that results from applying the action to the state,
goalTestF: a function that tests a given state to determine if it equals the goal 
hF: the heuristic function to apply in determining future cost at each state
fmax: the maximum depth/cost that the algorithm should search to

Result: Returns the path to the goal or 'failure' if not found along with the depth at which the goal was found.
'''
def astarHelper(bw_state, parentNode, goal, actionsF, takeActionF, goalTestF, hF, fmax):
    global nNodes
    #global nodes

    if goalTestF(parentNode.state, goal):
        return ([parentNode.path], parentNode.g)

    # Adjust the block world to reflect the drone position of the parent node - this should work for a recursive implementation,
    # but it's certainly not thread safe
    oldDronePos = copy.deepcopy(bw_state.dronePos)
    # Remove old location information for the drone
    bw_state._wr(oldDronePos[0], oldDronePos[1], oldDronePos[2], 0)
    #Overwrite new location information for the drone and its attached block
    bw_state.dronePos = parentNode.state
    bw_state._wr(bw_state.dronePos[0], bw_state.dronePos[1], bw_state.dronePos[2],9)

    # If the drone is attached to a block, then do the same for the attached block
    if bw_state.attached:
        #color = bw_state.arr[oldDronePos[0], oldDronePos[1]-1, oldDronePos[2]]
        bw_state._wr(oldDronePos[0], oldDronePos[1]-1, oldDronePos[2], 0)
        # Arbitrarily setting the attached block color to 1 because we're not working on the master version of the Block World anyway
        bw_state._wr(bw_state.dronePos[0], bw_state.dronePos[1]-1, bw_state.dronePos[2], 1)

    ## Construct list of children nodes with f, g, and h values
    actions = actionsF(bw_state)

    # If there are no more actions to take, return failure
    if not actions:
        return ("failure", float('inf'))
    children = []
    # For each action, create a child node based on that action and calculate h, g, and f values for it
    for action in actions:
        childDronePos,stepCost = takeActionF(parentNode.state, action)
        h = hF(childDronePos, goal)
        g = parentNode.g + stepCost
        f = max(h+g, parentNode.f)
        childNode = Node(state=childDronePos, f=f, g=g, h=h)
        childNode.path.append(action)
        children.append(childNode)

    # Loop until a result is found or the search space is exhausted
    while True:
        # find best child and start with it
        if len(children) > 0:
            children.sort(key = lambda n: n.f) # sort by f value
        else:
            return ("failure", float('inf'))

        bestChild = children[0]
        #if bestChild.h == 1:
        #    print(bestChild)
        # If the best option exceeds the provided max value for f, then this node has failed to find a solution
        # Added second condition in the if statement to account for a non-existant goal
        if bestChild.f > fmax or bestChild.f == float('inf'):
            return ("failure",bestChild.f)
        # alternatef is the next lowest f value
        alternativef = children[1].f if len(children) > 1 else float('inf')

        nNodes += 1
        # recursively call astarHelper to get the result and the f value
        result, bestChild.f = astarHelper(bw_state, bestChild, goal,actionsF, takeActionF, goalTestF,
                                        hF, min(fmax,alternativef))

        # If a result is found, build the solution path and return it
        if result is not "failure":
            result.insert(0,parentNode.path)
            return (result, bestChild.f)


'''Function: bw_actionsF(bw)
Purpose: Identify the possible moves that can be taken by the drone for the provided state of the Block World
Arguments:
bw: the Block World object representing a possible state of the world

Result: Return a list of tuples consisting of the possible movement vectors:
'''
def bw_actionsF(bw):
    moves = []

    for x in range (-1,2):
        for y in range (-1,2):
            for z in range(-1,2):
                droneX, droneY, droneZ = bw.dronePos
                droneX += x
                droneY += y
                droneZ += z
                if validMove(bw, droneX, droneY, droneZ):
                    moves.append((x,y,z))

    return moves

'''
Function: takeActionF_bw (state, moves)

Purpose: Calculate a new drone position given an action 
Arguments:

dronePos: The current location of the drone in the Block World
action: a tuple containing a single (dx, dy, dz) move vector

Result: Returns a tuple of the new drone position that results from the action and the cost to reach it (always 1)
'''
def takeActionF_bw(dronePos, action):
    newDroneX = dronePos[0] + action[0]
    newDroneY = dronePos[1] + action[1]
    newDroneZ = dronePos[2] + action[2]
    return (newDroneX, newDroneY, newDroneZ), 1


'''Function: pickup_goaltest (current_state, goal)

Purpose: Tests whether the drone in the provided state is in the pickup location

Arguments:
dronePos: The current location of the drone in the Block World
goal: a (start, end) tuple where start is the location of the block to be picked up

Result: Returns true if drone is directly above the pickup location, false otherwise
'''
def pickup_goalTest(dronePos, goal):
    blockStartPos = goal[0]

    return dronePos[0] == blockStartPos[0] and dronePos[1] == (blockStartPos[1]+1) and dronePos[2] == blockStartPos[2]

'''Function: dropoff_goaltest (current_state, goal)

Purpose: Tests whether the drone in the provided state is in a valid dropoff location

Arguments:
dronePos: The current location of the drone in the Block World
goal: the location of the block to be picked up

Result: Returns true if drone is above the dropoff location, false otherwise
'''
def dropoff_goalTest(dronePos, goal):
    blockEndPos = goal[1]
    return dronePos[0] == blockEndPos[0] and dronePos[1] >= (blockEndPos[1]+1) and dronePos[2] == blockEndPos[2]

'''
Function: pickup_distance_heuristic(dronePos, goal)

Purpose: Estimates minimum cost to for drone to reach pickup location.  Since the drone can move in X, Y, and Z simultaneously, 
the minimum cost to reach a specific location is the maximum of the difference between the current location and the desired location 
in any dimension, X, Y, or Z.

Arguments:
dronePos: The current location of the drone in the Block World
goal: a (start, end) tuple where start is the location of the block to be picked up

Result: Returns the minimum cost to reach the pickup location from the current drone location
'''
# Calculate heuristic for getting the drone to the location of the block to be moved
def pickup_distance_heuristic(dronePos, goal):
        blockStartPos = goal[0]

        #
        # The "+1" in the y position reflects that the drone needs to be above the block
        return max(abs(dronePos[0] - blockStartPos[0]), abs(dronePos[1] - (blockStartPos[1]+1)), abs(dronePos[2] - blockStartPos[2]))

'''
Function: dropoff_distance_heuristic(dronePos, goal)

Purpose: Estimates minimum cost to for drone to reach dropoff location.  Since the drone can move in X, Y, and Z simultaneously, 
the minimum cost to reach a specific location is the maximum of the difference between the current location and the desired location 
in any dimension, X, Y, or Z.

Arguments:
dronePos: The current location of the drone in the Block World
goal: a (start, end) tuple where end is the destination location of an attached block

Result: Returns the minimum cost to reach the pickup location from the current drone location
'''
def dropoff_distance_heuristic(dronePos, goal):
    blockEndPos = goal[1]
    # If the drone plus attached block is at or above the desired Y location, then only consider X and Z distance
    # since the block can be dropped to its position from any height above it.  Otherwise consider all three dimensions
    # The "+1" in the y position reflects that the drone and attached block must be above the current top block or the ground
    if dronePos[1] - (blockEndPos[1]+1) >= 0:
        return max(abs(dronePos[0] - blockEndPos[0]), abs(dronePos[2] - blockEndPos[2]))
    else:
        return max(abs(dronePos[0] - blockEndPos[0]), abs(dronePos[1] - (blockEndPos[1]+1)), abs(dronePos[2] - blockEndPos[2]))

'''
Function: initialize_world(Xsize, Zsize, config = "config.txt", goal_config = "goal_config.txt"):

Purpose: Optional setup helper function that intializes a BlockWorld and goal state (wildcards NOT implemented) from file
'''
def initialize_world(Xsize, Zsize, config = "config.txt", goal_config = "goal_config.txt"):
    print(config)
    bw = BlockWorld(Xsize, Zsize)
    bw.initialize(config)
    #print(bw.dronePos)
    goal = bw.initialize_goal(goal_config) #x+ Xsize //2 (0=5)
    print("initialized")
    return bw, goal


'''
Function: _validMove(bw, newX, newY, newZ)

Purpose: Repurposed from the original BlockWorldSim so that it returns Boolean values instead of raising exceptions
when the new location is invalid - used with the bw_actionF method to rule out invalid movements

'''
def validMove(bw, newX, newY, newZ):
        if max(abs(newX), abs(newZ)) > bw.Xsize /2 or newY > 51:
            return False
        elif newY < 0 or (newY == 0 and bw.attached):
            return False
        elif bw.attached and (bw._rd(newX, newY-1, newZ) not in [0,'drone']):
            return False
        elif bw.attached and newX == 0 and newZ == 0:
            pass
        elif bw._rd(newX, newY, newZ) != 0:
            return False
        return True

if __name__ == '__main__':

    #max_block_height = 2
    Xsize, Zsize =101,101
    #random.seed()
    #arr, ysum = StateCreator.stateCreator(max_block_height, Xsize, Zsize)
    #StateCreator.single_tower_goal_creator(ysum, Xsize, Zsize)

    bw, goal_state = initialize_world(Xsize, Zsize, config = "tiny_config.txt", goal_config="tiny_goal.txt")
    #path = []
    move_goal = ((-4,0,1),(0,0,1))
    start_time = time.time()

    #def aStarSearch(startState, goal, actionsF, takeActionF, goalTestF, hF, fmax= 1000):
    moves, cost, totalNodes = aStar_low_level(bw, move_goal)
    #            aStarSearch(startState, goal, actionsF, takeActionF, goalTestF, hF, fmax= 1000):
    time_taken = time.time() - start_time
    print ("Cost for best path is: {0}, number of nodes explored = {1}, moves are:".format(cost, totalNodes))
    print(moves)
    print("Time required = {0:.15f}".format(time_taken))

    for move in moves:
        if move == 'attach':
            bw.attach()
            print(bw.dronePos)
        elif move == 'detach':
            bw.detach()
            print(bw.dronePos)
        else:
            bw.move(*move)


    #  UNCOMMENT TO GENERATE NEW START OR GOAL STATES
    #StateCreator.single_tower_goal_creator(ysum, Xsize, Zsize)
    #StateCreator.random_goal_creator(arr, ysum, Xsize, Zsize)

    # Old code from assignment 1
    '''bw, goal = initialize_world(goal_config="random_goal_config.txt")

    gtl = get_tower_locations(goal, Xsize, Zsize)
    h = simple_heuristic(bw, goal, gtl)
    print("Initial heuristic for_{0} blocks is {1} ".format(len(bw.state())-1, h))

    path, cost = aStarSearch(bw,bw_movements,takeActionF_bw,lambda s: goal_test_bw(s, goal, gtl),
                            lambda s: simple_heuristic(s, goal, gtl))

    filename = "results_{0}blocks.txt".format(len(bw.state())-1)
    outfile = open(filename, 'a')
    
    outfile.close()
    print('saved results to file ', str(filename))
    '''

