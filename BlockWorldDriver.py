#!/usr/bin/env python
#coding: utf-8

'''
Created on Feb 1, 2018
@author: cs540 team I
'''

from __future__ import (print_function, absolute_import,
                       division, unicode_literals)

try:
    from future_builtins import *   # @UnusedWildImport
    import codecs                   # python-2
    open = codecs.open              # @ReservedAssignment
    input = raw_input               # @ReservedAssignment @UndefinedVariable
    range = xrange                  # @ReservedAssignment @UndefinedVariable
except ImportError:
    pass                            # python-3

from copy import deepcopy

try:
    from org.cs540.team1.BlockWorldSim import BlockWorld
except ImportError:
    from BlockWorldSim import BlockWorld
    
# import sys
# sys.setrecursionlimit(100)
        
def generateGoalState(numBlocks=10):
    goalStates = {}
    goalStates[(40,0,20)] = str('red')
    goalStates[(36,0,36)] = str('green')
#     goalStates[(-49,0,-49)] = str('red')
#     goalStates[(-45,0,-45)] = str('green')
    return goalStates

def getRemainingGoals(state, goalState):
    remaining = []
    currentBlocks = state.items()
    for block in goalState.items():
        if block not in currentBlocks and block[1] != 'drone':
            remaining.append(block)
    return remaining

def getRemainingBlocks(state, goalState):
    remaining = []
    goalBlocks = goalState.items()
    for block in state.items():
        if block not in goalBlocks and block[1] != 'drone':
            remaining.append(block)
    return remaining


def isFinalPosition(currentPos, stopPosition, droneOnly):
    if droneOnly:
        return currentPos == stopPosition
    else:
        return currentPos[0] == stopPosition[0] and \
                currentPos[2] == stopPosition[2]

def getShortestPath(state, start, stop, droneOnly):
    currentPos = start[0]
    x2, y2, z2 = stop[0]
    if droneOnly:     # drone must land on a block to attach
        y2 += 1
    moves = []
    while not isFinalPosition(currentPos, (x2,y2,z2), droneOnly):
        x1, y1, z1 = currentPos
        x = 1 if x2 > x1 else -1 if x2 < x1 else 0
        z = 1 if z2 > z1 else -1 if z2 < z1 else 0
        y = 1 if y2 > y1 else -1 if y2 < y1 else 0

        proposedPos = (currentPos[0] + x, currentPos[1] + y, currentPos[2] + z)        
        while proposedPos in state:
            #print('got blocked at ', currentPos, 'trying to enter',  proposedPos, ' on way to: ', stop)
            if y < 1:
                #print('attempting to climb over')
                y += 1 
            else:
                #print('going straight up')
                x, y, z = 0, 1, 0 
            proposedPos = (currentPos[0] + x, currentPos[1] + y, currentPos[2] + z)  
        moves.append((x,y,z))
        currentPos = proposedPos
    return moves

def getDronePosition(state):
    for k, v in state.items():
        if v == 'drone':
            return (k, v)

def scorePaths(paths):
    for path in paths:
        score = 0.0
        effects = path[3]
        if 'dump' in effects:
            score += 1
        if 'goalState' in effects:
            score += 1 
        score -= 0.004 * len(path[4])  # length of path for drone to pickup
        score -= 0.004 * len(path[5])  # length of path for drone to dropoff
        path[0] = score

def getCoverCount(state, position):
    x,y,z = position[0], position[1], position[2]
    count = 0
    while (x,y,z) in state:
        y += 1
        count += 1
    return count

def openGoalState(state, color, remainingGoals):
    for goal in remainingGoals:
        if goal[1] == color and not isBlockCovered(goal, state):
            return goal
    return None
    

def dumpBlock(state, obstacleBlock, remainingGoals):
    (x, _, z), color = obstacleBlock
    dumpsite = openGoalState(state, color, remainingGoals)
    #print('need to dump ', obstacleBlock)
    if dumpsite is not None:
        #print('got a dump match')
        return dumpsite, 'goalState'
    for i in range(1, 30):
        if -51 < x + i < 51:
            dumpsite = (x+i, 0, z), color
            if dumpsite[0] not in state:
                break 
        if -51 < x - i < 51:
            dumpsite = (x-i, 0, z), color
            if dumpsite[0] not in state:
                break 
        if -51 < z + i < 51:
            dumpsite = (x, 0, z+i), color
            if dumpsite[0] not in state:
                break 
        if -51 < z - i < 51:
            dumpsite = (x, 0, z-i), color
            if dumpsite[0] not in state:
                break 
    return dumpsite,''

def isBlockCovered(block, state):
    x,y,z = block[0]
    if (x,y+1,z) in state:
        #print(str(block) + 'is covered')
        return True
    else:
        #print(str(block) + 'is not covered')     
        return False
     
def uncoverGoal(goal, coverCount, state, remainingGoals, paths):
    effects = [] 
    x, z = goal[0][0], goal[0][2]
    y = goal[0][1] + coverCount - 1
    obstaclePos = (x, y, z)
    block = (obstaclePos, state[obstaclePos])
    dumpsite, sideEffect = dumpBlock(state, block, remainingGoals)
    if sideEffect:
        effects.append(sideEffect)
    pickupPath = getShortestPath(state, getDronePosition(state), block, droneOnly=True)
    dropoffPath = getShortestPath(state, block, dumpsite, droneOnly=False)
    effects.append('dump')
    path = [float('nan'), block, dumpsite, effects, pickupPath, dropoffPath]
    paths.append(path)
     
        
def generatePossiblePaths(state, goalState, remainingGoals):
    # There are three types of moves to select:
    # 1.  Moving block to goal position
    # 2.  Moving obstacle block on or above goal position
    # 3.  1 and 2 combined (this is very desirable)

    paths = []
    effects = []
    for goal in remainingGoals: 
        coverCount = getCoverCount(state, goal[0])
        if coverCount > 0:
            uncoverGoal(goal, coverCount, state, remainingGoals, paths)
        else:  # normal operation for uncovered goal position
            remainingBlocks = getRemainingBlocks(state, goalState) 
            for block in remainingBlocks:
                if block[1] == goal[1] and not isBlockCovered(block, state):
                    pickupPath2 = getShortestPath(state, getDronePosition(state), block, droneOnly=True)
                    dropoffPath2 = getShortestPath(state, block, goal, droneOnly=False)
                    effects.append('goalState')
                    path = [float('nan'), block, goal, deepcopy(effects), pickupPath2, dropoffPath2]
                    paths.append(path)
                    del effects[:]
    if len(paths) == 0:  # something went wrong to move random block anywhere
        remainingBlocks = getRemainingBlocks(state, goalState)
        for block in remainingBlocks:
            if not isBlockCovered(block, state):
                pickupPath2 = getShortestPath(state, getDronePosition(state), block, droneOnly=True)
                dumpsite, _ = dumpBlock(state,  block, remainingGoals)
                dropoffPath2 = getShortestPath(state, block, dumpsite, droneOnly=False)
    return paths

def selectPath(paths):
    bestPath = max(paths)  # will select based on first list element which is heuristic
    return bestPath

def cycleDrone(bw, path):
        pickupPath = path[4]
        dropoffPath = path[5]
        for move in pickupPath:
            bw.move(*move)
        bw.attach()
        for move in dropoffPath:
            bw.move(*move)
        bw.detach()
        return 2 + len(pickupPath) + len(dropoffPath)

def printPathsReport(remainingGoals, paths):
    print('')
    print('remaining goals: ', remainingGoals)
    print('Path Heuristic, Path start, Path end, effects, pickup moves, dropoff moves')
    for path in paths:
        print(path)

def go():    
    goalState = generateGoalState(1)
    bw = BlockWorld()
    bw.initialize('config3.txt')
    numOps = 0
    #bw.plotPoints3d(bw.state(), 'before')
    while True:    
        state = bw.state()
        remainingGoals = getRemainingGoals(state, goalState)      
        if len(remainingGoals) == 0:
            print('\nSolved in', str(numOps), 'operations')
            break
        paths = generatePossiblePaths(state, goalState, remainingGoals)
        scorePaths(paths)
        printPathsReport(remainingGoals, paths)
        path = selectPath(paths)
        numOps += cycleDrone(bw, path)
    #bw.plotPoints3d(bw.state(), 'finshed')
    print('finished running')
    
if __name__ == '__main__':
    go()
