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
    input = raw_input               # @ReservedAssignment
    range = xrange                  # @ReservedAssignment
except ImportError:
    pass                            # python-3

try:
    from org.cs540.team1.BlockWorldSim import BlockWorld
except ImportError:
    from BlockWorldSim import BlockWorld
    
# import sys
# sys.setrecursionlimit(100)
        
def generateGoalState(numBlocks=10):
    goalStates = {}
    goalStates[(-40,0,-40)] = str('blue')
    goalStates[(-49,0,-49)] = str('red')
    goalStates[(-45,0,-45)] = str('green')
    return goalStates

def getRemainingGoals(state, goalState):
    remaining = []
    currentBlocks = state.items()
    for block in goalState.items():
        #print('grc block: ', block)
        if block not in currentBlocks and block[1] != 'drone':
            #print('grc block append: ', block)
            remaining.append(block)
    return remaining

def getRemainingBlocks(state, goalState):
    remaining = []
    goalBlocks = goalState.items()
    for block in state.items():
        if block not in goalBlocks and block[1] != 'drone':
            remaining.append(block)
    return remaining


def findAlternatePath(state, currentPos, stop, dx, dz, blockType, isDrone):
    #print('got blocked from ', currentPos, 'try to enter',  proposedPos, ' on way to: ', stop)
    newDx, newDy, newDz = dx, 1, dz
    newMove = (newDx, newDy, newDz)
    newX = max(currentPos[0] + newDx, -50) 
    newY = max(currentPos[1] + newDy, -50) 
    newZ = max(currentPos[2] + newDz, -50)
    newPos = (newX, newY, newZ)
    if newPos in state:
        newDx, newDy, newDz = 0, 1, 0
        newMove = (newDx, newDy, newDz)
        newX = max(currentPos[0] + newDx, -50) 
        newY = max(currentPos[1] + newDy, -50) 
        newZ = max(currentPos[2] + newDz, -50)
        newPos = (newX, newY, newZ)   
    start = (newPos,blockType)
    return [newMove] + getShortestPath(state, start, stop, isDrone)


def getShortestPath(state, start, stop, isDrone=False):
    x1, y1, z1 = currentPos = start[0]
    x2, y2, z2 = stop[0]

    if isDrone:     # drone must land on a block
        y2 += 1
    deltaX, deltaY, deltaZ = x2-x1, y2-y1, z2-z1
    maxDim = max(abs(deltaX), abs(deltaY), abs(deltaZ))
    moves = []
    for _ in range(maxDim):
        x, y, z = 0, 0, 0
        if deltaX < 0:
            x = -1
            deltaX += 1
        elif deltaX > 0:
            x = 1
            deltaX -= 1
        if deltaY < 0:
            y = -1
            deltaY += 1
        elif deltaY > 0:
            y = 1
            deltaY -= 1
        if deltaZ < 0:
            z = -1
            deltaZ += 1
        elif deltaZ > 0:
            z = 1
            deltaZ -= 1
        proposedPos = (currentPos[0] + x, currentPos[1] + y, currentPos[2] + z)        
        if proposedPos in state:
            moves += findAlternatePath(state, currentPos, stop, x, z, start[1], isDrone)
            break
        else:
            moves.append((x,y,z))
        currentPos = proposedPos
    return moves

def getDronePosition(state):
    for k, v in state.items():
        if v == 'drone':
            return (k, v)

def calcHeuristic(block, goal, state):
    dronePos = getDronePosition(state)
    pickupPath = getShortestPath(state, dronePos, block, isDrone=True)
    dropoffPath = getShortestPath(state, block, goal, isDrone=False)
    # currently getting block into correct place counts as +1
    # each move counts as -0.004, so the max traversal (100+100 moves) is still positive
    heuristic = 1.0 - 0.004 * (len(pickupPath) + len(dropoffPath))
    return heuristic, pickupPath, dropoffPath 

def getCoverCount(state, position):
    x,y,z = position[0], position[1], position[2]
    count = 0
    while (x,y,z) in state:
        y += 1
        count += 1
    return count

def dumpBlock(state, obstacleBlock):
    (x, _, z), color = obstacleBlock
    dumpsite = None
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
    #print('dumpsite: ', dumpsite)
    return calcHeuristic(obstacleBlock, dumpsite, state)
        
        
def selectNextMove(state, goalState, remainingGoals):
    maxH = float('-inf')  # heuristic    
    for goal in remainingGoals: 
        coverCount = getCoverCount(state, goal[0])
        if coverCount:
            x, z = goal[0][0], goal[0][2]
            y = goal[0][1] + coverCount - 1
            obstaclePos = (x, y, z)
            obstacleBlock = (obstaclePos, state[obstaclePos])
            #print('obstacleBlock: ', obstacleBlock)
            _, dumpPathPickup, dumpPathDropoff = dumpBlock(state, obstacleBlock)
            return dumpPathPickup, dumpPathDropoff   
        
    remainingBlocks = getRemainingBlocks(state, goalState)       
    bestPathPickup, bestPathDropoff = None, None
    for goal in remainingGoals:
        for block in remainingBlocks:
            if block[1] == goal[1]:
                h, pickupPath, dropoffPath = calcHeuristic(block, goal, state)
                if h > maxH:
                    maxH, bestPathPickup, bestPathDropoff = h, pickupPath, dropoffPath
    return bestPathPickup, bestPathDropoff
    
def cycleDrone(bw, pickupPath, dropoffPath):
        #print('pickupPath: ', pickupPath)
        #print('dropoffPath: ', dropoffPath)
        for move in pickupPath:
            bw.move(*move)
        bw.attach()
        for move in dropoffPath:
            bw.move(*move)
        bw.detach()
        return 2 + len(pickupPath) + len(dropoffPath)
  
def go():    
    goalState = generateGoalState(1)
    bw = BlockWorld()
    bw.initialize('config1.txt')
    numOps = 0
    while True:    
        state = bw.state()
        remainingGoals = getRemainingGoals(state, goalState)
        print('remaining goals: ', remainingGoals)
        if len(remainingGoals) == 0:
            print('solved in', str(numOps), 'operations')
            return
        pickupPath, dropoffPath = selectNextMove(state, goalState, remainingGoals)
        numOps += cycleDrone(bw, pickupPath, dropoffPath)

if __name__ == '__main__':
    go()
    