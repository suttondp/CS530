# Possible operators to add if we use block object
# On(x,y) means block x is on top of block y
# OnTable(x) --- block x is on the table
# Clear(x) --- nothing is on top of block x
# Holding(x) --- robot arm is holding block x
# ArmEmpty() --- robot arm/hand is not holding anything (block in this world)

# How complex are these blocks going to get?  If they're going to have more than one
# property (e.g. color and letter) then they need to be objects

#!/usr/bin/env python
#coding: utf-8

'''
Created on Feb 1, 2018
@author: cs540 team 1
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

import numpy as np
np.set_printoptions(threshold=np.inf)
from datetime import datetime
import sys, logging, traceback
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import math

logging.basicConfig(filename='BlockWorldRun.log',level=logging.DEBUG)
#Xsize = 101
Ysize = 51
#Zsize = 101
class Block:
    def __init__(self, color, position, world):
        self.color = color
        self.position = position
        self.world = world
        self.sub_goal = None

    def assign_sub_goal(self, sub_goal):
        self.sub_goal = sub_goal

    def updatePosition(self, position):
        self.position = position

    def __str__(self):
        return self.color + ":" + str(self.position)

class Drone:
    def __init__(self, position, world):
        self.position = position
        self.world = world
        self.is_attached = False
        self.block_carried = None

    def updatePosition(self, position):#checking move validity is responsibility of the simulator
        if not self.is_attached:
            self.position = position
        else:
            self.position = position
            self.block_carried.position = (self.position[0], self.position[1]-1, self.position[2])

    def grab_block(self, block):
        self.is_attached = True
        self.block_carried = block

    def drop_block(self):
        #ensure actually carrying a block
        if self.is_attached:
            self.is_attached = False
            self.block_carried = None

    def __str__(self):
        return "Drone: " + str(self.position)

class BlockWorld:
    def __init__(self, X=101, Y=101, Z=101):
        self.Xsize = X
        self.Ysize = Y
        self.Zsize = Z
        # self.arr = np.zeros((self.Xsize, Ysize, self.Zsize), dtype=Block)
        self.arr = [[ [None for col in range(self.Xsize)] for col in range(self.Ysize)] for row in range(self.Zsize)]
        self.dronePos = [None, None, None]
        self.drone = None
        self.attached = None
        self.maxY = 0
        logging.debug('')
        logging.debug('')
        initString =  'Created block world object at ' + \
            datetime.now().strftime('%Y-%m-%d %H:%M:%S') + '\n'
        logging.debug(initString)
        
    def _validMove(self, newX, newY, newZ):
        if max(abs(newX), abs(newZ)) > self.Xsize /2 or newY > 51:
            raise ValueError('"{0}, {1}, {2} position out of bounds'.format(newX, newY, newZ))
        elif newY < 0 or (newY == 0 and self.attached):
            raise ValueError('Y value too low')
        elif self.attached and (self._rd(newX, newY-1, newZ) not in [None,self.drone]):
            strng = 'new attached block position occupied ' + str((newX, newY-1, newZ))
            raise ValueError(strng)
        elif self.attached and newX is None and newZ is None:
            pass
        elif self._rd(newX, newY, newZ) is not None:
            strng = 'new drone position occupied ' + str((newX, newY, newZ))
            raise ValueError(strng)
        return True
         
    def _wr(self, x, y, z, val):        
        self.arr[x+self.Xsize // 2][y][z+self.Zsize // 2] = val
        if val == self.drone:
            self.dronePos = [x, y, z]
    
    def _rd(self, x, y, z):        
        return self.arr[x+self.Xsize // 2][y][z+self.Zsize // 2]

    #def sim_to_arr(self, x, y, z):
    #    return x+self.Xsize // 2, y, z+self.Zsize // 2

    def move(self, dx, dy, dz):
        if max(abs(dx), abs(dy), abs(dz)) > 1:
            raise ValueError('drone only moves to adjacent blocks')
        origX, origY, origZ = origDronePos = self.dronePos
        newX, newY, newZ = origX+dx, origY+dy, origZ+dz
        if self._validMove(newX, newY, newZ):
            if self.attached:
                self._wr(origX, origY-1, origZ, None)          # clear old attach position
            self._wr(origX, origY, origZ, None)                # clear old drone position
            self._wr(newX, newY, newZ, self.drone)   # write new drone position
            self.maxY = max(newY, self.maxY)
            self.drone.updatePosition([newX, newY, newZ])
            if self.attached:                               # write new attach position
                self._wr(newX, newY-1, newZ, self.drone.block_carried)

            moveString = 'Drone with attachment ' + str(self.attached) + \
                ' moved from ' + str(origDronePos) + ' to ' + str(self.dronePos) + '\n'
        else:
            moveString = '****  INVALID MOVE ****'
        logging.debug(moveString)
    
                
   
    def attach(self):
        x, y, z = self.dronePos
        if self._rd(x, y-1, z) is None:
            raise ValueError('cannot attach to drone at ' + str(self.dronePos))
            attachString = 'FAILED to Attach'
        else:
            self.attached = self._rd(x,y-1,z)
            self.drone.grab_block(self._rd(x,y-1,z))
            attachString = 'Attached to ' + str(self.attached) + '\n'
        logging.debug(attachString)
            
    def detach(self):
        detachString = 'FAILED Detach'
        x, y, z = self.dronePos
        if self.attached is None:
            raise ValueError('cannot detach if already detached')
        #print("detach function, attached value: " + str(self.attached))
        for newY in range(y):
            if self._rd(x, newY, z) == 0 or self.dronePos[1] == newY+1:
                #if self.dronePos[1] != newY+1:
                #    print("shouldn't execute this right now")
                #    print("{0} = newY, {1} = droneY".format(newY,self.dronePos[1]))
                #    self._wr(x, y-1, z, 0)
                self._wr(x, y-1, z, 0)
                self._wr(x, newY, z, self.attached)
                #print((x, newY, z))
                #print(self._rd(x, newY, z))
                detachString = 'detached into position' + str((x, newY, z)) + '\n'
                self.drone.drop_block()
                self.attached = None
                #print("detach value: " + str(self._rd(x, newY, z)))
                break

        logging.debug(detachString)
                    
    def speak(self, strng='placeholder'):
        speakString = 'Speak String: ' + strng
        logging.debug(speakString)
        return strng
          
    def _verifyStates(self, blockStates):
        droneCount = 0
        for (x,y,z), block in blockStates.items():
            if max(abs(x), abs(z)) > self.Xsize or y >50 or y < 0:
                raise ValueError('invalid block position: ', (x,y,z))
            if block == 'drone':
                self.dronePos = [x,y,z]
                droneCount += 1
            elif y > 0: 
                if (x,y-1,z) not in blockStates:
                    raise ValueError('floating block not allowed')
                elif blockStates[(x,y-1,z)] == 'drone':
                    raise ValueError('block on drone not allowed')                 
        if droneCount != 1:
            raise ValueError('only one drone allowed')
        return True
                          
    def initialize(self, filename):
        # self.arr = np.zeros((self.Xsize, Ysize, self.Zsize), dtype=Block)
        self.arr = [[[None for col in range(self.Xsize)] for col in range(self.Ysize)] for row in range(self.Zsize)]
        blockStates = {}
        with open(filename, 'r', encoding='utf-8') as infile:
            for line in infile:
                try:
                    x,y,z,color = line.split(',')
                except ValueError:
                    x,y,z,color = line.split(' ')
                x = int(x)
                y = int(y)
                z = int(z)

                if self._rd(x,y,z) is not None:
                    raise ValueError('duplicate position: ', coords)
                if color != 'drone':
                    self._wr(x,y,z, Block(color.replace('\n', ''), [x,y,z], self))
                if color == 'drone':
                    self.drone = Drone([x,y,x], self)
                    self._wr(x,y,z, self.drone)
                    self.dronePos = [x,y,z]


    def initialize_goal(self, filename):
        goal = {}
        with open(filename, 'r', encoding='utf-8') as infile:
            for line in infile:
                try:
                    x,y,z,block = line.split(',')
                except ValueError:
                    x,y,z,block = line.split(' ')
                coords = (x,y,z)
                goal[coords] = block.replace('\n', '')
        return goal

    def state(self):
        dic = {}
        yDrone = self.dronePos[1]
        yStart = max(yDrone, self.maxY)
        for y in range(yStart,-1,-1):
            for (x, z), val in np.ndenumerate(self.arr[:][y][:]):
                if val > 0:
                    dic[(x - self.Xsize//2, y, z - self.Zsize//2)] = self.i2a[val]
        logging.debug('generated State\n')
        return dic
    
    def _saveState(self, state, fileName='savedStates.txt'):
        with open(fileName, 'w') as outfile:
            for (x,y,z), block in state.items():
                strng = str(x) + ',' + str(y) + ',' + str(z) + ',' + block + '\n'
                outfile.write(strng)
        saveString = 'saved State to file ' + str(fileName) + '\n'
        print(saveString)
        logging.debug(saveString)
        outfile.close()
        
    def printSingleLevel(self, levelNum):
        print(self.__str__(levelNum, levelNum-1))
           
    def __str__(self, start=None, end=None):
        if start is None:
            start = self.maxY
        if end is None:
            end = -1
        s = ''
        for level in range(start,end,-1):
            s += '\n\nLevel ' + str(level) +':\n'
            s += str(self.arr[:,level,:]).replace(' ', '').replace('\n', '') \
            .replace(']', '\n').replace('0', '.').replace('[', '') \
            .replace('1', 'R').replace('2','G').replace('3','B') \
            .replace('4','Y').replace('9', 'D')
        return s

    def print_blocks(self):
        for x in self.arr:
            for y in x:
                for z in y:
                    if z is not None:
                        print (z)
    def plotPoints3d(self, state, titleString='BlockWorld Plot'):
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D

            maxHeight = 50  # seems a good compromise visually
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            for (x,y,z), color in state.items():
                if color in ['red', 'green', 'blue', 'yellow']:
                    ax.scatter(x, z, y, c=color, marker='s')
                    maxHeight = max(z, maxHeight)  # scale height to > 30 if needed

            ax.set_xlim(-1* self.xSize/2, self.xSize/2)
            ax.set_zlim(self.zSize/2, -1* self.zSize/2)
            ax.set_ylim(0, maxHeight)
            plt.title(titleString)
            ax.set_xlabel('X axis')
            ax.set_ylabel('Z axis')
            ax.set_zlabel('Y axis')
            plt.show()


# Z is across, top-left corner is -50,-50

if __name__ == '__main__':
    bw = BlockWorld()
    bw.initialize('testInitialCreate.txt')
    print ('drone start at: ', bw.dronePos)
    for i in range(9):
        bw.move(0, -1, 0)
        print(bw.state())
    bw.attach()
    for i in range(2):
        bw.move(0,1,0)

    bw.move(-1,0,-1)
    bw.detach()
    print('drone end at: ', bw.dronePos)
    print('Block and drone positions: ')
    bw.print_blocks()
    goal = bw.initialize_goal('testGoalCreate.txt')
    print('Goal state: ', goal)