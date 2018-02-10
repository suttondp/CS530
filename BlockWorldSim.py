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
import random
import math

logging.basicConfig(filename='BlockWorldRun.log',level=logging.DEBUG)
Xsize = 101
Ysize = 51
Zsize = 101

class BlockWorld:
    def __init__(self):
        self.arr = np.zeros((Xsize, Ysize, Zsize), dtype=int)
        self.dronePos = [None, None, None]
        self.attached = None
        self.maxY = 0
        self.i2a = {1:str('red'), 2:str('green'), 3:str('blue'), \
                    4:str('yellow'), 9:str('drone'), 0:0}
        self.a2i = {'red':1, 'green':2, 'blue':3, \
                    'yellow':4, 'drone':9, 0:0}
        logging.debug('')
        logging.debug('')
        initString =  'Created block world object at ' + \
            datetime.now().strftime('%Y-%m-%d %H:%M:%S') + '\n'
        logging.debug(initString)
        
    def _validMove(self, newX, newY, newZ):
        if max(abs(newX), newY, abs(newZ)) > 50:
            raise ValueError('position out of bounds')
        elif newY < 0 or (newY == 0 and self.attached): 
            raise ValueError('Y value too low')
        elif self._rd(newX, newY, newZ) != 0:
            raise ValueError('new drone position occupied')
        elif self.attached and self._rd(newX, newY-1, newZ) != 0:
            raise ValueError('new attached block position occupied')
        return True
         
    def _wr(self, x, y, z, val):        
        self.arr[x+Xsize // 2, y, z+Zsize // 2] = val
        if val == self.a2i['drone']:
            self.dronePos = [x, y, z]
    
    def _rd(self, x, y, z):        
        return self.i2a[self.arr[x+Xsize // 2, y, z+Zsize // 2]]
        
    def move(self, dx, dy, dz):
        if max(abs(dx), abs(dy), abs(dz)) > 1:
            raise ValueError('drone only moves to adjacent blocks')
        origX, origY, origZ = origDronePos = self.dronePos
        newX, newY, newZ = origX+dx, origY+dy, origZ+dz
        if self._validMove(newX, newY, newZ):
            if self.attached:
                self._wr(origX, origY-1, origZ, 0)          # clear old attach position
            self._wr(origX, origY, origZ, 0)                # clear old drone position
            self._wr(newX, newY, newZ, self.a2i['drone'])   # write new drone position  
            self.maxY = max(newY, self.maxY)         
            if self.attached:                               # write new attach position
                self._wr(newX, newY-1, newZ, self.a2i[self.attached])
            moveString = 'Drone with attachment ' + str(self.attached) + \
                ' moved from ' + str(origDronePos) + ' to ' + str(self.dronePos) + '\n'
        else:
            moveString = '****  INVALID MOVE ****'
        logging.debug(moveString)
    
                
   
    def attach(self):
        x, y, z = self.dronePos
        if self._rd(x, y-1, z) == 0:
            raise ValueError('cannot attach to drone at ' + str(self.dronePos))
            attachString = 'FAILED to Attach'
        else:
            self.attached = self._rd(x,y-1,z)
            attachString = 'Attached to ' + str(self.attached) + '\n'
        logging.debug(attachString)
            
    def detach(self):
        detachString = 'FAILED Detach'
        x, y, z = self.dronePos
        if self.attached is None:
            raise ValueError('cannot detach if already detached')
        for newY in range(y):
            if self._rd(x, newY, z) == 0:
                self._wr(x, y-1, z, 0)
                self._wr(x, newY, z, self.a2i[self.attached])
                detachString = 'detached into position' + str((x, newY, z)) + '\n'
                self.attached = None
                break
        logging.debug(detachString)
                    
    def speak(self, strng='placeholder'):
        speakString = 'Speak String: ' + strng
        logging.debug(speakString)
        return strng
          
    def _verifyStates(self, blockStates):
        droneCount = 0
        for (x,y,z), block in blockStates.items():
            if max(abs(x), y, abs(z)) > 50 or y < 0:
                raise ValueError('invalid block position: ', (x,y,z))
            if block == 'drone':
                self.dronePos = [x,y,z]
                droneCount += 1
            elif block not in ['red', 'green', 'blue', 'yellow']:
                raise ValueError('invalid block type: ', block)
            elif y > 0: 
                if (x,y-1,z) not in blockStates:
                    raise ValueError('floating block not allowed')
                elif blockStates[(x,y-1,z)] == 'drone':
                    raise ValueError('block on drone not allowed')                 
        if droneCount != 1:
            raise ValueError('only one drone allowed')
        return True
                          
    def initialize(self, filename):
        self.arr = np.zeros((Xsize, Ysize, Zsize), dtype=int)
        blockStates = {}
        with open(filename, 'r', encoding='utf-8') as infile:
            for line in infile:
                x,y,z,block = line.split(',')
                coords = (int(x),int(y),int(z))
                if coords in blockStates:
                    raise ValueError('duplicate position: ', coords)
                blockStates[coords] = block.replace('\n','')
        if self._verifyStates(blockStates):
            for (x,y,z), block in blockStates.items():
                self._wr(x, y, z, self.a2i[block])
                if block != 'drone':  # probably don't care where drone is for printouts
                    self.maxY = max(y, self.maxY)
            initializeString = 'Initialized from file ' + str(filename) + '\n'
        else:
            initializeString = 'Failed to initalize' + '\n'
        logging.debug(initializeString)
                     
    def state(self):
        dic = {}
        yDrone = self.dronePos[1]
        yStart = max(yDrone, self.maxY)
        for y in range(yStart,-1,-1):
            for (x, z), val in np.ndenumerate(self.arr[:,y,:]):                
                if val > 0:
                    dic[(x - Xsize//2, y, z - Zsize//2)] = self.i2a[val]
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




# Z is across, top-left corner is -50,-50

if __name__ == '__main__':
    try:
        bw = BlockWorld()
        #bw.createInitialStates()
        #bw._saveState(bw.state())
        #print("Saved states")
        bw.initialize('config1.txt')
        bw.move(0, 1, 0)
        bw.move(0, 1, 1)
        bw.attach()
        bw.move(0, 0, 1)
        bw.move(0, 0, 1)
        bw.move(0, 0, 1)
        bw.move(1, 0, 1)
        bw.detach()
        bw._saveState(bw.state())
        bw.speak('testing speak')
        bw.move(1, 1, 1)

        #print(bw)
        #bw.printSingleLevel(0)
    except Exception as e:
        logging.error(traceback.format_exc())
        sys.exit()
    else:
        print('\n ****** finished running without errors')
        
