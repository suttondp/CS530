import random
import math
import numpy as np

i2a = {1:str('red'), 2:str('green'), 3:str('blue'), \
                    4:str('yellow'), 9:str('drone'), 0:0}

def stateCreator():

    random.seed()

    filename = "testInitialCreate.txt"
    outfile = open(filename, 'w')
    ysum = 0
    Xsize = 10
    Zsize = 10
    Ysize = 50
    arr = np.zeros((Xsize, Ysize, Zsize), dtype=int)
    for x in range(Xsize):
        for z in range(Zsize):
            # Using two random functions so that the average tower height is around 12.5 instead of 25 for now
            num_blocks = math.floor(random.random() * random.random() * 50)
            ysum += num_blocks
            for y in range(num_blocks):
                color = random.randint(1,4)
                arr[x, y, z] = color
                strng = str(x - Xsize//2) + ',' + str(y) + ',' + str(z - Zsize//2) + ',' + i2a[color] + '\n'
                outfile.write(strng)
    yavg = ysum / (Xsize * Zsize)
    print("Average tower height: ", str(yavg), "\nTotal number of blocks: ", str(ysum))
    saveString = 'saved State to file ' + str(filename) + '\n'
    print(saveString)
    outfile.close()
    return arr, ysum

# We can just specify that we want a column of blocks of a certain color at a certain location and ignore the start state
# However, if we want to use a variant on the generated state, this algorithm essentially mutates the columns in initial state

def goalCreator(arr, numBlocks, Xsize, Zsize):

    random.seed()
    for i in range(numBlocks //2):
        randX1 = random.randint(0,Xsize-1)
        randX2 = random.randint(0,Xsize-1)
        randY1 = math.floor(random.random() * random.random() * 50)
        randY2 = math.floor(random.random() * random.random() * 50)
        randZ1 = random.randint(0,Zsize-1)
        randZ2 = random.randint(0,Zsize-1)

        color1 = arr[randX1, randY1, randZ1]
        color2 = arr[randX2, randY2, randZ2]
        if (color1 != 0 and  color2 != 0 and color1 != color2):
            arr[randX1, randY1, randZ1] = color2
            arr[randX2, randY2, randZ2] = color1

    filename = "testGoalCreate.txt"
    outfile = open(filename, 'w')
    for (x, y, z), color in np.ndenumerate(arr[:,:,:]):
        if color > 0:
            colorstr = i2a[color]
            strng = '{0},{1},{2},{3}\n'.format(str(x - Xsize // 2), str(y), str(z - Zsize // 2), colorstr)
            outfile.write(strng)

    outfile.close()
    print('saved Goal to file ', str(filename))

arr, numBlocks = stateCreator()
goalCreator(arr, numBlocks, 10, 10)

