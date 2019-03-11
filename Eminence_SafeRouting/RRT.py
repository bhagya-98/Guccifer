#!/usr/bin/env python
# coding: utf-8

# # Rapidly-exploring Random Trees
# 
# A Rapidly-exploring Random Tree is an algorithm used for robot path planning.
# This is a pygame implementation of RRT. The obstacles considered in this case are static but can easily be modified to make it dynamic.
# 

# In[1]:


import math, sys, pygame, random
from math import *
from pygame import *


# ### Defining a node. 
# 
# Each node has a parent node (except the init node)


class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent


# 'delta' is the max length of the step for each iteration.
# white,black,red,blue etc contain the rgb values used to denote the respective colours.

# In[3]:
# bg = pygame.image.load("/Users/Himanshu/Desktop/unmarked.png")
# gameDisplay.blit(bg,(0,0))



# screen.fill([255, 255, 255])

XDIM = 720
YDIM = 500
windowSize = [XDIM, YDIM]
delta = 5.0
GAME_LEVEL = 1
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0
NUMNODES = 5000
pygame.init()
fpsClock = pygame.time.Clock()      #an object to help track time
screen = pygame.display.set_mode(windowSize)
white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0,180,105

BackGround = pygame.image.load('marked.png').convert()
# 'rectObs' contains the coordinates of the obstacle.

# In[ ]:


count = 0
rectObs = []


# #### Function to calculate distance between two points

# In[ ]:


def dist(p1,p2):    #distance between two points
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


# 

# In[ ]:


def point_circle_collision(p1, p2, radius):
    distance = dist(p1,p2)
    if (distance <= radius):
        return True
    return False


# #### The Steer function:
# 
# Steer: This function provides a control input u [0, T] which
# drives the system from z(0)= z rand to z(T)= z nearest along
# the path z:[0,T] → Z giving z new at a distance 'delta' from
# z nearest towards z rand where Δq is the incremental distance.

# In[ ]:


def step_from_to(p1,p2):
    if dist(p1,p2) < delta:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + delta*cos(theta), p1[1] + delta*sin(theta)


# #### A boolean function to check the collision with the obstacle
# 
# CollisionCheck: This function is used to check collision
# detection of a tree branch and returns true if it lies in
# obstacle free region, i.e., whether a path z:[0, T] lies in the
# Z free for all t=0 to t=T.

# In[ ]:


def collides(p):    #check if point collides with the obstacle
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            return True
    return False


# #### The Sampling function
# 
# Sample: This function generates a random positon z rand
# from search space in obstacle free region Z free .

# In[ ]:


def get_random_clear():
    while True:
        p = random.random()*XDIM, random.random()*YDIM
        noCollision = collides(p)
        if noCollision == False:
            return p


# Initializing the obstacle

# In[ ]:


def init_obstacles(configNum):  #initialized the obstacle
    global rectObs
    rectObs = []
    print("config "+ str(configNum))
    if (configNum == 0):
        rectObs.append(pygame.Rect((XDIM / 2.0 - 50, YDIM / 2.0 - 100),(100,200)))
    if (configNum == 1):
        # rectObs.append(pygame.Rect((410,400),(410,400)))
        rectObs.append(pygame.Rect((300,100),(370,200)))
        rectObs.append(pygame.Rect((700,50),(250,180)))
        rectObs.append(pygame.Rect((0,0),(180,300)))
        rectObs.append(pygame.Rect((300,380),(180,100)))



    if (configNum == 2):
        rectObs.append(pygame.Rect((100,50),(200,150)))

    if (configNum == 3):
        rectObs.append(pygame.Rect((100,50),(200,150)))

    for rect in rectObs:
        pygame.draw.rect(screen, black, rect)


# In[ ]:


def reset():
    global count
    screen.fill(white)
    init_obstacles(GAME_LEVEL)
    count = 0


# In[ ]:


def main():
    global count
    
    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    nodes = []
    reset()

    while True:
        # screen.fill([255, 255, 255, 12])
        # BLACK=(0,0,0)
        # BackGround.set_colorkey(BLACK)
        # BackGround.set_alpha(0)

        BackGround.set_alpha(10)
        screen.blit(BackGround, (0,0))

        if currentState == 'init':
            print('goal point not yet set')
            pygame.display.set_caption('Select Starting Point and then Goal Point')
            fpsClock.tick(40)
        elif currentState == 'goalFound':
            currNode = goalNode.parent
            pygame.display.set_caption('Goal Reached')
            print("Goal Reached")

            
            while currNode.parent != None:
                pygame.draw.line(screen,red,currNode.point,currNode.parent.point,4)
                currNode = currNode.parent
            optimizePhase = True
        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            count = count+1
            pygame.display.set_caption('Performing RRT')
            if count < NUMNODES:
                foundNext = False
                while foundNext == False:
                    rand = get_random_clear()
                    parentNode = nodes[0]
                    for p in nodes:
                        if dist(p.point,rand) <= dist(parentNode.point,rand):
                            newPoint = step_from_to(p.point,rand)
                            if collides(newPoint) == False:
                                parentNode = p
                                foundNext = True

                newnode = step_from_to(parentNode.point,rand)
                nodes.append(Node(newnode, parentNode))
                pygame.draw.line(screen,cyan,parentNode.point,newnode, 6)

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'

                    goalNode = nodes[len(nodes)-1]

                
            else:
                print("Ran out of nodes... :(")
                return;

        #handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    if initPoseSet == False:
                        nodes = []
                        if collides(e.pos) == False:
                            print('initiale point set: '+str(e.pos))

                            initialPoint = Node(e.pos, None)
                            nodes.append(initialPoint) # Start in the center
                            initPoseSet = True
                            pygame.draw.circle(screen, red, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSet == False:
                        print('goal point set: '+str(e.pos))
                        if collides(e.pos) == False:
                            goalPoint = Node(e.pos,None)
                            goalPoseSet = True
                            pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        
        pygame.display.update()

        fpsClock.tick(100000)



# In[ ]:


if __name__ == '__main__':
    main()

