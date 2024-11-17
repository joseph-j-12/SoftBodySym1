import SoftBodySim
import pygame
from random import randint

mySoftBody = SoftBodySim.SoftBodyObject([720,100], [120,120], 40, 20, 3, 25, 1.9, [0.99,0.99],[0.5,0.99], 0.9, 1)

#set width and height of screen
w = 1365 
h = 715

win = pygame.display.set_mode((w,h))
pygame.display.set_caption('Softbody')
clock = pygame.time.Clock()

def update_colliders(col): #draw the platforms
    for col in colliders:
        p = col[0]
        for i in range(1, len(col)-1):
            pygame.draw.line(win, (200,200,200), p, col[i],6)
            p = col[i]
        pygame.draw.line(win, (200,200,200), p, col[len(col)-1],6)
        pygame.draw.line(win, (200,200,200), col[len(col)-1], col[0],6)

def addCollider(points, collider):
    collider.append(points)
    return collider


colliders = []
colliders = addCollider([[100,600],[400,550],[1000,560],[1100,400],[1200,600],[1200,700],[100,700]],colliders)
colliders = addCollider([[700,250],[900,200],[920,300]],colliders)

done = True
mousepressed = False
mouseInitalPos = [0,0]
while done:
    x,y = pygame.mouse.get_pos()
    sPos = mySoftBody.GetPosition()
    win.fill((20,0,0))
    for event in pygame.event.get(): 
        if event.type == pygame.QUIT:
            done = False
        
    keys_pressed = pygame.key.get_pressed()

    #respond to keypresses
    if keys_pressed[pygame.K_LEFT] or keys_pressed[pygame.K_a]:
         mySoftBody.applyForce([-20, 0])
         mySoftBody.applyRotationalForce(100, 0.03)

    if keys_pressed[pygame.K_RIGHT] or keys_pressed[pygame.K_d]:
         mySoftBody.applyRotationalForce(-100, 0.03)
         mySoftBody.applyForce([20, 0])

    collisions = mySoftBody.GetCollisions()
    up,down,left,right = 0,0,0,0

    for l in range(len(collisions)):
        if collisions[l][1] > sPos[1]:
            down += 1
        elif collisions[l][1] < sPos[1]:
            up += 1
        if collisions[l][0] < sPos[0]:
            left += 1
        elif collisions[l][0] > sPos[0]:
            right += 1
    if keys_pressed[pygame.K_UP] or keys_pressed[pygame.K_w]: #jump only if standing on a platform
        if down != 0:
            mySoftBody.applyImpulse([0, -20], 0.03)

    if pygame.mouse.get_pressed()[0]:
        if mousepressed == False:
            mousepressed = True
            mouseInitalPos = [x,y]

    if not(pygame.mouse.get_pressed()[0]):
        if mousepressed:
            mousepressed = False

    

    lines,pts = mySoftBody.update(colliders, 0.03, [x,y], mousepressed)

    if lines != None:
        l = 0
        for i in lines:
            pygame.draw.line(win, (200,0,100), i[0], i[1],2)
            #pygame.draw.circle(win, (200,0,100), i[0], 6, 6)
            #pygame.draw.circle(win, (200,0,100), i[0], 6, 6)
            l+=1

    for i in pts:
        pygame.draw.circle(win, (255,100,255), i, 4, 4)

    update_colliders(colliders)

    pygame.display.update()
    clock.tick(120) 


        
