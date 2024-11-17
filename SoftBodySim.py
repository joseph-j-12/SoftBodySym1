from math import *
import copy

class SoftBodyObject:
    def __init__(self, pos, size, resolution, gravity, mass, stiffness, damping, drag, friction, selfCollisionBounce,platformbounce):
        self.pos = pos
        self.stiffness = stiffness
        self.gravity = gravity
        self.damping = damping
        self.__size = size
        self.__resolution = resolution
        self.__stDist = resolution
        self.__diDist = (sqrt(2*(resolution**2)))
        self.drag = drag
        self.friction = friction
        self.selfCollisionBounce = selfCollisionBounce
        self.platformBounce = platformbounce
        self.pAvg = [pos[0]+(size[0]/2), pos[1]+(size[1]/2)]
        self.rotAvg = 0
        self.points = []
        self.__avgVel = [0,0]

        self.nextForce = [0,0]
        self.__collisions = []

        for i in range(0, size[0], resolution):
            for j in range(0, size[1], resolution):
                self.points.append({"pos":[pos[0]+i, pos[1]+j], "v":[0,0], "f":[0,0], "m":mass, "connected":[]})
                ptsLen = len(self.points)
    def __dist(self, p1, p2):
        return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    def __norm(self, dx, dy):
        dist = self.__dist([0, 0], [dx, dy])
        if not(dist == 0):
            return [dx/dist, dy/dist]
        else:
            return[0,0]

    def __lineNorm(self, p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        nrm = self.__norm(dx,dy)
        return nrm[0],nrm[1]

    def __line_intersection(self,line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            pass#raise Exception('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y
    
    def __ccw(self,A,B,C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def __intersect(self,A,B,C,D):
        return self.__ccw(A,C,D) != self.__ccw(B,C,D) and self.__ccw(A,B,C) != self.__ccw(A,B,D)

    def __GetLineIntersection(self,p1,p2,p3,p4):
        intersecting = self.__intersect(p1,p2,p3,p4)
        if intersecting:
            ix, iy = self.__line_intersection([p1,p2], [p3,p4])
            return ix,iy
        else:
            return 'n','n'

    def GetIndexFromXY(self, pos2):
        pos = pos2
        if (pos[0] == 0 and pos[1] == 0):
            return 0
        elif pos[0] == 0:
            return pos[1]
        elif pos[1] == 0:
            #pos[0] += -1
            #pos[1] += -1
            return int(pos[0]*self.__size[1]//self.__resolution)
        else:
           # pos[0] += -1
            #pos[1] += -1
            return int(pos[0]*self.__size[1]//self.__resolution) + pos[1]
    
    def GetXYFromIndex(self, index):
        #print(index//(self.__size[1]//self.__resolution))
        return [int(index//(self.__size[1]//self.__resolution)), index%int(self.__size[1]/self.__resolution)]

    def GetBodyPoints(self, resolution):
        returnPoints = []

        for i in range(0, int((self.__size[0]/self.__resolution)), resolution):
            for j in range(0, int((self.__size[1]/self.__resolution)), resolution):
                returnPoints.append(self.points[int(self.GetIndexFromXY([i,j]))]["pos"])
        #print(self.GetIndexFromXY([1,0]))
        return returnPoints
    
    def __changeVel(self, index, deltaTime):
        self.points[index]["v"][0] += (self.points[index]["f"][0]/self.points[index]["m"]) * deltaTime
        self.points[index]["v"][1] += (self.points[index]["f"][1]/self.points[index]["m"]) * deltaTime

    def __changePos(self, index, deltaTime):
        self.points[index]["pos"][0] += (self.points[index]["v"][0] * deltaTime)
        self.points[index]["pos"][1] += (self.points[index]["v"][1] * deltaTime)

    def __closestpt(self, point, line1, line2):
        lineLength = self.__dist(line1, line2)
        distFromLine = (line2[0]-line1[0])*(line1[1]-point[1]) - ((line1[0]-point[0])-(line2[1]-line1[1]))/lineLength

        x1, y1 = line1[0], line1[1]
        x2, y2 = line2[0], line2[1]
        x3, y3 = point[0], point[1]
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return [x1+a*dx, y1+a*dy], self.__dist([x1+a*dx, y1+a*dy], point)
        
    def __line2Pt(self, line1, line2, pt):
        dx = line2[0] - line1[0]
        dy = line2[1] - line1[1]
        dr2 = float(dx ** 2 + dy ** 2)

        lerp = ((pt[0] - line1[0]) * dx + (pt[1] - line1[1]) * dy) / dr2
        if lerp < 0:
            lerp = 0
        elif lerp > 1:
            lerp = 1

        x = lerp * dx + line1[0]
        y = lerp * dy + line1[1]

        _dx = x - pt[0]
        _dy = y - pt[1]
        square_dist = _dx ** 2 + _dy ** 2
        return sqrt(square_dist)

    #def __shortest_dist_to_point(self, line, other_point):
        #return math.sqrt(self.__sq_shortest_dist_to_point(other_point))
    def __getAngle(self, p1, p2):
        dx = p2[0]-p1[0]
        dy = p2[1]-p1[1]
        rd = atan2(dy, dx)
        angle = degrees(rd)
        if (angle < 0):
            return (angle)+360
        else:
            return degrees(rd)

    def applyForce(self, force):
        self.nextForce = force.copy()
    
    def applyImpulse(self, force, deltaTime):
        self.nextForce = [force[0]/deltaTime,force[1]/deltaTime]

    def GetPosition(self):
        return self.pAvg

    def GetRotation(self):
        self.rotAvg = 45
        roationChecks = 0
        for i in range(len(self.points)):
            pos = copy.deepcopy(self.points[i]["pos"])
            pAvg = self.pAvg.copy()
            requiredPos = [pAvg[0]-self.__size[0]/2, pAvg[1]-self.__size[1]/2]
            gridx, gridy = self.GetXYFromIndex(i)
            requiredPos[0] += gridx*self.__resolution+self.__resolution/2
            requiredPos[1] += gridy*self.__resolution+self.__resolution/2
            pAngle = self.__getAngle(self.points[i]["pos"], pAvg)
            reqAngle = self.__getAngle(requiredPos, pAvg)
            angleChange = reqAngle-pAngle
            if angleChange < 0:
                angleChange += 360
            if not(gridx == (self.__size[0]/self.__resolution-1)/2 and gridy == (self.__size[1]/self.__resolution-1)/2):
                self.rotAvg += (angleChange)
                roationChecks += 1
                
        self.rotAvg = self.rotAvg/roationChecks
        return self.rotAvg

    def GetCollisions(self):
        return copy.deepcopy(self.__collisions)

    def applyRotationalForce(self,rotationalForce, deltaTime):
        for i in range(len(self.points)):
            self.points[i]["f"] = [0,0]
            pos = copy.deepcopy(self.points[i]["pos"])
            pAvg = self.pAvg.copy()
            requiredPos = pos
            gridx, gridy = self.GetXYFromIndex(i)
            reqAngle = self.__getAngle(requiredPos, pAvg)
            pRad = self.__dist(pAvg, requiredPos)
            requiredPos[0] = pAvg[0] + (pRad*(cos(radians(reqAngle+rotationalForce))))
            requiredPos[1] = pAvg[1] + (pRad*(sin(radians(reqAngle+rotationalForce))))

            length = self.__dist(self.points[i]["pos"], requiredPos)

            nx,ny = self.__lineNorm(self.points[i]["pos"], requiredPos)
                
            dvx = 0-self.points[i]["v"][0]
            dvy = 0-self.points[i]["v"][1]
            dot1 = (nx*dvx)+(ny*dvy)
        
            f1 = dot1 * self.damping
            sprf = length-pRad
        
            sprf = sprf*self.stiffness
            f1 = f1+sprf
            fx = f1 * nx
            fy = f1 * ny
            self.points[i]["f"][0] = fx
            self.points[i]["f"][1] = fy
            self.__changeVel(i, deltaTime)

    def rotate(self,rotation, deltaTime):
        for i in range(len(self.points)):
            self.points[i]["f"] = [0,0]
            pos = copy.deepcopy(self.points[i]["pos"])
            pAvg = self.pAvg.copy()
            requiredPos = pos
            gridx, gridy = self.GetXYFromIndex(i)

            reqAngle = self.__getAngle(requiredPos, pAvg)
            pRad = self.__dist(pAvg, requiredPos)
            requiredPos[0] = pAvg[0] + (pRad*(cos(radians(reqAngle-rotation))))
            requiredPos[1] = pAvg[1] + (pRad*(sin(radians(reqAngle-rotation))))

            self.points[i]["pos"] = requiredPos

    def setRotation(self,newRotation):
        for i in range(len(self.points)):
            self.points[i]["f"] = [0,0]
            pos = copy.deepcopy(self.points[i]["pos"])
            pAvg = self.pAvg.copy()
            requiredPos = [pAvg[0]-self.__size[0]/2, pAvg[1]-self.__size[1]/2]
            gridx, gridy = self.GetXYFromIndex(i)
            requiredPos[0] += gridx*self.__resolution+self.__resolution/2
            requiredPos[1] += gridy*self.__resolution+self.__resolution/2
            reqAngle = self.__getAngle(requiredPos, pAvg)
            pRad = self.__dist(pAvg, requiredPos)
            requiredPos[0] = pAvg[0] + (pRad*(cos(radians(reqAngle+newRotation))))
            requiredPos[1] = pAvg[1] + (pRad*(sin(radians(reqAngle+newRotation))))
            self.points[i]["pos"] = requiredPos
            
            
    def update(self, colliderLines, deltaTime, mousep, mousePressed):
        lines = []
        returnpoints = []
        #print(self.points[0]["pos"])
        self.pAvg = [0,0]#copy.deepcopy(self.points[0]["pos"])
        self.__avgVel = [0,0]
        self.__collisions = []
        #print(self.points[0]["pos"])
        for i in range(len(self.points)):
            self.points[i]["f"] = self.nextForce.copy()
            gravityForce = self.gravity
            self.points[i]["v"][1] += gravityForce*deltaTime
            mx, my = self.GetXYFromIndex(i)
            nearby = []
            #print(self.GetIndexFromXY([1,0]))
            #get nearby points connected by springs
            if mx == 0 and my == 0:
                nearby.append([self.GetIndexFromXY([1,0]), self.__stDist])
                nearby.append([self.GetIndexFromXY([0,1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([1,1]), self.__diDist])

            elif mx == 0 and not(my > (self.__size[1]//self.__resolution)-2) and my != 0:
                nearby.append([self.GetIndexFromXY([0,my+1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([0,my-1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([1,my+1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([1,my-1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([1,my]), self.__stDist])
            
            elif mx == 0 and (my > (self.__size[1]//self.__resolution)-2):
                nearby.append([self.GetIndexFromXY([0,my-1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([1,my]), self.__stDist])
                nearby.append([self.GetIndexFromXY([1,my-1]), self.__diDist])
            
            elif my == 0 and not(mx > (self.__size[0]//self.__resolution)-2):
                nearby.append([self.GetIndexFromXY([mx+1,0]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,0]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx+1,1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([mx-1,1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([mx,1]), self.__stDist])
            
            elif my == 0 and (mx > (self.__size[0]//self.__resolution)-2):
                nearby.append([self.GetIndexFromXY([mx-1,0]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx,1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,1]), self.__diDist])

            elif (mx > (self.__size[0]//self.__resolution)-2) and not(my > (self.__size[1]//self.__resolution)-2) and not my == 0:
                #nearby.append([self.GetIndexFromXY([mx-1,my]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx,my+1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx,my-1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,my-1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([mx-1,my+1]), self.__diDist])

            elif (my > (self.__size[1]//self.__resolution)-2) and not(mx > (self.__size[0]//self.__resolution)-2) and not mx == 0:
                nearby.append([self.GetIndexFromXY([mx,my-1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx+1,my]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,my]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,my-1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([mx+1,my-1]), self.__diDist])

            elif (my > (self.__size[1]//self.__resolution)-2) and (mx > (self.__size[0]//self.__resolution)-2):
                nearby.append([self.GetIndexFromXY([mx,my-1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,my]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,my-1]), self.__diDist])
                #nearby.append([self.GetIndexFromXY([mx-1,my-1]), self.__diDist])
                #nearby.append([self.GetIndexFromXY([mx+1,my-1]), self.__diDist])
            
            else:
                nearby.append([self.GetIndexFromXY([mx,my+1]), self.__stDist]) #straingt distance
                nearby.append([self.GetIndexFromXY([mx,my-1]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx+1,my]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,my]), self.__stDist])
                nearby.append([self.GetIndexFromXY([mx-1,my-1]), self.__diDist]) #diagonal distance
                nearby.append([self.GetIndexFromXY([mx+1,my-1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([mx-1,my+1]), self.__diDist])
                nearby.append([self.GetIndexFromXY([mx+1,my+1]), self.__diDist])

            #print(mx,my)
            #print(nearby)
            #print(i)
            #go thru the list of nearby points
            for j in range(len(nearby)):
                restLen = nearby[j][1]
                nearbyindex = nearby[j][0]
                stiff = self.stiffness
                #print(nearbyindex)
                
                lines.append([self.points[i]["pos"], self.points[nearbyindex]["pos"]])
                nx,ny = self.__lineNorm(self.points[i]["pos"], self.points[nearbyindex]["pos"])
                nx2, ny2 = self.__lineNorm(self.points[nearbyindex]["pos"], self.points[i]["pos"])
                dist1 = self.__dist([self.points[nearbyindex]["pos"][0],self.points[nearbyindex]["pos"][1]],[self.points[i]["pos"][0],self.points[i]["pos"][1]])
                dvx = self.points[nearbyindex]["v"][0]-self.points[i]["v"][0]
                dvy = self.points[nearbyindex]["v"][1]-self.points[i]["v"][1]
                dot1 = (nx*dvx)+(ny*dvy)

                #print(restLen, dist1)
                #if restLen != 40:
                    #print([self.points[nearbyindex]["v"][0],self.points[nearbyindex]["v"][1]],[self.points[i]["v"][0],self.points[i]["v"][1]])
                f1 = dot1 * self.damping
                sprf = dist1-restLen
                if dist1 < restLen/2:
                    sprf = sprf/(deltaTime*2)
                sprf = sprf*self.stiffness
                f1 = f1+sprf
                fx = f1 * nx
                fy = f1 * ny
                fx = (fx)
                fy = (fy)
                fx2 = f1 * nx2
                fy2 = f1 * ny2
                fx2 = (fx2)
                fy2 = (fy2)
                self.points[i]['f'][0] += fx
                self.points[i]['f'][1] += fy
                #print(self.points[i]['pos'])
                self.points[nearbyindex]['f'][0] += fx2
                self.points[nearbyindex]['f'][1] += fy2
                self.__changeVel(nearbyindex, deltaTime)
                #self.__changePos(nearbyindex, deltaTime)
                self.points[nearbyindex]['f'] = [0,0]
                
            #collision between points, so that points dont overlap
            for j in range(len(self.points)):
                if j != i:
                    dist1 = self.__dist([self.points[j]["pos"][0],self.points[j]["pos"][1]],[self.points[i]["pos"][0],self.points[i]["pos"][1]])
                    if dist1 < (self.__resolution/2):
                        depth = self.__resolution/2-dist1
                        if dist1 == 0:
                            self.points[i]["pos"][0] += 1
                            dist1 = 1
                            depth = self.__resolution/2-dist1
                        if depth > 0:
                            nx2, ny2 = self.__lineNorm(self.points[i]["pos"], self.points[j]["pos"])
                            depth = depth/2
                            self.points[i]["pos"][0] += nx2*(0-depth)
                            self.points[i]["pos"][1] += ny2*(0-depth)
                            self.points[j]["pos"][0] += nx2*(depth)
                            self.points[j]["pos"][1] += ny2*(depth)

                            rvx = (self.points[j]["v"][0]-self.points[i]["v"][0])
                            rvy = (self.points[j]["v"][1]-self.points[i]["v"][1])
                            rv = (nx2*rvx)+(ny2*rvy)

                            #bounce = 1
                            rv = (-1-self.selfCollisionBounce)*(rv/2)

                            self.points[i]["f"][0] += (nx2*(0-rv))
                            self.points[i]["f"][1] += (ny2*(0-rv))

                            self.points[j]["f"][0] += (nx2*(rv))
                            self.points[j]["f"][1] += (ny2*(rv))
                            self.__changeVel(j, deltaTime)

            #collision with polygons
            for l in range(len(colliderLines)):
                col = colliderLines[l]
                l1p1 = [self.points[i]["pos"][0],self.points[i]["pos"][1]]
                l1p2 = [self.points[i]["pos"][0]+10000,self.points[i]["pos"][1]+1000]
                collisions = 0
                pclx,pcly = 0,0
                cLine1 = [0,0]
                cLine2 = [0,0]
                closestpt = [0,0]
                clDist = 9999

                for m in range(len(col)):
                    l2p1 = col[m]
                    l2p2 = [0,0]

                    if m == len(col)-1:
                        l2p2 = col[0]
                    else:
                        l2p2 = col[m+1]

                    point1, dist1 = self.__closestpt(self.points[i]["pos"], l2p1, l2p2)
                    dist1 = self.__line2Pt(l2p1, l2p2, self.points[i]["pos"])

                    if dist1 < clDist:
                        clDist = dist1
                        closestpt = point1
                    clx, cly = self.__GetLineIntersection(l1p1,l1p2,l2p1,l2p2)

                    if clx != 'n':
                        pclx,pcly = clx,cly
                        cLine1 = l2p1
                        cLine2 = l2p2
                        collisions += 1

                if not(int(collisions%2) == 0) and collisions != 0:
                    clp = [0,0]
                    self.points[i]['v'][1] = self.points[i]['v'][1]*self.friction[1]
                    self.points[i]['v'][0] = self.points[i]['v'][0]*self.friction[0]

                    prevPos = self.points[i]["pos"]
                    self.points[i]["pos"] = closestpt
                    self.__collisions.append(closestpt)

                    nx1, ny1 = self.__lineNorm(prevPos, closestpt)
                    dot = self.platformBounce#(self.points[i]["v"][0]*nx1)-(self.points[i]["v"][1]*ny1)

                    self.points[i]["f"][0] += nx1*dot
                    self.points[i]["f"][1] += ny1*dot
                    for j in range(len(nearby)):
                        nearbyindex = nearby[j][0]
                        self.points[nearbyindex]["f"][0] += nx1*dot
                        self.points[nearbyindex]["f"][1] += ny1*dot
                        self.__changeVel(nearbyindex,deltaTime)
                        self.points[nearbyindex]["f"] = [0,0]

            #mouse
            dist = self.__dist(self.points[i]["pos"], mousep)
            if dist < 10:
                if mousePressed:
                    #self.points[i]["v"][1] += 1000
                    self.points[i]["pos"][1] += mousep[1]-self.points[i]["pos"][1]
                    self.points[i]["pos"][0] += mousep[0]-self.points[i]["pos"][0]
                    self.points[i]["v"] = [0,0]
                    self.points[i]["f"] = [0,0]
            
            self.__changeVel(i, deltaTime)
            self.points[i]["v"][0] = ((self.points[i]["v"][0]*self.drag[0]))
            self.points[i]["v"][1] = ((self.points[i]["v"][1]*self.drag[1]))
            self.__changePos(i, deltaTime)
            
            returnpoints.append(self.points[i]["pos"].copy())
            
            self.pAvg[0] += (self.points[i]["pos"][0])
            self.pAvg[1] += (self.points[i]["pos"][1])

            self.__avgVel[0] += self.points[i]["v"][0]
            self.__avgVel[1] += self.points[i]["v"][1]
            
        self.__avgVel[0] /= len(self.points)
        self.__avgVel[1] /= len(self.points)
        self.pAvg[0] = self.pAvg[0]/(len(self.points))
        self.pAvg[1] = self.pAvg[1]/(len(self.points))

        #get average rotation the whole body
        rotation = False
        if rotation == True:
            self.rotAvg = 45
            roationChecks = 0
            for i in range(len(self.points)):
                pos = copy.deepcopy(self.points[i]["pos"])
                pAvg = self.pAvg.copy()
                requiredPos = [pAvg[0]-self.__size[0]/2, pAvg[1]-self.__size[1]/2]
                gridx, gridy = self.GetXYFromIndex(i)
                requiredPos[0] += gridx*self.__resolution+self.__resolution/2
                requiredPos[1] += gridy*self.__resolution+self.__resolution/2
                #returnpoints.append(requiredPos.copy())
                #pRad = self.__dist(pAvg, self.points[i]["pos"])
                pAngle = self.__getAngle(self.points[i]["pos"], pAvg)
                reqAngle = self.__getAngle(requiredPos, pAvg)
                angleChange = reqAngle-pAngle
                if angleChange < 0:
                    angleChange += 360
                #print(i, angleChange)
                if not(gridx == (self.__size[0]/self.__resolution-1)/2 and gridy == (self.__size[1]/self.__resolution-1)/2):
                    self.rotAvg += (angleChange)
                    roationChecks += 1
                
            self.rotAvg = self.rotAvg/roationChecks

            requiredPos = [self.pAvg[0]-self.__size[0]/2, self.pAvg[1]-self.__size[1]/2]
            gridx, gridy = self.GetXYFromIndex(8)
            requiredPos[0] += gridx*self.__resolution+self.__resolution/2
            requiredPos[1] += gridy*self.__resolution+self.__resolution/2
            #print(self.__getAngle(self.points[8]["pos"], self.pAvg), self.__getAngle(requiredPos, pAvg), self.__getAngle(requiredPos, pAvg)-self.__getAngle(self.points[8]["pos"], self.pAvg))
            #print(self.__getAngle(requiredPos, pAvg)-self.__getAngle(self.points[8]["pos"], self.pAvg))
            #self.rotAvg = 0
            print(self.rotAvg)
            for i in range(len(self.points)):
                pos = copy.deepcopy(self.points[i]["pos"])
                pAvg = self.pAvg.copy()
                requiredPos = [pAvg[0]-self.__size[0]/2, pAvg[1]-self.__size[1]/2]
                gridx, gridy = self.GetXYFromIndex(i)
                requiredPos[0] += gridx*self.__resolution+self.__resolution/2
                requiredPos[1] += gridy*self.__resolution+self.__resolution/2
                requiredoffsetX = requiredPos[0]-pAvg[0]
                requiredoffsetY = requiredPos[1]-pAvg[1]
                reqAngle = self.__getAngle(requiredPos, pAvg)
                pRad = self.__dist(pAvg, requiredPos)
                requiredPos[0] = pAvg[0] + (pRad*(sin(radians(reqAngle+self.rotAvg))))
                requiredPos[1] = pAvg[1] + (pRad*(cos(radians(reqAngle+self.rotAvg))))

                length = self.__dist(self.points[i]["pos"], requiredPos)

                nx,ny = self.__lineNorm(self.points[i]["pos"], requiredPos)
                
                dvx = 0-self.points[i]["v"][0]
                dvy = 0-self.points[i]["v"][1]
                dot1 = (nx*dvx)+(ny*dvy)
        
                f1 = dot1 * self.damping
                sprf = length-pRad
        
                sprf = sprf*self.stiffness
                f1 = f1+sprf
                fx = f1 * nx
                fy = f1 * ny
                self.points[i]["f"][0] = fx
                self.points[i]["f"][1] = fy
                self.__changeVel(i, deltaTime)
                
        self.nextForce = [0,0]
        return lines,returnpoints
        
            

    
