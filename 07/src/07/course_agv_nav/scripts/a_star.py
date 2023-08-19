import numpy as np
import jps

class AStarPlanner:
    def __init__(self,plan_grid_size,plan_robot_radius):
        self.plan_grid_size = plan_grid_size
        self.plan_robot_radius = plan_robot_radius

    def planning(self,ox,oy,sx,sy,gx,gy,minox,minoy,maxox,maxoy):

        useJPS = True

        startx,starty = sx,sy
        goalx,goaly = gx,gy

        oxtem = np.array(ox)
        oxtem = np.floor_divide(oxtem,self.plan_grid_size)
        ox = oxtem.tolist()
        ox = [int(x) for x in ox]
        oytem = np.array(oy)
        oytem = np.floor_divide(oytem,self.plan_grid_size)
        oy = oytem.tolist()
        oy = [int(y) for y in oy]
        sxtem = np.array(sx)
        sxtem = np.floor_divide(sxtem,self.plan_grid_size)
        sx = sxtem.tolist()
        sx = int(sx)
        sytem = np.array(sy)
        sytem = np.floor_divide(sytem,self.plan_grid_size)
        sy = sytem.tolist()
        sy = int(sy)
        gxtem = np.array(gx)
        gxtem = np.floor_divide(gxtem,self.plan_grid_size)
        gx = gxtem.tolist()
        gx = int(gx)
        gytem = np.array(gy)
        gytem = np.floor_divide(gytem,self.plan_grid_size)
        gy = gytem.tolist()
        gy = int(gy)
        minox = np.floor_divide(minox,self.plan_grid_size)
        minox = int(minox)
        minoy = np.floor_divide(minoy,self.plan_grid_size)
        minoy = int(minoy)
        maxox = np.floor_divide(maxox,self.plan_grid_size)
        maxox = int(maxox)
        maxoy = np.floor_divide(maxoy,self.plan_grid_size)
        maxoy = int(maxoy)

        boardnum = self.plan_robot_radius / self.plan_grid_size 
        col = int(maxoy - minoy + 1)
        row = int(maxox - minox + 1)
        oxary = np.array(ox) - minox # obstacle
        oyary = np.array(oy) - minoy
        sxary = sx - minox # start
        syary = sy - minoy
        spoint = np.array([sxary,syary])
        gxary = gx - minox # goal
        gyary = gy - minoy
        gpoint = np.array([gxary,gyary])
        map = np.zeros((row,col))
        map[oxary,oyary] = -1
        map[sxary,syary] = 1
        proceed = np.full((2,row,col),np.nan)
        f = np.full((row,col),np.inf)
        g = np.full((row,col),np.inf)

        bx,by = [],[]
        for i in range(row):
            for j in range(col):
                if map[i, j] != -1:
                    distx = oxary - i
                    disty = oyary - j
                    dist = min(np.sqrt(distx**2 + disty**2))
                    if dist <= boardnum:
                        bx.append(i)
                        by.append(j)
        bxary = np.array(bx)
        byary = np.array(by)
        map[bxary,byary] = -1
        print(map)
        
        # JPS method
        if useJPS:
            jpsmap = map
            jpsmap[jpsmap != -1] = 0
            jpsmap[jpsmap == -1] = 1
            # print "jpsmap", jpsmap.tolist()
            rx,ry = self.JPS(jpsmap.tolist(),spoint.tolist(),gpoint.tolist(),row,col)
            new_length = 2*len(rx) - 1
            newrx,newry = np.zeros(new_length),np.zeros(new_length)
            insert_index = np.arange(0, new_length, 2)
            newrx[insert_index] = rx
            newry[insert_index] = ry
            mean_rx = np.mean(np.vstack((rx[:-1], rx[1:])), axis=0)
            mean_ry = np.mean(np.vstack((ry[:-1], ry[1:])), axis=0)
            newrx[insert_index[0:-1] + 1] = mean_rx
            newry[insert_index[0:-1] + 1] = mean_ry
            rx = newrx
            ry = newry
            rx = ((np.array(rx) + minox)* self.plan_grid_size).tolist()
            ry = ((np.array(ry) + minoy)* self.plan_grid_size).tolist()
            rx[-1],ry[-1] = startx, starty
            rx[0],ry[0] = goalx, goaly
            self.show = np.vstack( (np.array(rx),np.array(ry)) )
            print(self.show.T)
            print "JPS return!"
            return rx,ry

        print "A* method"
        # A* method
        openlist = [(sxary,syary)]
        g[sxary,syary] = 0
        f[sxary,syary] = np.linalg.norm(spoint-gpoint)
        # the heuristic is 2-norm of (cuerrentpoint - goalpoint) 

        rx,ry = [],[]
        while len(openlist)>0:
            cx,cy = min(openlist, key=lambda x: f[x[0],x[1]])
            cpoint = np.array([cx,cy])
            openlist.remove((cx,cy))
            #print("cx cy oplist",cx,cy,openlist)
            
            if (cpoint == gpoint).all():
                rs = np.array([[cx,cy]])
                while True:
                    if (rs[-1,:]==spoint.flatten()).all():
                        break
                    rs = np.vstack((rs,proceed[ :,int(rs[-1,0]),int(rs[-1,1]) ]))
                rx = rs[:,0]
                ry = rs[:,1]
                # rx = rx[::-1]
                # ry = ry[::-1]
                break
            
            # if use 8, A* will find path diagnally
            mapnum = 4
            if mapnum == 8:            
                for dx in range(-1,2):
                    for dy in range(-1,2):
                        nx = cx + dx
                        ny = cy + dy
                        npoint = np.array([nx,ny])
                        if nx<0 or ny <0 or nx>row or ny>col:
                            continue
                        if map[nx,ny]==-1:
                            # is obstacle
                            continue
                        cost = np.sqrt(dx**2 + dy**2)

                        # odists = np.hypot(oxary-nx,oyary-ny)
                        # args = np.argsort(odists)
                        # odist = odists[args[0:500]]
                        # tinycost = np.sum(1.0/(odist**4)) * 80
                        # # tinycost = 0
                        # ng = g[cx,cy] + cost
                        # nf = ng + np.linalg.norm(npoint-gpoint) + tinycost
                        # print(ng + np.linalg.norm(npoint-gpoint), nf)

                        ng = g[cx,cy] + cost
                        nf = ng + np.linalg.norm(npoint-gpoint)
                        if map[nx,ny]==0:
                            openlist.append((nx,ny))
                            g[nx,ny] = ng
                            f[nx,ny] = nf
                            proceed[:,nx,ny] = np.array([cx,cy])
                            map[nx,ny] = 1
                        elif nf < f[nx,ny]:
                            g[nx,ny] = ng
                            f[nx,ny] = nf
                            proceed[:,nx,ny] = np.array([cx,cy])
            else:
                candidate = [[cx-1,cy],[cx+1,cy],[cx,cy-1],[cx,cy+1]]
                for can in candidate:
                    print(can)
                    cost = 1
                    nx = can[0]
                    ny = can[1]
                    npoint = np.array(can)
                    if nx<0 or ny <0 or nx>row or ny>col:
                        continue
                    if map[nx,ny]==-1:
                        # is obstacle
                        # print("isobs")
                        continue
                    # odists = np.hypot(oxary-nx,oyary-ny)
                    # args = np.argsort(odists)
                    # odists = odists[args[0:100]]
                    # tinycost = np.sum(1.0/odists**2) * 0.05
                    tinycost = 0
                    ppoint = proceed[:,cx,cy]
                    if ppoint[0]==npoint[0] or ppoint[1]==npoint[1]:
                        tinycost += 1
                    else:
                        tinycost += 0
                    ng = g[cx,cy] + cost
                    nf = ng + np.linalg.norm(npoint-gpoint) + tinycost
                    # ng = g[cx,cy] + cost
                    # nf = ng + np.linalg.norm(npoint-gpoint)
                    if map[nx,ny]==0:
                        openlist.append((nx,ny))
                        g[nx,ny] = ng
                        f[nx,ny] = nf
                        proceed[:,nx,ny] = np.array([cx,cy])
                        map[nx,ny] = 1
                    elif nf < f[nx,ny]:
                        g[nx,ny] = ng
                        f[nx,ny] = nf
                        proceed[:,nx,ny] = np.array([cx,cy])
                    
        if rx==[] and ry==[]:
            print("no feasible path found")
        else:
            # prtrx,prtry = np.array(rx[::-1]),np.array(ry[::-1])
            # prtxy = np.vstack((prtrx,prtry))
            # print "rx and ry is", prtxy.T
            new_length = 2*len(rx) - 1
            newrx,newry = np.zeros(new_length),np.zeros(new_length)
            insert_index = np.arange(0, new_length, 2)
            newrx[insert_index] = rx
            newry[insert_index] = ry
            mean_rx = np.mean(np.vstack((rx[:-1], rx[1:])), axis=0)
            mean_ry = np.mean(np.vstack((ry[:-1], ry[1:])), axis=0)
            newrx[insert_index[0:-1] + 1] = mean_rx
            newry[insert_index[0:-1] + 1] = mean_ry
            rx = newrx
            ry = newry
            rx = ((np.array(rx) + minox)* self.plan_grid_size).tolist()
            ry = ((np.array(ry) + minoy)* self.plan_grid_size).tolist()
            rx[-1],ry[-1] = startx, starty
            rx[0],ry[0] = goalx, goaly
            self.show = np.vstack( (np.array(rx),np.array(ry)) )
            print(self.show.T)


            return rx,ry

    def testbatch(self):
        obs = np.array([[1,3],
                        [2,7],
                        [4,5],
                        [7,3],
                        [8,7]])
        print(obs)
        ox = obs[:,0].tolist()
        oy = obs[:,1].tolist()
        sx = [6]
        sy = [3]
        gx = [7]
        gy = [7]
        minox = np.array(ox).min()
        minoy = np.array(oy).min()
        maxox = np.array(ox).max()
        maxoy = np.array(oy).max()
        self.planning(ox,oy,sx,sy,gx,gy,minox,minoy,maxox,maxoy)

    def JPS(self,map,s,g,row,col):
        jps.updatemap(map)
        path,err = jps.find_path(s,g,row,col)
        pathary = np.array(path)
        rx = pathary[:,0].tolist()
        ry = pathary[:,1].tolist()
        rx,ry = rx[::-1],ry[::-1]
        return rx,ry



