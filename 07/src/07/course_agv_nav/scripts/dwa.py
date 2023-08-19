import numpy as np

"""
dwa
    Config()
      return an config object c
      c should have c.max_speed and c.predict_time
    planning()
        input:
            self.plan_x,self.plan_config,self.plan_goal,self.plan_ob
            plan_x: [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)] list
            plan_goal: an local goal for dwa; np.array
            plan.ob: the obstacle gotten by laser in the robot frame, np.arrray
        output:
            u: [v w] as an input for controller
            another output
"""

class config:
    def __init__(self):
        self.max_speed = 1.0
        self.max_av = 0.03
        self.max_w = 3.1415/6
        self.max_aw = 3.1415/18 * 5
        self.predict_time = 2.5
        self.grid_size = 0.3
        pass 
    
def Config():
    return config()
    pass



class DWA:
    def __init__(self,X):
        self.alpha = 10.0
        self.beta = 50.0
        self.gamma = 1
        self.x = X[0]
        self.y = X[1]
        self.theta = X[2]
        self.v = X[3]
        self.w = X[4]
        self.dt = 0.1
        pass

    def predict(self,X,dt):
        # r = self.v / self.w
        # xn = self.x - r*np.sin(self.theta) + r*np.sin(self.theta + self.w*dt)
        # yn = self.y + r*np.cos(self.theta) - r*np.cos(self.theta + self.w*dt)
        # thetan = self.theta + self.w*dt
        # Xn = np.array([xn,yn,thetan,self.v,self.w])
        # return Xn
        x,y,theta,v,w = X[0],X[1],X[2],X[3],X[4]
        if w == 0:
            w = 1e-8
        r = v/w
        xn = x - r*np.sin(theta) + r*np.sin(theta + w*dt)
        yn = y + r*np.cos(theta) - r*np.cos(theta + w*dt)
        thetan = theta + w*dt
        Xn = np.array([xn,yn,thetan,v,w]) 
        return Xn
        pass

    def get_window(self,config):
        v_max = config.max_speed
        av_max = config.max_av
        w_max = config.max_w
        aw_max = config.max_aw
        vl = self.v - av_max*self.dt
        vh = self.v + av_max*self.dt
        wl = self.w - aw_max*self.dt
        wh = self.w + aw_max*self.dt
        window = np.array([max(vl,-v_max),min(vh,v_max),
                           max(wl,-w_max),min(wh,w_max)])
        # window = np.array([0,min(vh,v_max),
        #                    max(wl,-w_max),min(wh,w_max)])
        return window
        pass

    def get_feasible_window(self,win,ob,config):
        w = self.w
        if w == 0:
            w = 1e-8
        r = self.v/w
        # center of the circle
        cx = self.x - r*np.sin(self.theta)
        cy = self.y + r*np.cos(self.theta)
        obx = ob[0,:]
        oby = ob[1,:]
        # distance to the circle
        cdist = abs(np.hypot(cx-obx,cy-oby) - r)
        # obstacles on the circle
        index = np.nonzero(cdist < config.grid_size)
        obc = ob[:,cdist<config.grid_size]
        obcx = obc[0,:]
        obcy = obc[1,:]
        # print("obc",obc)
        # print("index",index)
        # print("dist to circle arc", (np.hypot(cx-obcx, cy-obcy)).T)
        # print("len(obc)",len(obc[0,:]))
        if not len(obc[0,:])==0:
            # angles to the circle center of obstacles on circle
            obangle = np.arctan2(obc[1,:]-cy, obc[0,:]-cx)
            roboangle = np.arctan2(self.y-cy, self.x-cx)
            if  w > 0:
                obangle[obangle<roboangle] += 2*np.pi
                deltaangle = min(obangle-roboangle)
            else:
                obangle[obangle>roboangle] -= 2*np.pi
                deltaangle = min(roboangle-obangle)
            if deltaangle<1e-8:
                deltaangle = 1e-8
            ShortestArc = deltaangle * r
            vdist = np.sqrt(2*ShortestArc*config.max_av)
            wdist = np.sqrt(2*ShortestArc*config.max_aw)
            feasible_window = np.array([max(-vdist,win[0]),
                                        min(vdist,win[1]),
                                        max(-wdist,win[2]),
                                        min(wdist,win[3])])
        else:
            # print("no obs ahead")
            feasible_window = win
            ShortestArc = np.inf
        # print("Arc = ",ShortestArc)
        return feasible_window,ShortestArc

    def calc_u(self,goal,ob,config):
        win = self.get_window(config)
        fwin,_ = self.get_feasible_window(win,ob,config)
        candidate_v = np.linspace(fwin[0],fwin[1],num=5)
        candidate_w = np.linspace(fwin[2],fwin[3],num=5)
        minloss = np.inf
        bestu = np.array([self.v,self.w])
        obx = ob[0,:]
        oby = ob[1,:]
        # print("win = ",win)
        # print("fwin = ",fwin)
        for cdv in candidate_v:
            for cdw in candidate_w:
                cdX = np.array([self.x,self.y,self.w,cdv,cdw])
                predictX = self.predict(cdX,config.predict_time)
                goaldist = np.hypot(predictX[0]-goal[0],predictX[1]-goal[1])
                # obdist = np.hypot(predictX[0]-obx,predictX[1]-oby)
                # minobdist = min(obdist)
                _,minobdist = self.get_feasible_window(fwin,ob,config)
                # print("minobdist",minobdist)
                v_err = config.max_speed - abs(predictX[3])
                loss = self.alpha*goaldist + self.beta*(1/minobdist) + self.gamma*v_err
                if loss < minloss:
                    minloss = loss
                    bestu = np.array([cdv,cdw])
        return bestu



def planning(plan_x,config,goal,ob):
    u = [0,0]
    out = 0
    X = np.array(plan_x)
    ob = np.floor_divide(ob, config.grid_size)
    d = DWA(X)
    u = d.calc_u(goal,ob,config)
    # print("u = ",u)
    return u, out
    pass