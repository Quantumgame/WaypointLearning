import time, pdb
from numpy import *
from testplanning import SimpleNavigationPlanning
from openravepy import *

# load enironment
env = Environment()
env.SetViewer('qtcoin')
viewer = env.GetViewer()
env.Load('/home/dmolina/Documents/ASU/590/envs/test.env.xml')
viewer.SetCamera([[  9.99903071e-01,  -7.69185870e-03,  -1.16053160e-02,
          5.02935469e-01],
       [ -3.12990072e-03,  -9.36386676e-01,   3.50956117e-01,
         -3.20194983e+00],
       [ -1.35665682e-02,  -3.50885776e-01,  -9.36319988e-01,
          7.83092308e+00],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

# change collision checker from ODE to Bullet since default build of ODE doesn't recognize cylinder collision for Ubuntu
collisionChecker = RaveCreateCollisionChecker(env, 'bullet')
env.SetCollisionChecker(collisionChecker)

# load robots
robot = env.GetRobots()[0]
env.UpdatePublishedBodies()
time.sleep(0.1)

# perform motion planning
planner = SimpleNavigationPlanning(robot)
planner.performNavigationPlanning()

#pdb.set_trace()


