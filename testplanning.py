#!/usr/bin/env python
# -*- coding: utf-8 -*-

# adapted from Simple Navigation OpenRAVE example, Rosen Diankov (rosen.diankov@gmail.com)
# http://openrave.org/docs/0.8.2/_modules/openravepy/examples/simplenavigation/#SimpleNavigationPlanning

import pdb
import time, numpy, scipy
from mss import mss
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

datasetsize = 100

class SimpleNavigationPlanning:
    def __init__(self,robot,randomize=False,dests=None,switchpatterns=None):
        self.env = robot.GetEnv()
        self.robot = robot
        self.cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.robot)

        if not self.cdmodel.load():
            self.cdmodel.autogenerate()

        self.basemanip = interfaces.BaseManipulation(self.robot)

    def performNavigationPlanning(self):
        trace = []
        path = []
        size = 0

        # obtain boundaries of the environment
        with self.env:
            envmin = []
            envmax = []

            for b in self.env.GetBodies():
                ab = b.ComputeAABB()
                envmin.append(ab.pos()-ab.extents())
                envmax.append(ab.pos()+ab.extents())

            abrobot = self.robot.ComputeAABB()
            envmin = numpy.min(array(envmin),0)+abrobot.extents()
            envmax = numpy.max(array(envmax),0)-abrobot.extents()

        bounds = array(((envmin[0],envmin[1],0),(envmax[0],envmax[1],0)))

		# motion and tracing
        while size < datasetsize:       
		    with self.env:
				self.robot.SetAffineTranslationLimits(envmin,envmax)
				self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
				self.robot.SetAffineRotationAxisMaxVels(ones(4))
				self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1]) # joints, DOFs, ?

				with self.robot:
					while True:
						goal = bounds[0,:]+random.rand(3)*(bounds[1,:]-bounds[0,:]) # picks random goal location
						start = self.robot.GetActiveDOFValues()

						if start[0] * goal[0] > 0: # make sure goal is on opposite side (based on x-coordinate) of which robot currently is
							continue

						self.robot.SetActiveDOFValues(goal) # set robot at goal to check for potential collision

						if not self.env.CheckCollision(self.robot):
							self.robot.SetActiveDOFValues(start) # return robot to start configuration if proposed goal cause collision
							break

						self.robot.SetActiveDOFValues(start) # return robot to start configuration if proposed goal cause collision

		    print 'planning to: ' + str(goal) + '\nfrom: ' + str(start) + '\n'

		    traj = self.basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1, execute=False, outputtrajobj=True) # path trajectory information

		    if traj is None: # allows trajectory if no joint collisions
				print 'retrying...'
				continue

		    prevtime = 0
		    starttime = time.time()

			# save path points
		    while time.time()-starttime < traj.GetDuration():
		    	curtime = time.time()-starttime
		    	trajdata = traj.Sample(curtime) # trajdata[0],[1],[2] contain the x,y,z coordinates of the robot at time = curtime
		    	move = array((trajdata[0], trajdata[1], trajdata[2]))
		    	path.append(move)
		    	prevtime = curtime

		    	time.sleep(0.01)
		    
			# trace and move robot
		    for point in path:
		    	with self.env: # lock environment when accessing robot
		    		self.robot.SetActiveDOFValues(point)
		    		trace.append(self.env.plot3(points=self.robot.GetLinks()[0].GetTransform()[0:3,3], pointsize=0.03, colors=array((0,1,0)), drawstyle=1)) # plots robot's new position

		    # record data point // consider taking shots for each timestep, not just final trajectory
		    with mss() as screen:
		    	imagepath = 'data/dp' + str(size) + '.png'
		    	screen.shot(output = imagepath) # fullscreen shot
		    
		    print 'waiting for controller'
		    self.robot.WaitForController(0)

		    #pdb.set_trace()
		    trace = []
		    path = []
		    size += 1

