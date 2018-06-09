# adapted from Simple Navigation OpenRAVE example, Rosen Diankov (rosen.diankov@gmail.com)
# http://openrave.org/docs/0.8.2/_modules/openravepy/examples/simplenavigation/#SimpleNavigationPlanning
from __future__ import division
import pdb
import time
import os, sys
import shutil
import PIL.Image
#from mss import mss, tools
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

saliencypath = os.path.join('saliency-map-master', 'src')
sys.path.append(saliencypath)
from saliency_map import SaliencyMap
from utils import OpencvIo

datasetsize = 1
pathsperimage = 1
random.seed(None)

class mNavigationPlanningLabels:
    def __init__(self, robot, envnum, bounds, limits, labelsize=224):
        self.env = robot.GetEnv()
        self.robot = robot
        self.bounds = bounds
        self.limits = limits
        self.labelsize = labelsize
        self.cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.robot)
        self.envnum = envnum

        if not self.cdmodel.load():
            self.cdmodel.autogenerate()

        self.basemanip = interfaces.BaseManipulation(self.robot)

    def idealPath(self, start, goal):
    	# guarantee ideal paths for listed enivironments
		if self.envnum == 0: # make sure goal is on opposite side (based on x-coordinate) of robot's current location
			if start[0] * goal[0] > 0: 
				return False
			else:
				return True
		elif self.envnum == 0.1:
			if (start[0] > 0.75 and goal[0] > 0.75) or (start[0] < 0.75 and goal[0] < 0.75):
				return False
			else:
				return True
		elif self.envnum == 0.2:
			if (start[0] > -0.75 and goal[0] > -0.75) or (start[0] < -0.75 and goal[0] < -0.75):
				return False
			else:
				return True
		elif self.envnum == 1 or self.envnum == 1.1: # make sure goal is not in the same section of the room as the robot
			if start[0] * goal[0] > 0: 
				return False
			else:
				return True
		elif self.envnum == 2: # make sure goal and robot are not in the same room
			if start[0] < 0 and goal[0] < 0: # goal and robot are in outer room, left of inner room
				return False
			elif (start[0] < 0 and goal[0] > 0 and goal[1] > 0.25) or (goal[0] < 0 and start[0] > 0 and start[1] > 0.25): # robot/goal on left of inner room while goal/robot is above inner room
				return False
			elif (start[0] > 0 and start[1] > 0.25) and (goal[0] > 0 and goal[1] > 0.25): # goal and robot are above the inner room
				return False
			elif (start[0] > 0 and start[1] < 0.25) and (goal[0] > 0 and goal[1] < 0.25): # goal and robot are in the inner room
				return False
			else:
				return True
		elif self.envnum == 2.1:
			if start[0] > 0 and goal[0] > 0: # goal and robot are in outer room, right of inner room
				return False
			elif (start[0] > 0 and goal[0] < 0 and goal[1] > 0.25) or (goal[0] > 0 and start[0] < 0 and start[1] > 0.25): # robot/goal on right of inner room while goal/robot is above inner room
				return False
			elif (start[0] < 0 and start[1] > 0.25) and (goal[0] < 0 and goal[1] > 0.25): # goal and robot are above the inner room
				return False
			elif (start[0] < 0 and start[1] < 0.25) and (goal[0] < 0 and goal[1] < 0.25): # goal and robot are in the inner room
				return False
			else:
				return True
		elif self.envnum == 3.0:
			if (start[0] > 0 and start[1] > 0) and (goal[0] > 0 and goal[1] > 0): # both in Q1
				return False
			elif (start[0] < 0 and start[1] > 0) and (goal[0] < 0 and goal[1] > 0): # both in Q2
				return False
			elif (start[0] < 0 and start[1] < 0) and (goal[0] < 0 and goal[1] < 0): # both in Q3
				return False
			elif (start[0] > 0 and start[1] < 0) and (goal[0] > 0 and goal[1] < 0): # both in Q4
				return False
			elif (-1 < goal[0] < 1) and (-1 < goal[1] < 1): # goal in inaccessible location
				return False
			else:
				return True
		elif self.envnum == 4.0:
			if start[0] < -0.5 and goal[0] < -0.5: # goal and robot left of obstacles
				return False
			elif (start[0] > -0.5 and goal[0] > -0.5) and (start[1] < 0 and goal[1] < 0): # goal and robot in Q4
				return False
			elif (start[0] > -0.5 and goal[0] > -0.5) and (start[1] > 0 and goal[1] > 0): # goal and robot in Q1
				return False
			elif (start[0] < -0.5 and goal[0] > -0.5) or (goal[0] < -0.5 and start[0] > -0.5): # robot/goal left of obstacles and goal/robot on obstacle side
				return False
			else:
				return True
		else: # random envs
			if (start[0] > 0 and start[1] > 0) and (goal[0] > 0 and goal[1] > 0): # both in Q1
				return False
			elif (start[0] < 0 and start[1] > 0) and (goal[0] < 0 and goal[1] > 0): # both in Q2
				return False
			elif (start[0] < 0 and start[1] < 0) and (goal[0] < 0 and goal[1] < 0): # both in Q3
				return False
			elif (start[0] > 0 and start[1] < 0) and (goal[0] > 0 and goal[1] < 0): # both in Q4
				return False
			else:
				return True

    def pixelInPath(self, pixel, path, pixelwidth, bounds):
    	'''
		first obtain pixel bounds in terms of world coordinates
    	'''
    	pixminx = bounds[0][0] + (pixel[0] * pixelwidth)
    	pixminy = bounds[1][1] - ((pixel[1] + 1) * pixelwidth)
    	pixmaxx = bounds[0][0] + ((pixel[0] + 1) * pixelwidth)
    	pixmaxy = bounds[1][1] - (pixel[1] * pixelwidth)
    	b = [(pixminx, pixminy), (pixmaxx, pixmaxy)]

    	'''
		see if a path point is within the pixel bounds; return True if so, else False
    	'''
    	for point in path:
    		if (b[0][0] <= point[0] <= b[1][0]) and (b[0][1] <= point[1] <= b[1][1]):
    			return True

    	return False

    def pixelInObstacle(self, pixel, pixelwidth, bounds):
    	'''
		first obtain pixel bounds in terms of world coordinates
    	'''
    	pixminx = bounds[0][0] + (pixel[0] * pixelwidth)
    	pixminy = bounds[1][1] - ((pixel[1] + 1) * pixelwidth)
    	pixmaxx = bounds[0][0] + ((pixel[0] + 1) * pixelwidth)
    	pixmaxy = bounds[1][1] - (pixel[1] * pixelwidth)

    	'''
		see if moving pixel-sized cube to centroid of pixel bound causes collision; return True if so, else False
    	'''
    	aveX = (pixmaxx + pixminx) / 2
    	aveY = (pixmaxy + pixminy) / 2
    	
    	body = openravepy.RaveCreateKinBody(self.env, '')
    	body.SetName('tracer')
    	body.InitFromBoxes(array([[aveX, aveY, 0.2, pixelwidth, pixelwidth, pixelwidth]]), True) # make sure z-coordinate is bounded by shortest wall height
    	self.env.AddKinBody(body)
    	self.env.UpdatePublishedBodies()
    	
    	if self.env.CheckCollision(body): 
    		self.env.Remove(body)
    		return True
    	else:
    		self.env.Remove(body)
    		return False

    def buildLabel(self, lblpath, fullpath, bounds): 
    	'''
		build label images manually through prodding robot environment
    	'''
    	lbl = PIL.Image.new('RGB',(224, 224))

    	ppwu = lbl.size[0] / (bounds[1][0] - bounds[0][0]) # number of pixels per unit in the robot world
    	pixelwidth = 1 / ppwu
    	
    	for length in xrange(lbl.size[1]):
    		for width in xrange(lbl.size[0]):
				
				if self.pixelInPath((width, length), fullpath, pixelwidth, bounds):
					lbl.putpixel((width, length), (0, 255, 0))
					# record pixel bounds in file in the catkin_ws
				else:
					lbl.putpixel((width, length), (0, 0, 0)) # 255, 255, 255
				
    	lbl.save(lblpath, 'PNG')
    	# save total number of green pixels at top of samples.txt file 

    def buildEnv(self, imgpath, bounds):
    	'''
		build environment images manually through prodding robot environment
    	'''
    	oldCC = self.env.GetCollisionChecker()
    	collisionChecker = RaveCreateCollisionChecker(self.env, 'bullet')
    	self.env.SetCollisionChecker(collisionChecker)

    	prev = self.robot.GetActiveDOFValues()
    	with self.env:
    		self.robot.SetActiveDOFValues([5, 5, 0])

    	img = PIL.Image.new('RGB',(224, 224))

    	ppwu = img.size[0] / (bounds[1][0] - bounds[0][0]) # number of pixels per unit in the robot world
    	pixelwidth = 1 / ppwu
    	
    	for length in xrange(img.size[1]):
    		for width in xrange(img.size[0]):
				if self.pixelInObstacle((width, length), pixelwidth, bounds):
					img.putpixel((width, length), (0, 0, 0))
				else:
					img.putpixel((width, length), (255, 255, 255))
    			
    	img.save(imgpath, 'PNG')

    	with self.env:
    		self.robot.SetActiveDOFValues(prev)

    	self.env.SetCollisionChecker(oldCC)

    def buildSM(self, gazepath, lblpath):
    	'''
		build saliency map from gaze labels
    	'''
    	'''
    	# convert black pixels in lbl to white 
    	with open(lblpath, 'r+b') as f:
				with PIL.Image.open(f) as image:
					for width in xrange(image.size[0]):
						for length in xrange(image.size[1]):
							pixel = image.getpixel((width, length))

							if pixel == (0, 0, 0):
								image.putpixel((width, length), (255, 255, 255))
		'''
    	oi = OpencvIo()
    	gaze = oi.imread(lblpath)
    	sm = SaliencyMap(gaze)
    	oi.imwrite(sm.map, gazepath)

    	# threshold SM
    	with open(gazepath, 'r+b') as f:
			with PIL.Image.open(f).convert('RGB') as image:
				for length in xrange(image.size[0]):
					for width in xrange(image.size[1]):
						pixel = image.getpixel((length, width)) 

						# putpixel() is slow, learn to use paste for single pixel placement if possible 
						if pixel[0] < 50: 
							image.putpixel((length, width), (0, 0, 0))
						elif 50 <= pixel[0] < 100:
							image.putpixel((length, width), (100, 100, 100))
						elif 100 <= pixel[0] < 150:
							image.putpixel((length, width), (175, 175, 175))
						else:
							image.putpixel((length, width), (255, 255, 255))

				image.save(gazepath, image.format)

    def performNavigationPlanning(self):
        size = 0
        numpaths = 1
        trace = []
        path = []
        fullpath = []
     	
		# motion and tracing
        while size < datasetsize:       
		    with self.env:
				self.robot.SetAffineTranslationLimits(self.limits[0], self.limits[1])
				self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
				self.robot.SetAffineRotationAxisMaxVels(ones(4))
				self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1]) # joint DOFs, affine DOFs, axis of rotation

				with self.robot:
					while True:
						goal = self.bounds[0,:] + random.rand(3) * (self.bounds[1,:] - self.bounds[0,:]) # picks random goal location
						start = self.robot.GetActiveDOFValues()
						
						if not self.idealPath(start, goal):
							continue

						self.robot.SetActiveDOFValues(goal) # set robot at goal to check for potential collision

						if not self.env.CheckCollision(self.robot): # return robot to start configuration if proposed goal cause collision
							self.robot.SetActiveDOFValues(start) 
							break

						self.robot.SetActiveDOFValues(start) # return robot to start configuration if proposed goal cause collision
		    
		    print '\nplanning from: ' + str(start) + '\nto: ' + str(goal)
		    try:
		    	traj = self.basemanip.MoveActiveJoints(goal=goal, maxiter=10000, steplength=0.1, execute=False, outputtrajobj=True) # path trajectory information
		    except:
		    	print 'retrying...'
		    	continue

		    if traj is None: # allows trajectory if no joint collisions
				print 'retrying...'
				continue

		    prevtime = 0
		    starttime = time.time()

			# save path points
		    while time.time()-starttime < traj.GetDuration():
		    	curtime = time.time() - starttime
		    	trajdata = traj.Sample(curtime) # trajdata[0],[1],[2] contain the x,y,z coordinates of the robot at time == curtime
		    	move = [trajdata[0], trajdata[1], trajdata[2]]
		    	path.append(move)
		    	fullpath.append(move)
		    	prevtime = curtime
		    	time.sleep(0.01)
		    
			# trace and move robot
		    for point in path:
		    	with self.env: # lock environment when accessing robot
		    		self.robot.SetActiveDOFValues(array(point))
		    		trace.append(self.env.plot3(points=self.robot.GetLinks()[0].GetTransform()[0:3,3], pointsize=0.03, colors=array((0,1,0,0.5)), drawstyle=1)) # tranlucent green
		    
		    # save multi-path labels 
		    if numpaths < pathsperimage: 
		    	numpaths += 1
		    else:
			    # create label without screenshot
			    print '\ncreating label'
			    lblpath = os.path.join('data', 'env' + str(self.envnum), 'lbl', str(self.envnum) + str(size) + '.png')
			    self.buildLabel(lblpath, fullpath, self.bounds)
			    print '\nlabel created'
 
			    '''
			    # screenshot approach
			    with self.env:
			    	location = self.robot.GetActiveDOFValues() # save robot location
			    	self.robot.SetActiveDOFValues(array((5, 5, 0))) # move robot out of view before screenshot 

			    time.sleep(0.5) # allows environment viewer time to update
			    
			    with mss() as screen:
			    	imagepath = os.path.join('data', 'env' + str(self.envnum), 'lbl', str(self.envnum) + str(size) + 'sc.png')
			    	monitor = {'top': 70, 'left': 1175, 'width': 224, 'height': 224} # might have to edit depending on docker loation
			    	srcn = screen.grab(monitor) 
			    	tools.to_png(data=srcn.rgb, size=srcn.size, output=imagepath) 
			    
			    with self.env:
			    	self.robot.SetActiveDOFValues(location) # return robot to previous location
			    '''
			    # reset for next run
			    fullpath = []
			    trace = []
			    size += 1
			    numpaths = 1

		    # wait 
		    print '\nwaiting for controller'
		    self.robot.WaitForController(0)

			# reset for next path		 
		    path = []

		# create env scans
        print 'creating environment scans'
        lbldir = os.path.join('data', 'env' + str(self.envnum), 'lbl')
        env = os.path.join('env.png')
        self.buildEnv(env, self.bounds)

        for image in xrange(len(os.listdir(lbldir))):
        	imgpath = os.path.join('data', 'env' + str(self.envnum), 'img', str(self.envnum) + str(image) + '.png')
        	shutil.copy(env, imgpath)
        	gazepath = os.path.join('data', 'env' + str(self.envnum), 'gaze', str(self.envnum) + str(image) + '.png')
        	lblpath = os.path.join('data', 'env' + str(self.envnum), 'lbl', str(self.envnum) + str(image) + '.png')
        	#self.buildSM(gazepath, lblpath)
		
        os.remove(env)
        print 'scans created'

        

