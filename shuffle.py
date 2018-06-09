import pdb
import os
import shutil
import numpy.random

'''
shuffle images and move to SegNet data directory
'''
numpy.random.seed(None)
dataPath = 'data'
split = 0.9
segnetTrain = os.path.join('segnet.tf-master', 'src', 'input', 'raw', 'train')
segnetTrainLabels = os.path.join('segnet.tf-master', 'src', 'input', 'raw', 'train-labels')
segnetTest = os.path.join('segnet.tf-master', 'src', 'input', 'raw', 'test')
segnetTestLabels = os.path.join('segnet.tf-master', 'src', 'input', 'raw', 'test-labels')
imgs = os.path.join('placeholder', 'img') # delete placeholder directory after images moved to SegNet
lbls = os.path.join('placeholder', 'lbl')

os.makedirs(imgs)
os.mkdir(lbls)

'''
move all data into an intermediary folder to be shuffled for sampling
'''
for env in os.listdir(dataPath): 
	envPath = os.path.join(dataPath, env)

	for category in ['img', 'lbl']:
		path = os.path.join(envPath, category)

		for pic in os.listdir(path): 
			if pic.endswith('.png'):
				imagePath = os.path.join(path, pic)

				if category == 'img':
					shutil.copy(imagePath, imgs)
				else:
					shutil.copy(imagePath, lbls)

'''
randomly shuffle images in the placeholder folder and move them into their corresponding folders in the SegNet data directory using a 80-20 training/test split
'''
trainsize = 0
images = os.listdir(imgs)

numpy.random.shuffle(images)

for pic in images:
	if pic.endswith('.png'):
		imagePath = os.path.join(imgs, pic)
		labelPath = os.path.join(lbls, pic)

		if trainsize < split * len(images): # add image to training set
			shutil.copy(imagePath, segnetTrain) # image
			shutil.copy(labelPath, segnetTrainLabels) # label

			trainsize += 1
		else: # add image to test set
			shutil.copy(imagePath, segnetTest) # image
			shutil.copy(labelPath, segnetTestLabels) # label

'''
delete intermediary folder
'''			
shutil.rmtree('placeholder')


####################################










