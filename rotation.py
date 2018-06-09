import pdb
import os
import shutil
import PIL.Image

'''
create more data through rotating existing input and their labels
'''

dataPath = 'data'

for env in os.listdir(dataPath): 
	envPath = os.path.join(dataPath, env)

	for category in ['img', 'lbl']:
		path = os.path.join(envPath, category)

		for pic in os.listdir(path): 
			if pic.endswith('.png'):
				imagePath = os.path.join(path, pic)

				with open(imagePath, 'r+b') as f:
					with PIL.Image.open(f) as image:
						imgname = pic.replace('.png', '')
						r1Path = os.path.join(path, imgname + 'r1.png')
						r2Path = os.path.join(path, imgname + 'r2.png')
						r3Path = os.path.join(path, imgname + 'r3.png')

						r1 = image.rotate(90)
						r2 = image.rotate(180)
						r3 = image.rotate(270)

						r1.save(r1Path, image.format)
						r2.save(r2Path, image.format)
						r3.save(r3Path, image.format)