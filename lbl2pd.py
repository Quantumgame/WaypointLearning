import pdb
import os
import PIL.Image

'''
Create probabilty distribution over pixels in environment image.
Take average of all surrounding pixels as new pixel value.
'''

dataPath = os.path.join('segnet.tf-master', 'src', 'input', 'raw') 

for lblset in ['train-labels', 'test-labels']:  
	path = os.path.join(dataPath, lblset)
	 
	for pic in os.listdir(path): 
		if pic.endswith('.png'):
			imagePath = os.path.join(path, pic)
			
			with open(imagePath, 'r+b') as f:
				with PIL.Image.open(f) as image:
					pd = PIL.Image.new('RGB',(image.size[0], image.size[1]))

					for length in xrange(image.size[1]):
						for width in xrange(image.size[0]):
							neighbors = [image.getpixel((width, length))] # fill initially with current pixel
							#if image.getpixel((width, length))[1] == 255:
								#pdb.set_trace()
							# corners
							if width == 0 and length == 0: # top-left corner
								neighbors.append(image.getpixel((width + 1, length))) # east
								neighbors.append(image.getpixel((width, length + 1))) # south
								neighbors.append(image.getpixel((width + 1, length + 1))) # se

							elif width == 0 and length == (image.size[1] - 1): # bottom-left corner
								neighbors.append(image.getpixel((width + 1, length))) # east
								neighbors.append(image.getpixel((width, length - 1))) # north
								neighbors.append(image.getpixel((width + 1, length - 1))) # ne

							elif width == (image.size[0] - 1) and length == 0: # top-right corner
								neighbors.append(image.getpixel((width - 1, length))) # west
								neighbors.append(image.getpixel((width, length + 1))) # south
								neighbors.append(image.getpixel((width - 1, length + 1))) # sw

							elif width == (image.size[0] - 1) and (image.size[1] - 1): # bottom-left corner
								neighbors.append(image.getpixel((width - 1, length))) # west
								neighbors.append(image.getpixel((width, length - 1))) # north
								neighbors.append(image.getpixel((width - 1, length - 1))) # nw

							# non-corner outer pixels
							elif width == 0: # left edge
								neighbors.append(image.getpixel((width + 1, length))) # east
								neighbors.append(image.getpixel((width, length + 1))) # south
								neighbors.append(image.getpixel((width, length - 1))) # north
								neighbors.append(image.getpixel((width + 1, length + 1))) # se
								neighbors.append(image.getpixel((width + 1, length - 1))) # ne

							elif width == (image.size[0] - 1): # right edges
								neighbors.append(image.getpixel((width - 1, length))) # west
								neighbors.append(image.getpixel((width, length + 1))) # south
								neighbors.append(image.getpixel((width, length - 1))) # north
								neighbors.append(image.getpixel((width - 1, length + 1))) # sw
								neighbors.append(image.getpixel((width - 1, length - 1))) # nw

							elif length == 0: # top edge
								neighbors.append(image.getpixel((width + 1, length))) # east
								neighbors.append(image.getpixel((width - 1, length))) # west
								neighbors.append(image.getpixel((width, length + 1))) # south
								neighbors.append(image.getpixel((width + 1, length + 1))) # se
								neighbors.append(image.getpixel((width - 1, length + 1))) # sw

							elif length == (image.size[1] - 1): # bottom edges
								neighbors.append(image.getpixel((width + 1, length))) # east
								neighbors.append(image.getpixel((width - 1, length))) # west
								neighbors.append(image.getpixel((width, length - 1))) # north
								neighbors.append(image.getpixel((width + 1, length - 1))) # ne
								neighbors.append(image.getpixel((width - 1, length - 1))) # nw

							# inner pixels
							else:
								neighbors.append(image.getpixel((width + 1, length))) # east
								neighbors.append(image.getpixel((width - 1, length))) # west
								neighbors.append(image.getpixel((width, length - 1))) # north
								neighbors.append(image.getpixel((width, length + 1))) # south
								neighbors.append(image.getpixel((width + 1, length - 1))) # ne
								neighbors.append(image.getpixel((width - 1, length - 1))) # nw
								neighbors.append(image.getpixel((width + 1, length + 1))) # se
								neighbors.append(image.getpixel((width - 1, length + 1))) # sw

							average = [0, 0, 0]
							for n in neighbors:
								average[0] += n[0] # R
								average[1] += n[1] # G
								average[2] += n[2] # B
							average = [(pixelsum / len(neighbors)) for pixelsum in average]
							pd.putpixel((width, length), tuple(average))

			pd.save(imagePath, image.format)
