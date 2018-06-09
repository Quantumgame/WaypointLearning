import pdb
import Image
from sys import argv

imgpath = argv[1]
lblpath = argv[2]
mergepath = argv[3]

background = Image.open(imgpath).convert("RGBA")
foreground = Image.open(lblpath).convert("RGBA")

for width in xrange(foreground.size[0]):
	for length in xrange(foreground.size[1]):
		pixel = foreground.getpixel((width, length))
		if pixel[0] == pixel[1] == pixel[2]:
			foreground.putpixel((width, length), (255, 255, 255, 100))
		else:
			foreground.putpixel((width, length), (0, 255, 0, 100))

background.paste(foreground, (0, 0), foreground)
background.save(mergepath, background.format)