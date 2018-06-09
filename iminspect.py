import pdb
import PIL.Image
from sys import argv

if len(argv) == 2:
	imPath = argv[1]

	with open(imPath, 'r+b') as f:
		image = PIL.Image.open(f)
		colors = image.getcolors(maxcolors=10000)

		with open(imPath + 'Colors.txt', 'w') as clist:
			for color in colors:
				clist.write(str(color) + '\n')
else:
	print "/nincorrect number of arguments"
	exit(0)