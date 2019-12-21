import numpy as np 


class ImgPoints(object): 

	def getPoints(self):  
		img_npy = 'distance_coordinates.npy'
		points = np.load(img_npy, "r")
		return points 


