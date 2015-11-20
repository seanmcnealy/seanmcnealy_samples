def rotateCW(m):
	m = m * matrix([
	[0, -1, 0, 0, 0], 
	[1,  0, 0, 0, 0],
	[0,  0, 1, 0, 0],
	[0,  0, 0, 1, 0],
	[0,  0, 0, 0, 1]])
	m.value[0][3] = m.value[0][3] - pi/2
	return m

def rotateCCW(m):
	m = m * matrix([
	[ 0, 1, 0, 0, 0], 
	[-1, 0, 0, 0, 0],
	[ 0, 0, 1, 0, 0],
	[ 0, 0, 0, 1, 0],
	[ 0, 0, 0, 0, 1]])
	m.value[0][3] = m.value[0][3] + pi/2
	return m

class Edges:
	delta = 12
	recordedCollisions = {}
	buckets = 51

	#initialize the edges object given coordinate data
	def __init__(self, fullCoordData):
		self.left = min(map(lambda x: x[0], fullCoordData))
		self.right = max(map(lambda x: x[0], fullCoordData))
		self.top = max(map(lambda y: y[1], fullCoordData))
		self.bottom = min(map(lambda y: y[1], fullCoordData))
		for i in range(self.buckets):
			self.recordedCollisions[i] = []

	#determine if a coordinate is near an edge, and if so, which one
	def isNearEdge(self, coord):
		self.left = min(self.left, coord[0])
		self.right = max(self.right, coord[0])
		self.top = max(self.top, coord[1])
		self.bottom = min(self.bottom, coord[1])

		if (coord[0] < self.left + self.delta): return 1
		elif (coord[0] > self.right - self.delta): return 2
		elif (coord[1] > self.top - self.delta): return 3
		elif (coord[1] < self.bottom + self.delta): return 4
		else: return 0

	#determine the angle of the object in relation to the wall it is bouncing off of
	def angleToIndex(self, o, edge):
		a = o % (2*pi)
		if(a < 0): a = a + 2.*pi

		index = 0
		if(edge == 1):
			index = a - (pi/2.)
		elif(edge == 2):
			index = a + (pi/2.)
		elif(edge == 3):
			index = a
		elif(edge == 4):
			index = a - pi
		
		index = index % (2*pi)
		if(index < 0): index = index + 2*pi

		index = index / pi * self.buckets
		return int(round(index))

	#save a detected collision for later use when predicting points
	def saveCollision(self, instate, output, time, edge):
		index = self.angleToIndex(instate.value[3][0], edge)
		if(index < 0 or index >= self.buckets):
			return
		sameCollisions = self.recordedCollisions[index]

		deltaOutput = matrix([[
			output.value[0][0] - instate.value[0][0], 
			output.value[1][0] - instate.value[1][0], 
			output.value[2][0], 
			output.value[3][0],
			output.value[4][0]]])

		rotatedOutput = None
		if(edge == 1):
			rotatedOutput = rotateCW(deltaOutput)
		elif(edge == 2):
			rotatedOutput = rotateCCW(deltaOutput)
		elif(edge == 3):
			rotatedOutput = deltaOutput
		elif(edge == 4):
			rotatedOutput = rotateCCW(rotateCCW(deltaOutput))

		orientation = rotatedOutput.value[0][3] % (2*pi)
		if(orientation < pi): orientation = orientation + 2*pi
		if(orientation > pi): orientation = orientation - 2*pi
		rotatedOutput.value[0][3] = orientation

		if(rotatedOutput.value[0][1] > 2. or #don't go inside wall
			rotatedOutput.value[0][2] < 0. or #velocity positive
			rotatedOutput.value[0][3] > 0. or #orientation not toward wall
			abs(rotatedOutput.value[0][4]) > 0.1 ): #curvature higher than we believe
			1==1
		else:
			sameCollisions.append([rotatedOutput, time])

	def findNearbyBucket(self, index):
		contents = self.recordedCollisions[index]
		if(len(contents) > 0): return contents

		#spread out search, average if we're between data
		for i in range(1,self.buckets):
			leftBucket = []
			rightBucket = []
			if index + i < self.buckets:
				rightBucket = self.recordedCollisions[index+i]
			if index - i >= 0:
				leftBucket = self.recordedCollisions[index-i]
			if len(leftBucket) + len(rightBucket) > 0:
				return leftBucket + rightBucket
		return []

	#estimate a collision based on the past data we have seen.
	def estimateCollision(self, instate, edge):
		index = self.angleToIndex(instate.value[3][0], edge)
		if(index < 0 or index >= self.buckets): index = 0
		sameCollisions = self.findNearbyBucket(index)
		sameCollisionsLen = len(sameCollisions)
		if(sameCollisionsLen == 0): return [None, 0]

		estimatedSum = reduce( 
			lambda acc, x: map(sum, zip(acc,x)), 
			map(lambda x: x[0].value[0], sameCollisions)
		)
		estimated = matrix([map(lambda x: x / sameCollisionsLen, estimatedSum)])
		estimatedTime = sum(map(lambda x: x[1], sameCollisions)) / sameCollisionsLen
		rotatedEstimated = None
		if(edge == 1):
			rotatedEstimated = rotateCCW(estimated)
		elif(edge == 2):
			rotatedEstimated = rotateCW(estimated)
		elif(edge == 3):
			rotatedEstimated = estimated
		elif(edge == 4):
			rotatedEstimated = rotateCCW(rotateCCW(estimated))
		return [rotatedEstimated, int(round(estimatedTime))]
