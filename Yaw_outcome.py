import numpy as np
print("Looking for the max sigment in a LiDAR")

LiDAR1= [float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 0.1, float('inf')]
LiDAR2= [float('inf'), .1, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf')] #Lidar 2 flipped
LiDAR2LiDAR_distance = .10
arr1 = np.array(LiDAR1)
maxSig1 = np.amin(arr1)
maxSigNumber1=list(arr1).index(maxSig1)

print("Max sigment value of LiDAR1 is:", maxSig1, "that is number:", maxSigNumber1)

arr2 = np.array(LiDAR2)
maxSig2 = np.amin(arr2)
maxSigNumber2=list(arr2).index(maxSig2)

print("Max sigment value of LiDAR2 is:", maxSig2, "that is number:", maxSigNumber2)

nLs= ((8-maxSigNumber2)*0.2178*maxSig2) - ((maxSigNumber1+5+1)*0.104*maxSig1)
print("nLs value", nLs)

Yaw=np.rad2deg(np.arctan(nLs/LiDAR2LiDAR_distance))

print("Yaw value", Yaw)
