import numpy as np

# subscribed to detect_object which outputs x_axis position and radius of contour
# subscribed lidar which outputs distance at every angle, and other stuff
x_axis = 250
x1 = x_axis
r = 40
vector_size = 360
lidar_data  = 328*np.random.rand(vector_size)
#two_dimensional_array  = np.random.rand(vector_size, 2)

# Set direction. Positive is angled right, negative is angled left
direction = 1
if x1 > 328/2:
    direction = -1

# Perpendicular distance in pixels to center and outer edges of object
x1_bar = 328/2 - x_axis # units of pixel values. Describes perpendicular distance from x_axis of robot
x1_bar = x1_bar*direction

x2_bar = x1_bar - r # result is perp distance from center axis to edge of object. In pixels
x3_bar = x1_bar + r # result is perp distance from center axis to other edge of object. In pixels
# make sure x3_bar is further from horizontal

# angular offset from x axis to center and outer edges of object
theta1 = x1_bar * (62.2/2) / (328/2) # result is angular displacement from x_axis of robot in degrees
theta2 = x2_bar * (62.2/2) / (328/2) # result is angular displacement from x_axis of robot in degrees
theta3 = x3_bar * (62.2/2) / (328/2) # result is angular displacement from x_axis of robot in degrees

# this part will access lidar range data for specific angles
angular_resolution = 1.62 # degrees
angle_index1 = int(theta1 / angular_resolution) # index that refers to angle of center of object
angle_index2 = int(theta2 / angular_resolution) # index that refers to angle of edge of object
angle_index3 = int(theta3 / angular_resolution) # index that refers to angle of other edge of object

# Check all distances in that range and pick the closest (smallest one)
length = abs(angle_index3)-abs(angle_index2)+1
range_at_angle = [None] * length
for i in range(0, length, 1):
    range_at_angle[i] = lidar_data[angle_index2+i]

# Find error to output
error = min(range_at_angle)
index = range_at_angle.index(error)
ang_err = direction*(angle_index2+index) * angular_resolution # angular error in degrees

print(angle_index1)
print(angle_index2)
print(angle_index3)
print(range_at_angle)
print(index)
print(ang_err)
