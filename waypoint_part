import numpy as np
import matplotlib.pyplot as plt

x_wp_1 = []
y_wp_1 = []
x_wp_2 = []
y_wp_2 = []
x_wp_3 = []
y_wp_3 = []
x_wp_4 = []
y_wp_4 = []

xpos = [2.204,1.312,0.755,0.755,0.755]
ypos = [-0.251,-0.251,-0.251,-0.909,-1.783]

# calculate seg 1
dist_seg_1 = xpos[0]-xpos[1]
increment_seg_1 = abs(dist_seg_1)/15
x_wp_1.append(xpos[0])
y_wp_1.append(ypos[0])
for i in range(0,15): #segmen 1
    x_wp_1.append(x_wp_1[-1]-increment_seg_1)
    y_wp_1.append(ypos[0])

# calculate seg 2
dist_seg_2 = xpos[1]-xpos[2]
increment_seg_2 = abs(dist_seg_2)/10
x_wp_2.append(xpos[1])
y_wp_2.append(ypos[1])
for i in range(0,10): #segmen 2
    x_wp_2.append(x_wp_2[-1]-increment_seg_2)
    y_wp_2.append(ypos[1])
  
# calculate seg 3
dist_seg_3 = ypos[2]-ypos[3]
increment_seg_3 = abs(dist_seg_3)/10
x_wp_3.append(xpos[2])
y_wp_3.append(ypos[2])
for i in range(0,10): #segmen 3
    y_wp_3.append(y_wp_3[-1]-increment_seg_3)
    x_wp_3.append(xpos[2])
  
# calculate seg 4
dist_seg_4 = ypos[3]-ypos[4]
increment_seg_4 = abs(dist_seg_4)/15
x_wp_4.append(xpos[3])
y_wp_4.append(ypos[3])
for i in range(0,15): #segmen 4
    y_wp_4.append(y_wp_4[-1]-increment_seg_4)
    x_wp_4.append(xpos[3])

plt.plot(x_wp_1,y_wp_1, label="satu")
plt.plot(x_wp_2,y_wp_2, label="dua")
plt.plot(x_wp_3,y_wp_3, label="tiga")
plt.plot(x_wp_4,y_wp_4, label="empat")
plt.legend()

# waypoint gabungan 
y_wp = y_wp_1 + y_wp_2 + y_wp_3 + y_wp_4
x_wp = x_wp_1 + x_wp_2 + x_wp_3 + x_wp_4

# x_wp
plt.plot(x_wp_1[0],y_wp_1[0],'o', label="start")
plt.plot(x_wp_2[0],y_wp_2[0],'o', label="turn")
plt.plot(x_wp_3[0],y_wp_3[0],'o', label="edge")
plt.plot(x_wp_4[0],y_wp_4[0],'o', label="turn")
plt.plot(x_wp_4[-1],y_wp_4[-1],'o', label="finish")
plt.plot(x_wp, y_wp)
plt.legend()

print(x_wp)
print(y_wp)
