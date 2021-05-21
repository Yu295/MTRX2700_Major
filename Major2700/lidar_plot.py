import matplotlib.pyplot as plt
import csv

elev = []
azi = []
dist = []

with open('lidar_test.csv') as f:
    obj = csv.reader(f)

    for row in obj:
        elev.append(int(row[0]))
        azi.append(int(row[1]))
        dist.append(int(row[2]))

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(elev, azi, dist)
ax.set_xlabel('Elevation (deg)')
ax.set_ylabel('Azimuth (deg)')
ax.set_zlabel('Distance (mm)')
plt.show()
