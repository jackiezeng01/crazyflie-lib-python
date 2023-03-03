import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt


def spline_path(x, y, res):
    """ Takes x, y path points in meters and fits a smooth curve to the path. 
        Also calculates the yaw and makes all z values 1 m.
    """
    tck, _ = splprep([x, y], s=.02, per=False)
    u = np.linspace(0,1,num=res)
    new_points = splev(u, tck)

    if True:
        fig, ax = plt.subplots()
        ax.plot(x, y, 'ro')
        ax.plot(new_points[0], new_points[1], 'ro')
        # plt.show()

    # Add z values
    new_points.append([1] * len(new_points[0]))

    # Find the angle between the points
    new_points.append([]) # add new list for yaw
    yaw = []
    for i in range(0,len(new_points[0])-1):
        x1, y1 = new_points[0][i], new_points[1][i]
        x2, y2 = new_points[0][i+1], new_points[1][i+1]
        
        new_points[3].append(np.arctan2((y2-y1),(x2-x1)))

    new_points[3].append(None) # Make the last yaw 

    return np.array(new_points)

if __name__ == "__main__":

    x = [0,1,1,0,0]
    y = [0,0,1,1,0]

    new_points = spline_path(x,y)

    print(new_points)
