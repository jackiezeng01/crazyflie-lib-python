import numpy as np

from generate_map import map_generator
from generate_path import path_generator

import matplotlib.pyplot as plt

from spline_path import spline_path

if __name__ == "__main__":
  # Obstacle polygon corners
  obstacle_list = [np.array([[0, .3], [0, .605], [1.22, .605], [1.22, .3]]),
                   np.array([[0,-.3], [0,-.605], [1.22,-.605], [1.22,-.3]])]

  waypoints = {"A": np.array([-.6,-.6]),
               "B": np.array([-.6,-.3]),
               "C": np.array([-.6,  0]),
               "D": np.array([-.6, .3]),
               "E": np.array([-.6, .6]),
               "F": np.array([1.3,-.6])}
  
  map_size = np.array([2.1,3.3])


  resolution = 8   # Resolution of solution in pixels per meter


  #BE WARNED: this funciton also changes the obstacle and waypoints lists
  M = map_generator(map_size, obstacle_list, waypoints, resolution)
  # M.show(overlay=True)
  # print(M.map.shape)  

  P = path_generator(M.map, resolution)

  P.A_star(waypoints["E"], waypoints["F"])

  print(P.path.shape)

  P.path = np.flipud(P.path)

  new_waypoints = spline_path(P.path[:,0], P.path[:,1], 60)

  print(new_waypoints.shape)



  if True:
    plt.clf()
    # im = plt.imread(P.map)
    implot = plt.imshow(P.map_basic, cmap="gray")

    # fig, ax = plt.subplots()
    plt.plot(new_waypoints[0]*resolution, new_waypoints[1]*resolution, 'bo')
    plt.plot(P.path[:,0]*resolution, P.path[:,1]*resolution, 'ro')
    plt.axis('equal')
    plt.show()

