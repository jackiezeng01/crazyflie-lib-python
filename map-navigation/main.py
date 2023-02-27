import numpy as np

from generate_map import map_generator
from generate_path import path_generator

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


  resolution = 10   # Resolution of solution in pixels per meter


  #BE WARNED: this funciton also changes the obstacle and waypoints lists
  M = map_generator(map_size, obstacle_list, waypoints, resolution)
  # M.show(overlay=True)
  print(M.map.shape)

  P = path_generator(M.map, resolution)

  P.A_star(waypoints["A"], waypoints["F"])

  print(P.path)