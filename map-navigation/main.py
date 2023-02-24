import numpy as np

from generate_map import map_generator
from generate_path import path_generator

if __name__ == "__main__":
  M = map_generator(np.array([2.1,3.3]), 20)
  # M.show(overlay=True)

  P = path_generator(M.map)

  P.A_star(np.array([5,5]), np.array([50,20]))
  # P.show(overlay=True)