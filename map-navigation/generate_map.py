import cv2
import numpy as np
import matplotlib.pyplot as plt

class map_generator():
  def __init__(self, size, resolution):
    self.size = size.astype(int) * resolution
    self.resolution = resolution
    
    # Obstacle polygon corners
    obstacle_1 = np.array([[0, .3], [0, .605], [1.22, .605], [1.22, .3]])
    obstacle_2 = np.array([[0,-.3], [0,-.605], [1.22,-.605], [1.22,-.3]])

    known_obstacle_list = [obstacle_1, obstacle_2]

    waypoint_dict = {
      "A": np.array([-.6,-.6]),
      "B": np.array([-.6,-.3]),
      "C": np.array([-.6,  0]),
      "D": np.array([-.6, .3]),
      "E": np.array([-.6, .6])
    }

    self.map = self.create_map(known_obstacle_list)

    self.overlay = self.create_overlay(waypoint_dict)
    

  def create_map(self, known_obstacles):
    map = np.zeros(self.size)+255

    for obj in known_obstacles:
      # Flip the transform to the center of the image as pictures are (col, row) instead of (x, y)
      obj = self.scale2plot(obj)

      # Draw a filled polygon to represent the obstacle
      cv2.fillPoly(
        img = map,
        pts=[obj],
        color=(0, 0, 0)
        )

    return map

  def create_overlay(self, point_dict):
    overlay = np.zeros(np.append(self.size,3))+255

    for label in point_dict:
      point_dict[label] = self.scale2plot(point_dict[label])
      overlay = cv2.circle(overlay, point_dict[label], 1, (255, 0, 0), 1)
      cv2.putText(overlay, label, point_dict[label] - np.array([-5,0]), cv2.FONT_HERSHEY_SIMPLEX, .2, (0,0,255), 1)

    return overlay

    
  def scale2plot(self, array):
    # All map objects must be scaled by the resolution of the map image and moved about the center
    array *= self.resolution
    array += self.size[::-1]/2
    return array.astype(int)

  def show(self, overlay = False):
    fig = plt.figure()
    viewer = fig.add_subplot(111)

    img = np.repeat(self.map[:, :, np.newaxis], 3, axis=2)

    if overlay:
      img *= self.overlay
    
    viewer.imshow(img, interpolation='nearest')

    viewer.tick_params(labelbottom=False)
    viewer.tick_params(labelleft=False)

    fig.canvas.draw()
    plt.pause(10)


# if __name__ == "__main__":
#   M = map_generator(np.array([2.1,3.3]), 1000)