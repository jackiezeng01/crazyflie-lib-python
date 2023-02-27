import cv2
import numpy as np
import matplotlib.pyplot as plt

class map_generator():
  def __init__(self, size, obstacle_list, waypoints, resolution):
    self.size = (size * resolution).astype(int)
    self.resolution = resolution

    self.map = self.create_map(obstacle_list)

    self.overlay = self.create_overlay(waypoints)
    

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

    res_scalar = int(self.resolution/20)

    for label in point_dict:
      point_dict[label] = self.scale2plot(point_dict[label])
      overlay = cv2.circle(overlay, point_dict[label], 0, (255, 0, 0), 1*res_scalar)
      cv2.putText(overlay, label, point_dict[label] - np.array([-5,0]), cv2.FONT_HERSHEY_SIMPLEX, .1*res_scalar, (0,0,255), int(res_scalar/2))

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

    # viewer.tick_params(labelbottom=False)
    # viewer.tick_params(labelleft=False)

    fig.canvas.draw()
    plt.pause(2)


# if __name__ == "__main__":
#   M = map_generator(np.array([2.1,3.3]), 1000)