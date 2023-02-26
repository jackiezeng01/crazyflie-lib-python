import cv2
import numpy as np
import matplotlib.pyplot as plt

import time

class Node:
  def __init__(self, pos, f=0, g=0, h=0, parent=None):
    self.pos = pos
    self.f = f
    self.g = g
    self.h = h
    self.parent = parent



class path_generator:
  def __init__(self, map):
    # 2D map for A* operations
    self.map = map
    
    # 'Color' B&W map to combine with self.overlay
    self.map_img = np.repeat(self.map[:, :, np.newaxis], 3, axis=2)
    
    # Overlay used for visualizations
    self.overlay = None

    self.last_drawn = None

    self.fig = plt.figure()
    self.viewer = self.fig.add_subplot(111)

    self.viewer.tick_params(labelbottom=False)
    self.viewer.tick_params(labelleft=False)


  def A_star(self, start, end):
    self.overlay = self.create_overlay(start, end)
    self.last_drawn = start

    open = [Node(start)]
    closed = []

    while open:
      tic = time.perf_counter()
      
      q = open.pop(open.index(min(open, key=lambda x:x.f)))
      
      toc1 = time.perf_counter()

      if np.array_equal(q.pos, end):
        print("Goal Achieved")
        break
      
      # set each neighbor as distance 1 from q
      descendants = [ Node(q.pos + [-1, 1], parent=q),
                      Node(q.pos + [ 0, 1], parent=q),
                      Node(q.pos + [ 1, 1], parent=q),
                      Node(q.pos + [-1, 0], parent=q),
                      Node(q.pos + [ 1, 0], parent=q),
                      Node(q.pos + [-1,-1], parent=q),
                      Node(q.pos + [ 0,-1], parent=q),
                      Node(q.pos + [ 1,-1], parent=q)]

      # for successor in descendants
      for s in descendants:
        # if outside map
        if s.pos[1]>=self.map.shape[0] or s.pos[0]>=self.map.shape[1]:
          break
        # if on an obstacle
        if self.map[s.pos[1], s.pos[0]] == 0.0:
          break

        # if undiscovered
        if s not in open and s not in closed:
              # s.g = self.distance(s.pos, q.pos) + q.g
              s.g = 1
              s.h = self.distance(s.pos, end)
              s.f = s.g + s.h

              open.append(s)
        # if discovered previously
        else:
          print("THIS WILL NEVER PRINT")
          if s.g > self.distance(s.pos, q.pos)+q.g: #TODO: fix for other distances
            # s.g = self.distance(s.pos, q.pos)+q.g
            s.g = 1
            s.h = self.distance(s.pos, end)
            s.f = s.g + s.h

            if s in closed:
              # print("==============================")
              # print("CLOSED BEFORE POP: ", len(open))
              # print("CLOSED AFTER POP: ", len(open))
              closed.pop(closed.index(s))
              open.append(s)

        self.update_overlay(s.pos, np.array([.7,.7,.7]))

        # print(len(open))
        # print(len(closed))

      closed.append(q)
      toc2 = time.perf_counter()
        
      self.update_overlay(q.pos, np.array([0, 1, 0]))
      self.update_overlay(self.last_drawn, np.array([1,1,0]))
      self.last_drawn = q.pos

      self.show()

      toc3 = time.perf_counter()
      
      # print("OPEN LENGTH", len(open))
      # print("CLOSED LENGTH", len(closed))
      # print("TimeDelta1: ", toc1 - tic)
      # print("TimeDelta2: ", toc2 - toc1)
      # print("TimeDelta3: ", toc3 - toc2)
    
  def distance(self, p1, p2):
    return np.linalg.norm(p1 - p2)

  def update_overlay(self, point, color):
    self.overlay[point[1], point[0], :] = color
    
  def show(self):
    self.viewer.clear()
    self.viewer.imshow((self.map_img*self.overlay).astype('uint8'), interpolation='nearest')

    self.fig.canvas.draw()
    plt.pause(.01)

  def create_overlay(self, start, end):
    overlay = np.ones(np.append(self.map.shape,3))   #Add 3 layers to create an RGB image

    overlay = cv2.circle(overlay, start, 0, (1, .5, 0), 1)
    overlay = cv2.circle(overlay, end, 0, (1, .5, 0), 1)

    print("PLOTTED END: ", end)

    # Works at resolution: 100
    cv2.putText(overlay, "Start", start - np.array([-5,0]), cv2.FONT_HERSHEY_SIMPLEX, .25, (0,0,0.5), 1)
    cv2.putText(overlay, "End", end - np.array([-5,0]),     cv2.FONT_HERSHEY_SIMPLEX, .25, (0,0,0.5), 1)

    return overlay