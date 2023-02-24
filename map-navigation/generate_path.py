import cv2
import numpy as np
import matplotlib.pyplot as plt

class Node:
  def __init__(self, pos, f=0, g=0, h=0, parent=None):
    self.pos = pos
    self.f = f
    self.g = g
    self.h = h
    self.parent = parent



class path_generator:
  def __init__(self, map):
    self.map = map
    
    self.map_img = np.repeat(self.map[:, :, np.newaxis], 3, axis=2)
    
    self.overlay = None
    self.last_drawn = None

    self.fig = plt.figure()
    self.viewer = self.fig.add_subplot(111)

    self.viewer.tick_params(labelbottom=False)
    self.viewer.tick_params(labelleft=False)

    # self.last_drawn = None

  # def generate2(self, start, end):
  #   open = set(start)
  #   closed = set()
  #   g = {}    # dist from start
  #   parents = {}

  #   g[start] = 0

  #   parents[start] = start

  #   while open:
  #     n = none
  #     for v in open:
  #       if n == None or g[v] + heuristic(v) < g[n] + heuristic(n)
  #         n = v
      
  #     if n == end or self.map[end] == None:
  #       pass
  #     else:
  #       for (m, weight) in get_neighbors(n):
  #         if m not in open and m not in closed:
  #           open.add(m)
  #           parents[m] = n
  #           g[m] = g[n] + weight

  #         else:
  #           if g[m] > g[n] + weight:
  #             g[m] = g[n] + weight

  #             parents[m] = n

  #             if m in closed:
  #               closed.remove(m)
  #               open.add(m)







  def generate(self, start, end):
    self.overlay = self.create_overlay(start, end)
    self.last_drawn = start

    open = [Node(start)]
    closed = []

    while open:
      q = open.pop(open.index(min(open, key=lambda x:x.f)))

      if q.pos.all == end.all or q.pos[1]>self.map.shape[1]:   # TODO: Add check if pos is within graph
        pass
      
      # set each neighbor as distance 1 from q
      descendants = [ Node(q.pos + [-1, 1], g=1+q.g, parent=q),
                      Node(q.pos + [ 0, 1], g=1+q.g, parent=q),
                      Node(q.pos + [ 1, 1], g=1+q.g, parent=q),
                      Node(q.pos + [-1, 0], g=1+q.g, parent=q),
                      Node(q.pos + [ 1, 0], g=1+q.g, parent=q),
                      Node(q.pos + [-1,-1], g=1+q.g, parent=q),
                      Node(q.pos + [ 0,-1], g=1+q.g, parent=q),
                      Node(q.pos + [ 1,-1], g=1+q.g, parent=q)]

      # for successor in descendants
      for s in descendants:
        s.h = self.distance(s.pos, end)

        s.f = s.g + s.h

        if s not in open and s not in closed:
          open.append(s)
        else:
          if s.g > q.g + 1: #TODO: fix for other distances
            s.g = q.g + 1

            if s in closed:
              closed.remove(closed.index(s))
              open.append(s)

      closed.append(q)

      self.update_overlay(q.pos)
    
  def distance(self, p1, p2):
    return np.linalg.norm(p1 - p2)

  def update_overlay(self, point):
    self.overlay[point[0], point[1], :] = np.array([100, 255, 0])
    self.overlay[self.last_drawn[0], self.last_drawn[1], :] = np.array([0, 255, 0])
    
    self.viewer.imshow(self.map_img*self.overlay, interpolation='nearest')

    self.last_drawn = point

    self.fig.canvas.draw()
    plt.pause(.05)

  def create_overlay(self, start, end):
    overlay = np.zeros(np.append(self.map.shape,3))+255   #Add 3 layers to create an RGB image

    overlay = cv2.circle(overlay, start, 0, (255, 0, 0), 1)
    overlay = cv2.circle(overlay, end, 0, (255, 0, 0), 1)

    # Works at resolution: 100
    # cv2.putText(overlay, "Start", start - np.array([-5,0]), cv2.FONT_HERSHEY_SIMPLEX, .25, (0,0,255), 1)
    # cv2.putText(overlay, "End", end - np.array([-5,0]),     cv2.FONT_HERSHEY_SIMPLEX, .25, (0,0,255), 1)

    return overlay