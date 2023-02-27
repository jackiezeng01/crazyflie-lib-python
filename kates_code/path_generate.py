from numpy import arctan2
path = [0,0,1], [0,.5,1], [0,1,1], [.5,1,1], [1,1,0], [1, .5, 1], [1,0,1] 

for index,node in enumerate(path):
    if index != len(path) - 1:
        vector = [path[index + 1][0] - node[0], path[index + 1][1] - node[1]]
        angle = arctan2(vector[1], vector[0])
        path[index].append(angle)
    else:
        path[index].append(path[index - 1][3])

print(path)

