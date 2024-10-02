import cv2
import numpy as np

def gridMaker(img):
    for i in range(49):
        cv2.line(img, ((i+1)*10, 0), ((i+1)*10, 499), (255,255,255), 1)
        cv2.line(img, (0, (i+1)*10), (499, (i+1)*10), (255,255,255), 1)
    cv2.rectangle(img, (0,0), (499, 10), (255,255,255), -1)
    cv2.rectangle(img, (0,490), (499, 499), (255,255,255), -1)
    cv2.rectangle(img, (0,0), (10, 499), (255,255,255), -1)
    cv2.rectangle(img, (490, 0), (499, 499), (255,255,255), -1)

    cv2.rectangle(img, (0, 40), (70, 50), (255,255,255), -1)
    cv2.rectangle(img, (110, 0), (120, 90), (255,255,255), -1)
    cv2.rectangle(img, (350, 40), (400, 100), (255,255,255), -1)
    cv2.rectangle(img, (300,300), (450,450), (255,255,255), -1)
    cv2.rectangle(img, (0,150), (350,160), (255,255,255), -1)
    cv2.rectangle(img, (100, 150), (110, 390), (255,255,255), -1)
    cv2.rectangle(img, (220, 499), (230, 250), (255,255,255), -1)

def findNeighbours(node, graph, img, vis):
    l = []
    neighbours = []
    if ((node[0]+10, node[1]) in graph):
        l.append((node[0]+10, node[1]))
        highlight((node[0]+10, node[1]), blue, img)
    if ((node[0]-10, node[1]) in graph):
        l.append((node[0]-10, node[1]))
        highlight((node[0]-10, node[1]), blue, img)
    if ((node[0], node[1]+10) in graph):
        l.append((node[0], node[1]+10))
        highlight((node[0], node[1]+10), blue, img)
    if ((node[0], node[1]-10) in graph):
        l.append((node[0], node[1]-10))
        highlight((node[0], node[1]-10), blue, img)
    for i in l:
        if i not in vis:
            neighbours.append(i)
    return neighbours

def highlight(pt, color, img):
    cv2.rectangle(img, (pt[0]-5, pt[1]-5), (pt[0]+5, pt[1]+5), color, -1)

def retracePath(node, start, img, parentTracker):
    highlight(node, green, img)
    cv2.imshow("the GRID", img)
    cv2.waitKey(25)
    if (node==start):
        return
    else:
        paths[start].insert(0, [node, 1])
        retracePath(parentTracker[node], start, img, parentTracker)

def Astar(img, graph, node, start, goal, map, vis, parentTracker, CT):
        neighbours = findNeighbours(node, graph, img, vis)
        for i in neighbours:
            if i not in parentTracker.keys():
                parentTracker[i] = node
        vis.append(node)
        if (node==goal):
            retracePath(goal, start, img, parentTracker)
            return
        highlight(node, red, img)
        for i in neighbours:
            g_score = map[node][0] + round(((i[0] - node[0])**2 + (i[1] - node[1])**2)**0.5)
            h_score = abs(goal[0] - i[0]) + abs(goal[1] - i[1])
            if i not in map.keys():
                map[i] = [g_score, h_score]
            else:
                if (map[i][0]+map[i][1] > g_score+h_score):
                    map[i][0] = g_score
                    map[i][1] = h_score
        potential_nodes = []
        min = 10000
        for i in map.keys():
            if i not in vis:
                if (map[i][0]+map[i][1])<min:
                    min = map[i][0]+map[i][1]
                    potential_nodes = [i]
                elif (map[i][0]+map[i][1])==min:
                    potential_nodes.append(i)
        new_node = node
        if len(potential_nodes)==1:
            new_node = potential_nodes[0]
        else:
            min = 10000
            for i in potential_nodes:
                if map[i][1]<min:
                    min = map[i][1]
                    new_node = i
        for i in vis:
            highlight(i, red, img)
        cv2.imshow("The GRID", img)
        cv2.waitKey(25)
        Astar(img, graph, new_node, start, goal, map, vis, parentTracker, CT)

def showAllPaths():
    global no_of_agents
    img = np.full((500,500,3), 0, dtype = np.uint8)
    gridMaker(img)
    max = len(paths[starts[0]])
    for i in paths.keys():
        if len(paths[i])>max:
            max = len(paths[i])
    for i in range(max):
        for j in paths.keys():
            if (i<len(paths[j])):
                highlight(paths[j][i][0], green, img)
        cv2.imshow("the GRID", img)
        cv2.waitKey(120)
    for i in range(no_of_agents):
        highlight(starts[i], yellow, img)
        highlight(goals[i], red, img)
    cv2.imshow("the GRID", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def loop(agent, CT):
    img = np.full((500,500,3), 0, dtype = np.uint8)
    gridMaker(img)
    graph = []
    map = {}
    vis = []
    parentTracker = {}
    for i in range(49):
        for j in range(49):
            if (img[j*10 + 5][i*10 + 5][0] == 0):
                graph.append(((i*10 + 5, j*10 + 5)))
    start = starts[agent]
    goal = goals[agent]
    paths[start] = []
    map[start] = [0,0]
    highlight(start, yellow, img)
    highlight(goal, green, img)
    cv2.imshow("The GRID", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    Astar(img, graph, start, start, goal, map, vis, parentTracker, CT)
    for i in range(len(paths[start])):
        paths[start][i][1] = i
    cv2.destroyAllWindows()
    return graph

def conflictFinder(CT, min, paths, graph):
    flag = 0
    for i in paths.keys():
        for j in paths.keys():
            if (i!=j):
                for k in range(min):
                    if (paths[i][k][0]==paths[j][k][0]):
                        CT.append((i, paths[i][k][0], paths[i][k][1]))
                        graph.remove(paths[i][k][0])
                        flag = 1
    return flag

no_of_agents = 3
starts = [(15,15), (475,415), (25,165)]
goals = [(185, 405), (135,175), (415, 155)]
blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)
yellow = (0, 255, 255)
CT = []

while True:
    paths = {}
    for i in range(no_of_agents):
        graph = loop(i, CT)
    showAllPaths()
    min = 10000
    for i in paths.keys():
        if (min>len(paths[i])):
            min = len(paths[i])
    CT = []
    result = conflictFinder(CT, min, paths, graph)
    if (result):
        print("Conflict found!!")
        continue
    else:
        print("NO CONFLICT!!!")
        break