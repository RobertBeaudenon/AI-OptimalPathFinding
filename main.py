# -------------------------------------------------------
# Assignment 1
# Written by Robert Beaudenon 40022364
# For COMP 472 Section JX – Summer 2020
# --------------------------------------------------------

import math
import numpy as np
import shapefile as shp
import matplotlib.pyplot as plt
from matplotlib import colors
import pandas as pd
import sys
from timeit import default_timer as timer

""" 

    Node class to represent elements of the grid
    :param parent
    :param position

"""


class Node():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


"""

    Calculates the cost of g , moving from one node to a target node
    It will also help us filter for A* the nodes that we don't want to include in the 
    open list by returning a Big number with sys.maxsize 
    :param maze
    :param current_node
    :param target_node
    :param end_node
    :param treshold
    :param size
    
"""


def cost(maze, current_node, target_node, end_node, threshold, size):
    # Collect row and column of the current node
    current_row = current_node.position[1]
    current_column = current_node.position[0]

    # Collect row and column of the target node
    target_row = target_node.position[1]
    target_column = target_node.position[0]

    # Check if the column are out of bound
    if current_column < 0 or target_column < 0 or current_column > size or target_column > size:
        return sys.maxsize

    # Check if the rows are out of bound
    if current_row < 0 or target_row < 0 or current_row > size or target_row > size:
        return sys.maxsize

    # In order to identify what kind of movement we are doing : diagonal , vertical, horizontal : to identify the checks
    horizontal_movement = target_column - current_column
    vertical_movement = target_row - current_row

    # If we reach the boundaries and it's not the end node
    if target_node != end_node:
        if target_row == 0 or target_column == 0 or target_column == size or target_row == size:
            return sys.maxsize

    # 1 - To check for diagonal movements : if horizontal movement is 1 or -1 and vertical movement is 1 or -1
    # All the edges between two low crime rate areas and diagonal are accessible. The cost diagonal cost = 1.5
    if horizontal_movement != 0 and vertical_movement != 0:

        # Setting column to check if we moved to the left or right
        if horizontal_movement > 0:
            new_column = current_column
        else:
            new_column = current_column - 1

        # Setting row to check if we moved to the up or down
        if vertical_movement > 0:
            new_row = current_row
        else:
            new_row = current_row - 1

        # Once we know in which direction our diagonal is ( 4 different choices ) then we check the particular grid
        # to see if its a block or not
        if maze[new_column, new_row] < threshold:
            return 1.5
        else:
            return sys.maxsize

    # 2 - To check for horizontal movements
    if vertical_movement == 0:

        new_column = current_column

        # if we are moving leftwards
        if horizontal_movement < 0:
            new_column = current_column - 1

        # if we are moving rightwards
        if horizontal_movement > 0:
            new_column = current_column

        # All the edges between two low crime rate areas. The cost of this edge = 1
        if maze[new_column, current_row - 1] < threshold and maze[new_column, current_row] < threshold:
            return 1

        # All the edges between one block (yellow) and one low crime rate area (blue) are
        # accessible and the cost of this edge is 1.3
        if maze[new_column, current_row - 1] < threshold or maze[new_column, current_row] < threshold:
            return 1.3

        # All the edges between two block areas and its diagonal are forbidden
        return sys.maxsize

    # 3 - To check for vertical movements
    if horizontal_movement == 0:

        new_row = current_row

        # if we are moving downwards
        if vertical_movement < 0:
            new_row = current_row - 1

        # if we are moving upwards
        if vertical_movement > 0:
            new_row = current_row

        # All the edges between two low crime rate areas. The cost of this edge = 1
        if maze[current_column - 1, new_row] < threshold and maze[current_column, new_row] < threshold:
            return 1

        # All the edges between one block (yellow) and one low crime rate area (blue) are
        # accessible and the cost of this edge is 1.3
        if maze[current_column - 1, new_row] < threshold or maze[current_column, new_row] < threshold:
            return 1.3

        # All the edges between two block areas and its diagonal are forbidden
        return sys.maxsize


"""

    Calculates the cost of the heuristic h,w we compute the number of steps you take if you can’t take a diagonal,
    then subtract the steps you save by using the diagonal. There are min(dx, dy) diagonal steps, 
    and each one costs D2 but saves you 2⨉D non-diagonal steps.
    :param current_node
    :param goal_node

"""


def diagonal_heuristic(current_node, goal_node):
    x = abs(current_node[0] - goal_node[0])
    y = abs(current_node[1] - goal_node[1])
    return (x + y) + (1.5 - 2 * 1) * min(x, y)


"""  

    Returns a list of tuples as a path from the given start to the given end in the given maze 
    :param maze
    :param start
    :param end
    :param threshold
    :param size
    
"""


def astar(maze, start, end, threshold, size):
    # start timer
    start_time = timer()

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        # Check if we are over 10s
        end_time = timer()

        # calculate time and print it
        time = end_time - start_time
        if time > 10:
            print('Time is up. the optimal path was not found.')
            sys.exit()

        # Pop current off open list, add to closed list
        current_node = open_list.pop(0)
        closed_list.append(current_node)

        # Found the goal node
        if current_node == end_node:
            # stop timer when the goal is found
            end_time = timer()

            # calculate time and print it
            time = end_time - start_time
            print('Time : ', time)

            # initializing path list that will contains the nodes to the goal
            path = []
            current = current_node

            # getting total cost of path
            # g cost of parent of goal node + g cost of getting from parent of goal node to goal node
            total_cost = current_node.parent.g + cost(maze, current_node.parent, end_node, end_node, threshold,
                                                      size)
            print('Total cost : ', total_cost)

            # looping through parents to find the path
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate 8 neighbours : this loop can be optimized using a numpy 2D array or by writing down the operations
        # on each neighbour
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            # Create the position of the new neighbour
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Checking that we are within the acceptable range of the maze
            if node_position[0] > (size - 1) or node_position[0] < 0 or node_position[1] > (size - 1) or node_position[
                1] < 0:
                continue

            # Create new node
            child = Node(current_node, node_position)

            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + cost(maze, current_node, child, end_node, threshold, size)
            child.h = diagonal_heuristic(child.position, end_node.position)
            child.f = child.g + child.h

            # If you can't access the node and it's the goal return
            if child.g >= sys.maxsize and child.position == end_node.position:
                return None

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # If you can't access the node
            if child.g >= sys.maxsize:
                continue

            # If the node meets all the conditions add it to the openlist and sort it
            open_list.append(child)
            open_list.sort(key=lambda item: item.f)


"""

    Handles onclick event on the maze to specify start node and end node.
    It will be responsible of drawing the path once returned.
    :param event
    
"""


def onclick(event):
    global ix, iy

    if event.inaxes is not None:
        ix, iy = event.xdata, event.ydata

        global coords
        coords.append((ix, iy))
    else:
        print('Click inside the window')  # user should click inside the bounderies of the grid

    if len(coords) == 2:
        print(coords)

        # disabling click if we already have start and end points
        fig.canvas.mpl_disconnect(clickID)

        # converting latitude to index in array for the start node
        start = (int((coords[0][1] - y_min) / cell_size), int((coords[0][0] - x_min) / cell_size))

        # converting longitude to index in array for the end node
        end = (int((coords[1][1] - y_min) / cell_size), int((coords[1][0] - x_min) / cell_size))

        # calling A star on both nodes
        path = astar(dict_of_coordinates, start, end, threshold, rows)

        print('Path : ', path)
        print('Thank you, see you next time.')

        # mapping and plotting points returned by A* to grid latitude longitude using the panda Dataframe generated
        if path is not None:
            for i in range(len(path) - 1):
                point1X = dataframe.loc[path[i][1], path[i][0]][0]
                point1Y = dataframe.loc[path[i][1], path[i][0]][1]
                point2X = dataframe.loc[path[i + 1][1], path[i + 1][0]][0]
                point2Y = dataframe.loc[path[i + 1][1], path[i + 1][0]][1]
                x_t = [point1X, point2X]
                y_t = [point1Y, point2Y]
                ax.plot(x_t, y_t)
        else:
            print('Due to blocks, no path is found. Please change the map and try again')

        plt.draw()  # re-draw the figure
        plt.pause(0.000000000001)

    return coords


# getting path of shape file
shpfile = "Shape/crime_dt.shp"

# manually adding ISO instead of opening the file
shape = shp.Reader(shpfile, encoding="ISO-8859-1")

# checkout in the file and find min max for x and y
x_min, y_min, x_max, y_max = shape.bbox

# user input for grid size
cell_size = float(input("Either choose 0.002 or 0.003 as the cell size: "))

# user input for threshold
thresholdFromUser = float(input("Please input the desired threshold from 0 to 100:"))

# creating rows and column
rows = math.ceil((x_max - x_min) / cell_size)
columns = math.ceil((y_max - y_min) / cell_size)

# create the grid based on the number of rows and columns calculated and initialize each cell to 0
grid = np.array([[0] * rows] * columns)

# create a grid that will give a translation of points containing the path to their actual coordinates
gridToCoord = np.array([[0] * rows] * columns)

# creating pandas dataframe of the same size ad the gridtoCord in order to insert (latitude, longitude) objects
# into specific indexes which is not possible using numpy
dataframe = pd.DataFrame(gridToCoord.reshape(rows, columns))
dataframe = dataframe.astype(object)  # converting column types in order to insert my objects: (latitude, longitude)

# to keep track of number of crimes for each cell in the grid
gridCrimeDict = {}

maxCrimes = 0

# returns all the shapes as a list
shapeRecords = shape.shapeRecords()

# initializing list of coordinates grids
x1 = []
y2 = []

# initializes each block cell with its corresponding crime rate
for i in range(len(shapeRecords)):

    # extracting points from shape file
    latitude = shapeRecords[i].shape.__geo_interface__["coordinates"][0]
    longitude = shapeRecords[i].shape.__geo_interface__["coordinates"][1]
    x1.append(latitude)
    y2.append(longitude)

    # getting latitude and longitude and converting it to x and y index based on numpy 2D array boundaries
    x = int(
        (latitude - x_min) / cell_size)
    y = int(
        (longitude - y_min) / cell_size)

    # If a crime was made in this coordinate then we increment the value of the cell
    grid[x][y] = grid[x][y] + 1

    # populating dictionary with keys in order to get max value
    if (x, y) in gridCrimeDict.keys():
        gridCrimeDict[x, y] = gridCrimeDict[x, y] + 1
    else:
        gridCrimeDict[x, y] = 1
    if maxCrimes < gridCrimeDict[x, y]:
        maxCrimes = gridCrimeDict[x, y]

grid = grid.transpose()

# initializing list of coordinates
coords = []

# When the threshold is 100% we don't want to display any yellow block
if thresholdFromUser != 100:
    cmap = colors.ListedColormap(['purple', 'yellow'])
else:
    cmap = colors.ListedColormap(['purple', 'purple'])

# converting numpy 2D array into a numpy 1D array
grid1D = grid.flatten()
gridDescending = sorted(grid1D, reverse=True)  # sorting the array in descending order

# Based on the threshold that the user specified, we can apply it on the descending array
# in order to find the cutoff value
threshold = np.percentile(gridDescending,
                          thresholdFromUser)

# ax stands for axes, we can think of it as a plot that is set to a single avis/plot that's because
# subplots creates a figure and specify a certain number of rows and columns (by default 1x1)
fig, ax = plt.subplots()

bounds = [0, threshold, maxCrimes]  # creating color bounds for the grid
norm = colors.BoundaryNorm(bounds, cmap.N)
h = ax.hist2d(x1, y2, bins=rows, cmap=cmap,
              norm=norm)  # plotting grid that returns an array 'h' of details about the grid that we can explore
clb = fig.colorbar(h[3], ax=ax)

# computing average and standard deviation
average = np.mean(gridDescending)
std = np.std(gridDescending)

# adding titles to plot
clb.ax.set_title('Threshold')
plt.title('Crime Rate Map\n' + 'Average : ' + str(average) + '\nStandard Deviation : ' + str(std))

# adding axes ranges to the plot
axes = plt.gca()
axes.set_ylim([45.490, 45.530])
axes.set_xlim([-73.590, -73.550])

# adding onclick event to select start and end points
clickID = fig.canvas.mpl_connect('button_press_event', onclick)

# displaying in console the 2D array of crime rates that maps the plot
array_of_crimes = h[0].transpose()
print(np.flip(array_of_crimes, axis=0))
print(type(h[0]))

# creating a dictionary mapping the position in the array to the value of the crime rate for faster access time
dict_of_coordinates = {}
i = 0
j = 0
for i in range(columns):
    for j in range(rows):
        dict_of_coordinates[i, j] = array_of_crimes[i][j]

# populating dataframe mapping the position of the array to the coordinates (latitude, longitude) of the grid
i = 0
j = 0
for i in range(columns):
    for j in range(rows):
        dataframe.loc[i][j] = [h[1][i], h[2][j]]

plt.show()
