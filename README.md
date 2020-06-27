# Artificial Intelligence - Path Finding using A* Algorithm.

In this program I generate a crime risk map based on the crime rates and
implement an A* heuristic search algorithm to find an optimal path between two
coordinates on the map.

## Getting Started
### Prerequisites

List of libraries to install:

```
math
numpy
shapefile
matplotlib
pandas
sys
timeit
```

## Running the program

- Run the python code
- You will be prompted by the console with two inputs:
   - Input the desired grid size
   - Input the desired threshold in %
- The corresponding plot will pop up
- Choose two grid for your paths by clicking on them on the plot
- As soon as you finish clicking on the second grid A* will be triggered and the output will be printed in the console
- You will find in the console:
   - Total Cost
   - Execution time in case its under 10s
   - Start and end point coordinates ( in terms of latitude longitude of the grid )
   - The list of points that builds the path ( in terms of indexes of a 2D array )

## Notes

- If you choose 0.003 as your grid size :
When you click on the grid to choose your start and end nodes you might see when the path is drawn that your start and end points are not where you clicked:
This is happening because since the grid is bigger than 0.002 when i convert the latitude and longitude to integer to get the mapping of the latitude and longitude to my int array indexes values
it can have a +/- 1 difference when the rounding happens depending on where exactly you clicked within the particular grid. So make sure to be precise on where you click.

- Instead of printing the number of crime rates on the grid which make it messy i print them in the console in the form of a numpy 2D array.
## Authors

* **Robert Beaudenon**


