# AI Project 1:
# Teammates:
# 1. Rohaan Advani - rna3535
# 2. Kirthan Reddy  - kr3026

import heapq
import math
import sys

# List of all possible directions the robot could move in the grid
directions = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]

# Used to make the origin as the top-left corner initially to make it easier for us to work on the grid
def flipGrid(grid):
    return grid[::-1]

# Check if input grid is valid as per the instructions of input format
def checkInputGrid(start, goal, grid):
    # Number of rows = 30
    # Number of cols = 50
    # Start position depicted by '2' in grid
    # End position depicted by '5' in grid
    if len(grid) == 30 and len(grid[0]) == 50 and grid[goal[1]][goal[0]] == 5 and grid[start[1]][start[0]] == 2:
        return True
    return False

# Read input file returns start, goal, and grid
def read_input(file_path):
    with open(file_path, 'r') as file:
        # Read start and goal positions
        start_goal_line = file.readline().strip()
        start_i, start_j, goal_i, goal_j = map(int, start_goal_line.split())
        start = (start_i, start_j)
        goal = (goal_i, goal_j)
        # Read the workspace grid
        grid = []
        for _ in range(30):  # 30 rows as specified
            row = list(map(int, file.readline().strip().split()))
            grid.append(row)
    grid = flipGrid(grid)
    res = checkInputGrid(start, goal, grid)
    if not res:
        print('Inavlid input file. Please check specifications.')
        sys.exit(0)
    return start, goal, grid, res

def heuristicFunction(currPos, goalPos):
    # Euclidean distance as a heuristic, ensuring non-negative values
    return math.sqrt((goalPos[0] - currPos[0]) ** 2 + (goalPos[1] - currPos[1]) ** 2)

# Create a list of all valid neighbours
def get_neighbors(position, grid):
    neighbors = []
    # Use directions list to compute all possible neighbour indices in grid
    for direction in directions:
        new_i = position[0] + direction[0]
        new_j = position[1] + direction[1]
        # Do not include neighbours if out of bounds or black cells
        if 0 <= new_i < len(grid[0]) and 0 <= new_j < len(grid) and grid[new_j][new_i] != 1:
            neighbors.append([(new_i, new_j), direction])
    return neighbors

# Calculate stepcost = angular + distance cost
def stepCost(k, start, currPos, nextPos, currDirn):
    direction = (nextPos[0] - currPos[0], nextPos[1] - currPos[1])
    # Ensure distCost calculation only involves real numbers
    distCost = math.sqrt(direction[0] ** 2 + direction[1] ** 2)
    if distCost == 1.0 or distCost == math.sqrt(2):
        # Handle angleCost calculation
        if currPos == start:
            angleCost = 0 # Angle cost is 0 for start as mentioned in question
        else:
            # Calculate difftheta
            diffTheta = abs(directions.index(direction) - directions.index(currDirn)) * 45
            # Adjust difftheta if more than 180
            if diffTheta > 180:
                diffTheta = 360 - diffTheta
            angleCost = k * (diffTheta / 180)
        return distCost + angleCost
    else:
        print(direction)
        print(distCost)
        print('Invalid distance cost while calculating step cost for neighbour.')
        sys.exit(0)

# Run the A* search algorithm
def a_star_search(start, goal, grid, k):
    # Open list is the Frontier (Sorted based on f(n) values of nodes)
    open_list = []
    heapq.heappush(open_list, (heuristicFunction(start, goal), start, None))
    # Came from keeps track of the parent node
    came_from = {start: None}
    # g_score / f_score dictionaries keep track of the scores
    g_score = {start: 0}
    f_score = {start: heuristicFunction(start, goal)}

    nodes_generated = 1 #start node
    
    # While frontier not empty
    while open_list:
        f_score_curr, current, currDirn = heapq.heappop(open_list)
        # If goal found in frontier return solution
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            # Reverse path (Start goes to begining and Goal to end)
            path.reverse()
            return path, f_score, nodes_generated
        
        # If goal not found as current node explore neighbours to add to tree
        for neighbor in get_neighbors(current, grid):
            step_cost = stepCost(k, start, current, neighbor[0], currDirn)
            # Update g_score of neighbours
            tentative_g_score = g_score[current] + step_cost
            tentative_f_score = tentative_g_score + heuristicFunction(neighbor[0], goal)
            # If lower f_score found for neighbor update dictioaries
            if neighbor[0] not in g_score or tentative_f_score < f_score[neighbor[0]]:
                nodes_generated += 1
                came_from[neighbor[0]] = current
                g_score[neighbor[0]] = tentative_g_score
                f_score[neighbor[0]] = tentative_f_score
                # Push new values to heap for resorting for next iteration
                if isinstance(f_score[neighbor[0]], float) and not isinstance(f_score[neighbor[0]], complex):
                    heapq.heappush(open_list, (f_score[neighbor[0]], neighbor[0], neighbor[1]))
    # If no solution found
    return None, float('inf'), len(came_from) 

def write_output(file_path, path, f_score, total_nodes, grid):
    with open(file_path, 'w') as file:
        # Depth of goal node = len(path) [LINE 1]
        file.write(f"{len(path)}\n")
        # len(came_from) = total_nodes [LINE 2]
        file.write(f"{total_nodes}\n")
        
        if path:
            moves = []
            f_score_print_list = [f_score[path[0]]]
            for i in range(1, len(path)):
                # Print index of appropriate direction
                di = path[i][1] - path[i - 1][1]
                dj = path[i][0] - path[i - 1][0]
                moves.append(directions.index((di, dj)))
                # Print f_score of each node in solution path
                f_score_print_list.append(f_score[path[i]])
                # Number of f_scores should be one more than number of directions
                if len(f_score_print_list) != (len(moves) + 1):
                    print('Error in output format creation.')
                    sys.exit(0)
                # Replace path grid indices with '4' instead of '0'
                if i < len(path) - 1:
                    grid[path[i][1]][path[i][0]] = 4
            file.write(" ".join(map(str, moves)) + "\n") # [LINE 3]
            file.write(" ".join(map(str, f_score_print_list)) + "\n") # [LINE 4]
        
        # Updated grid
        grid = flipGrid(grid)
        for i in range(len(grid)):
            line = " ".join(map(str, grid[i]))
            file.write(line + "\n") # [LINE 5-34]

# MAIN
file_path = input('Enter filepath / filename: ')
start, goal, grid, res = read_input(file_path)
k = int(input('Enter value for k: '))
path, f_score, total_nodes = a_star_search(start, goal, grid, k)
output_file_path = input('Enter filepath / filename of output file you wish to create: ')
write_output(output_file_path, path, f_score, total_nodes, grid)