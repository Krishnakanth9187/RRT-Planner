import numpy as np
from math import *
start = np.array([0, 0])
goal = np.array([5,9])
grid = np.array([[0, 0, 0, 0, 0, 0, 0, 1, 0, 0],  # Row 0
                 [0, 1, 1, 0, 0, 0, 0, 1, 0, 0],  # Row 1
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # Row 2
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # Row 3
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # Row 4
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # Row 5
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # Row 6
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # Row 7
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],  # Row 8
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]) # Row 9
        # Columns 0  1  2  3  4  5  6  7  8  9

# Copies of grid to be used for visualizing results.
path = np.zeros([len(grid), len(grid)], dtype=int)
best_path = np.zeros([len(grid), len(grid)], dtype=int)
h_grid = np.zeros([len(grid), len(grid)], dtype=float)


class AStarSearch:
    def __init__(self, start, goal, grid, path, h_grid):
        self.pos = start
        self.pos_str = str(start)
        self.pos_depth = 0
        self.goal_str = str(goal)
        self.explored = {}
        self.not_explored = {}
        self.not_explored[str(start)] = self.heuristic(start)
        self.grid = grid
        self.path = path
        self.h_grid = h_grid

    # START - Student Section

    def get_possible_moves(self):
        potential_moves = self.generate_potential_moves(self.pos)
        for move in potential_moves:
            # Check if potential move is valid.
            if not self.valid_move(move):
                continue
            # Check if move has already been explored.
                # Visualize the Heuristic Grid
            if str(move) not in self.not_explored and str(move) not in self.explored:
                h_val = self.heuristic(move)
                self.h_grid[move[0], move[1]] = h_val
                self.not_explored[str(move)] = h_val  
                print(f"==============={self.not_explored}")
            self.explored[self.pos_str] = 0

        # Since all next possible moves have been determined,
        # consider current location explored.
        return True

    def goal_found(self):
        if str(goal) in self.not_explored:
            self.pos = goal
            self.pos_depth = self.not_explored.pop(str(goal))
            self.path[self.pos[0], self.pos[1]] = self.pos_depth
            return True
        else:
            return False



    def explore_next_move(self):
        # Determine next move to explore.
        sorted_not_explored = sorted(
            self.not_explored,
            key=self.not_explored.get,
            reverse=False)

        # Determine the pos and depth of next move.
        print(f"sorted_not_explored============{sorted_not_explored}")
        self.pos_str = sorted_not_explored[0]
        self.pos = self.string_to_array(self.pos_str)
        self.pos_depth = self.not_explored.pop(self.pos_str)
        self.path[self.pos[0], self.pos[1]] = self.pos_depth
        print(f"{self.path}")
        return True
        
        # Write depth of next move onto path.

        return True

    def heuristic(self, move):
        h_val = sqrt(pow((goal[0] - move[0]),2) + pow((goal[1] - move[1]),2))
        return round(h_val, 1)

    # END - Student Section

    # Helper Functions

    def generate_potential_moves(self, pos):
        u = np.array([-1, 0])
        d = np.array([1, 0])
        l = np.array([0, -1])
        r = np.array([0, 1])

        potential_moves = [pos + u, pos + d, pos + l, pos + r]
        # Students, uncomment the line below,  what happens?
        potential_moves += [pos + u+r, pos + u+l, pos + d+r, pos + d+l]
        return potential_moves

    def valid_move(self, move):
        # Check if out of boundary.
        if (move[0] < 0) or (move[0] > 9):
            return False
        if (move[1] < 0) or (move[1] > 9):
            return False
        # Check if wall or obstacle exists.
        if self.grid[move[0], move[1]] == 1:
            return False
        return True

    def string_to_array(self, string):           
        print(f"string =========={string}")
        array = [int(string[1]), int(string[3])]
        return np.array(array)


# Init
astar = AStarSearch(start, goal, grid, path, h_grid)

while True:
    # Determine next possible moves.
    astar.get_possible_moves()
    if astar.goal_found():
        break
    astar.explore_next_move()


print('')
print('Heuristic Grid')
print('--------------')
print(h_grid)
print('')
print('')
print('Explored Path')
print('-------------')
path[start[0], start[1]] = 9999
print(path)
print('')
print('Fully explored count ' + str(np.count_nonzero(path)))


def find_best_path(pos):
    best_path[pos[0], pos[1]] = 1
    h_pos = path[pos[0], pos[1]]
    print(f"{best_path}")
    if h_pos == 1:
        return 1

    potential_moves = astar.generate_potential_moves(pos)
    best_move = [0, 0]
    best_h = h_pos
    for move in potential_moves:
        if not astar.valid_move(move):
            continue
        h_move = path[move[0], move[1]]
        if h_move <= best_h and h_move != 0:
            best_h = h_move
            best_move = move
    return find_best_path(best_move) + 1


goal_count = find_best_path(goal)
best_path[start[0], start[1]] = 9999
print('')
print('Best Path To Goal')
print('-----------------')
print(best_path)
print('')
print('Moves to Goal: ' + str(goal_count))
print('')
