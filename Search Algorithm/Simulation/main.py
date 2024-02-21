from enum import Enum
from typing import List, Tuple
import random

class SquareType(Enum):
    VISITED = '.'
    EMPTY = ' '
    BLOCKED = '*'
    TARGET = ''
    HEXAPOD = 'ඬ'


class Orientation(Enum):
    HORIZONTAL = 0
    VERTICAL = 1


def print_grid(grid: List[List[SquareType]]):
    for col in range(len(grid[0]) + 2):
        print(SquareType.BLOCKED.value, end=" ")
    print()
    for row in range(len(grid)):
        print(SquareType.BLOCKED.value, end=" ")
        for col in range(len(grid[0])):
            print(grid[row][col].value, end=" ")
        print(SquareType.BLOCKED.value)
    for col in range(len(grid[0]) + 2):
        print(SquareType.BLOCKED.value, end=" ")
    print()
        


def change_squares(grid: List[List[SquareType]], orientation: Orientation, offset: int, 
                start: int, end: int, square_type: SquareType = SquareType.BLOCKED) -> None:
    if orientation == Orientation.HORIZONTAL:
        for col in range(start, end):
            grid[offset][col] = square_type
    else:  # orientation == Orientation.VERTICAL
        for row in range(start, end):
            grid[row][offset] = square_type


def generate_building_grid() -> List[List[SquareType]]:
    """
    Creating map specified in https://ieeexplore.ieee.org/document/1389384
    Assume edges of grid are walls (BLOCKED)

    TODO: Use Tkinter to create graphical UI to set walls
    """
    # Overall building
    building_dim = 5  # m
    resolution = 0.2  # m. Assumed range of ultrasonic
    num_squares = int(building_dim / resolution)
    building = [[SquareType.EMPTY for i in range(num_squares)] for j in range(num_squares)]

    # Add rooms
    third = int(1 / 3 * num_squares)
    half = int(1 / 2 * num_squares)
    change_squares(building, Orientation.HORIZONTAL, third, 0, third + 1)
    change_squares(building, Orientation.VERTICAL, third, 0, third)

    change_squares(building, Orientation.HORIZONTAL, 2 * third, 0, third)
    change_squares(building, Orientation.VERTICAL, third, 2 * third, num_squares)

    change_squares(building, Orientation.VERTICAL, 2 * third, 0, num_squares)
    change_squares(building, Orientation.HORIZONTAL, half, 2 * third, num_squares - 4)

    # Add doorways
    sixth = int(1 / 6 * num_squares)
    change_squares(building, Orientation.VERTICAL, third, sixth - 2, sixth + 2, SquareType.EMPTY)
    change_squares(building, Orientation.HORIZONTAL, 2 * third, sixth - 2, sixth + 2, SquareType.EMPTY)

    change_squares(building, Orientation.VERTICAL, 2 * third, sixth - 2, sixth + 2, SquareType.EMPTY)
    change_squares(building, Orientation.VERTICAL, 2 * third, 5 * sixth - 2, 5 * sixth + 2, SquareType.EMPTY)

    # Add targets
    building[0][0] = SquareType.TARGET
    building[0][num_squares - 1] = SquareType.TARGET
    building[half+1][2 * third + 1] = SquareType.TARGET
    
    # Add Hexapods
    building[5 * sixth][sixth] = SquareType.HEXAPOD

    return building


def find_hexapods(building: List[List[SquareType]]) -> List[Tuple[int, int]]:
    res = []
    for row in range(len(building)):
        for col in range(len(building[0])):
            if building[row][col] == SquareType.HEXAPOD:
                res.append((row, col))
    return res


def simulate_step(building: List[List[SquareType]], optimize: bool = False) -> None:
    """
    optimize: avoid revisiting previously visited nodes
    # TODO: Walk away from other hexapods
    """
    directions = [(i, j) for i in range(-1, 2) for j in range(-1, 2)]
    directions.remove((0, 0))
    rows, cols = len(building), len(building[0])
    hexapods = find_hexapods(building)
    for hexapod in hexapods:
        row, col = hexapod
        while True:
            drow, dcol = random.choice(directions)
            new_row, new_col = row + drow, col + dcol
            if (0 <= new_row < rows and 0 <= new_col < cols and 
                building[new_row][new_col] in [SquareType.EMPTY, SquareType.VISITED]):
                # Unoccupied square, feel free to enter
                break
        building[new_row][new_col] = SquareType.HEXAPOD
        building[row][col] = SquareType.VISITED
        # TODO: Check if a target
    return


if __name__ == '__main__':
    building = generate_building_grid()
    print_grid(building)
    for step in range(500):
        simulate_step(building)
    print_grid(building)

