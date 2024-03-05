from enum import Enum
from typing import List, Tuple
import random
import copy
import pandas as pd


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


def generate_building_grid(numHexapods = 1) -> List[List[SquareType]]:
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
    for i in range(numHexapods):
        # Deploy Hexapods scattered in left-bottom room
        while True:
            row, col = random.choice(range(2 * third + 1, num_squares)), random.choice(range(1, third))
            if building[row][col] == SquareType.EMPTY:
                building[row][col] = SquareType.HEXAPOD
                break

    return building


def find_obj(building: List[List[SquareType]], obj: SquareType) -> List[Tuple[int, int]]:
    res = []
    for row in range(len(building)):
        for col in range(len(building[0])):
            if building[row][col] == obj:
                res.append((row, col))
    return res


def simulate_step(building: List[List[SquareType]], optimize: bool) -> int:
    """
    Optimize: avoid revisiting previously visited nodes
    TODO: Implement
    return: Number of targets found in this simulation step
    """
    directions = [(i, j) for i in range(-1, 2) for j in range(-1, 2) if not i == 0 == j]
    rows, cols = len(building), len(building[0])
    hexapods = find_obj(building, SquareType.HEXAPOD)
    found = 0
    for hexapod in hexapods:
        row, col = hexapod
        empty_choices = []
        visited_choices = []
        for (drow, dcol) in directions:
            new_row, new_col = row + drow, col + dcol
            if 0 <= new_row < rows and 0 <= new_col < cols:
                if building[new_row][new_col] in [SquareType.EMPTY, SquareType.TARGET]:
                    empty_choices.append((new_row, new_col))
                if building[new_row][new_col] == SquareType.VISITED:
                    visited_choices.append((new_row, new_col))
            if optimize and empty_choices:  # There are empty choices
                choices = empty_choices
            else:
                choices = empty_choices + visited_choices
        new_row, new_col = random.choice(choices)
        if building[new_row][new_col] == SquareType.TARGET:
            # TODO: Notify Medbot in real simulation
            found += 1
        building[new_row][new_col] = SquareType.HEXAPOD
        building[row][col] = SquareType.VISITED
    return found


def profile_performance(num_hexapods: List[int], num_runs: int, optimize: bool = False) -> None:
    results = pd.Series(index=num_hexapods)
    for hexapods in num_hexapods:
        total_steps = 0
        for run in range(num_runs):
            building = generate_building_grid(hexapods)
            num_steps = 0
            while find_obj(building, SquareType.TARGET):
                simulate_step(building, optimize)
                num_steps += 1
            total_steps += num_steps
        avg_steps = total_steps / num_runs
        results[hexapods] = avg_steps
        # print_grid(building)
    print("Num Hexapods vs Number of Steps")
    print(results)
    return


if __name__ == '__main__':
    building = generate_building_grid(1)
    print("Original map:")
    print_grid(building)
    profile_performance([1, 3, 5, 10], 10, False)
