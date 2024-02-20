from enum import Enum
from typing import List

class SquareType(Enum):
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


def generate_grid() -> List[List[SquareType]]:
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
    change_squares(building, Orientation.HORIZONTAL, third, 0, third)
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

    print_grid(building)


if __name__ == '__main__':
    generate_grid()