# Copy this into visualizer for debugging!

def print_grid(self):
    # Print the current grid formatted in a readable way
    for col in range(len(self.self.grid[0]) + 2):
        self.get_logger().info(SquareType.BLOCKED.value, end=" ")
    self.get_logger().info()
    for row in range(len(self.grid)):
        self.get_logger().info(SquareType.BLOCKED.value, end=" ")
        for col in range(len(self.grid[0])):
            self.get_logger().info(self.grid[row][col].value, end=" ")
        self.get_logger().info(SquareType.BLOCKED.value)
    for col in range(len(self.grid[0]) + 2):
        self.get_logger().info(SquareType.BLOCKED.value, end=" ")
    self.get_logger().info()
