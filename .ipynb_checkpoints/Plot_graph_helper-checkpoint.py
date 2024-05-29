import matplotlib.pyplot as plt
import numpy as np

def plot_things(path, origin, robotPosition, robotDirection, rWidth):
    """
    Plot several things using Matplotlib:
    1. List of tuples (path) as individual points.
    2. Tuple (origin) as a red 'x'.
    3. Arrow with tail at robotPosition and length rWidth in the direction of robotDirection.

    Parameters:
        path (list of tuples or numpy arrays): List of tuples (x, y) or numpy arrays to be plotted as individual points.
        origin (tuple or numpy array): Tuple (x, y) or numpy array to be plotted as a red 'x'.
        robotPosition (tuple or numpy array): Tuple (x, y) or numpy array specifying the position of the robot.
        robotDirection (float): Direction of the robot in degrees.
        rWidth (float): Length of the arrow representing the robot's direction.
    """
    # Convert input tuples to numpy arrays
    if isinstance(path, list):
        path = np.array(path)
    if isinstance(origin, tuple):
        origin = np.array(origin)
    if isinstance(robotPosition, tuple):
        robotPosition = np.array(robotPosition)
    
    # Extract x and y coordinates from path
    x_path, y_path = path[:, 0], path[:, 1]
    
    # Plot the path as individual points
    plt.plot(x_path, y_path, 'bo', label='Path')
    
    # Plot the origin as a red 'x'
    plt.plot(origin[0], origin[1], 'rx', label='Origin')
    
    # Calculate the end point of the arrow
    x_end = robotPosition[0] + rWidth * np.cos(np.radians(robotDirection))
    y_end = robotPosition[1] + rWidth * np.sin(np.radians(robotDirection))
    
    # Plot the arrow with smaller head
    plt.arrow(robotPosition[0], robotPosition[1], x_end - robotPosition[0], y_end - robotPosition[1],
              head_width=2, head_length=3, fc='g', ec='g', label='Robot')
    plt.plot(robotPosition[0], robotPosition[1], 'go')

    # Set plot limits
    plt.xlim(min(x_path.min(), origin[0], robotPosition[0]), max(x_path.max(), origin[0], robotPosition[0]))
    plt.ylim(max(y_path.max(), origin[1], robotPosition[1]), min(y_path.min(), origin[1], robotPosition[1]))  # Inverted y-axis
    
    # Set plot title and labels
    plt.title('Plot of Path, Origin, and Robot')
    plt.xlabel('X')
    plt.ylabel('Y')
    
    # Add legend
    plt.legend()
    
    # Invert y-axis
    plt.gca().invert_yaxis()
    
    # Show the plot
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

# Example usage:
path = [(10, 20), (30, 40), (50, 60)]  # Example path
origin = (5, 5)  # Example origin
robotPosition = (25, 30)  # Example robot position
robotDirection = 45  # Example robot direction (in degrees)
rWidth = 20  # Example arrow length

# Call the function to plot things
plot_things(path, origin, robotPosition, robotDirection, rWidth)
