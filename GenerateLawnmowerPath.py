import matplotlib.pyplot as plt

def generate_lawnmower_path(xTopLeft, yTopLeft, xBottomRight, yBottomRight, L, ds=2):
    '''
    Parameters
    ----------
    xTopLeft : float
        X coordinate of Top Left (or bottom left) point of bounding box.
    yTopLeft : float
        Y coordinate of Top Left (or bottom left) point of bounding box.
    xBottomRight : float
        X coordinate of Bottom Right (or top right) point of bounding box.
    yBottomRight : float
        X coordinate of Bottom Right (or top right) point of bounding box.
    L : float
        Distance between zig zags. Input the width of the robot or something.
    ds : float, optional
        The distance between points. The default is 2 units.

    Returns
    -------
    points : List of tuples, where each row/tuple is (x,y) of that point along path
        DESCRIPTION.

    '''
    points = []
    x_current = xTopLeft
    y_current = yTopLeft

    width = xBottomRight - xTopLeft
    height = yBottomRight - yTopLeft

    # Ensure that height is positive, assuming the direction is downward
    if height < 0:
        height = -height
        y_direction = -1
    else:
        y_direction = 1

    direction = 1  # 1 for right, -1 for left

    while (y_direction == 1 and y_current < yBottomRight) or (y_direction == -1 and y_current > yBottomRight):
        # Horizontal movement
        if direction == 1:
            while x_current < xBottomRight:
                points.append((x_current, y_current))
                x_current += ds
                if x_current >= xBottomRight:
                    x_current = xBottomRight
                    points.append((x_current, y_current))
                    break
        else:
            while x_current > xTopLeft:
                points.append((x_current, y_current))
                x_current -= ds
                if x_current <= xTopLeft:
                    x_current = xTopLeft
                    points.append((x_current, y_current))
                    break

        # Vertical movement
        y_next = y_current + y_direction * L
        if y_direction == 1 and y_next >= yBottomRight:
            y_next = yBottomRight
        elif y_direction == -1 and y_next <= yBottomRight:
            y_next = yBottomRight
        while (y_direction == 1 and y_current < y_next) or (y_direction == -1 and y_current > y_next):
            y_current += y_direction * ds
            if (y_direction == 1 and y_current > y_next) or (y_direction == -1 and y_current < y_next):
                y_current = y_next
            points.append((x_current, y_current))

        direction *= -1

    return points

# Example usage:
xTopLeft = 0
yTopLeft = 0
xBottomRight = 100
yBottomRight = 200
L = 20
ds = 2

lawnmower_points = generate_lawnmower_path(xTopLeft, yTopLeft, xBottomRight, yBottomRight, L, ds)

# Plotting the points with matplotlib
plt.figure(figsize=(8, 8))
for idx, (x, y) in enumerate(lawnmower_points):
    plt.plot(x, y, 'bo')  # Plot the point
    plt.text(x, y, str(idx), fontsize=9, ha='right')  # Label the point with its index

# Set plot limits
plt.xlim(xTopLeft - 10, xBottomRight + 10)
plt.ylim(min(yTopLeft, yBottomRight) - 10, max(yTopLeft, yBottomRight) + 10)

# Set labels and title
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Discretized Lawnmower Path with Point Indices')
plt.grid(True)
plt.show()
