class Trajectory:
    index = 0  # the index of the corresponding pedestrian
    time = 0  # the time step
    x = 0  # the x-coordinate of trajectory
    y = 0  # the y-coordinate of trajectory
    point = []  # the point

    def __init__(self, index, time, x, y):
        self.index = index
        self.time = time
        self.x = x
        self.y = y
        self.point = [x, y]
