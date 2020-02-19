import numpy as np

class Map:
    def __init__(self, map_msg):
        # Store the dimensions of the map
        self.size_x = map_msg.info.width
        self.size_y = map_msg.info.height

        # Store the occupancy grid as a np array row-major notation
        self.occupancy_grid = np.array(map_msg.data, dtype = np.int8)
        self.occupancy_grid = np.reshape(self.occupancy_grid, newshape = (self.size_x, self.size_y))


        # Threshold above which occupancy grid cell is considered occupied
        OCCUPIED_THRESHOLD = 50

        # Create a flat occupancy grid
        # 1 - Occupied
        # 0 - Not Occupied
        # -1 - Unknown
        self.occupancy_grid_flat = np.zeros(shape = self.occupancy_grid.shape)
        self.occupancy_grid_flat[self.occupancy_grid > OCCUPIED_THRESHOLD] = 1
        self.occupancy_grid_flat[self.occupancy_grid == -1] = -1



        # Store the resolution
        self.resolution = map_msg.info.resolution

        # Stores the distances to the nearest occupied cell
        self.distances = np.zeros(shape = self.occupancy_grid.shape)
        # Stores whether the distance has been marked for a cell yet
        self.marked = np.zeros(shape = self.occupancy_grid.shape)

        # Queue for storing grid elements while finding distances to occupied cells
        self.queue = []

        # Maximum distance to a occupied node that we care about
        #TODO Update max distance threshold
        self.MAX_DISTANCE = 10


        print(self.size_x)
        print(self.size_y)
        self.calc_distances()

    def calc_distances(self):

        for i in range(self.occupancy_grid.shape[0]):
            for j in range(self.occupancy_grid.shape[1]):
                cell = {
                    "src_i" : i,
                    "src_j" : j,
                    "i" : i,
                    "j" : j
                }

                # Cell is occupied
                if(self.occupancy_grid_flat[i][j] == 1):
                    # Store the distance as 0 and note that distance has been marked
                    self.distances[i][j] = 0
                    self.marked[i][j] = 1

                    self.queue.append(cell)

                else:
                    self.distances[i][j] = self.MAX_DISTANCE

        while(len(self.queue) > 0):
            cur_cell = self.queue.pop(0)
            if(cur_cell["i"] > 0):
                self.enqueue({
                    "i" : cur_cell["i"] - 1,
                    "j" : cur_cell["j"],
                    "src_i" : cur_cell["src_i"],
                    "src_j" : cur_cell["src_j"]
                })
                # print("Adding cell ({} {})".format(cur_cell["i"] - 1, cur_cell["j"]))
            if(cur_cell["j"] > 0):
                self.enqueue({
                    "i" : cur_cell["i"],
                    "j" : cur_cell["j"] - 1,
                    "src_i" : cur_cell["src_i"],
                    "src_j" : cur_cell["src_j"]
                })
                # print("Adding cell ({} {})".format(cur_cell["i"], cur_cell["j"] - 1))
            if(cur_cell["i"] < self.size_x - 1):
                self.enqueue({
                    "i" : cur_cell["i"] + 1,
                    "j" : cur_cell["j"],
                    "src_i" : cur_cell["src_i"],
                    "src_j" : cur_cell["src_j"]
                })
                # print("Adding cell ({} {})".format(cur_cell["i"] + 1, cur_cell["j"]))
            if(cur_cell["j"] < self.size_y - 1):
                self.enqueue({
                    "i" : cur_cell["i"],
                    "j" : cur_cell["j"] + 1,
                    "src_i" : cur_cell["src_i"],
                    "src_j" : cur_cell["src_j"]
                })
                # print("Adding cell ({} {})".format(cur_cell["i"], cur_cell["j"] + 1))

    def enqueue(self, cell):
        if(self.marked[cell["i"]][cell["j"]] == 1):
            return

        distance_i = abs(cell["i"] - cell["src_i"])
        distance_j = abs(cell["j"] - cell["src_j"])



        # Record distance for cell
        self.distances[cell["i"]][cell["j"]] = distance_i + distance_j

        self.queue.append(cell)

        self.marked[cell["i"]][cell["j"]] = 1

    def get_distance(self, cell):
        distance_i = abs(cell["i"] - cell["src_i"])
        distance_j = abs(cell["j"] - cell["src_j"])

        return self.resolution*np.sqrt(np.power(distance_i, 2) + np.power(distance_j, 2))

    def on_map(self, i, j):
        if(i < 0 or i >= self.size_x or j < 0 or j >= self.size_y):
            return False

        return True