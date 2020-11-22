#!/usr/bin/env python3

from robot_sim.bresenham import bresenham
import numpy as np


MOVE_IDS = {'N': 0, 'S': 1, 'W': 2, 'E': 3}

MOVE = [(0, 1),     # Up
        (0, -1),    # Down
        (-1, 0),    # Left
        (1, 0)]     # Right


class RobotSimulator2D:
    """
    A class that defines a simple environment for a robot including its interaction with it (sensing/moving).
    """

    def __init__(self, world: np.ndarray = None, pos: list = None, size_x: int = None, size_y: int = None, num_objects: int = None, num_sensors: int = 4):
        """
        Initializes the class RobotEnvironment.

        @param size_x: The size of the world in x direction.
        @param size_y: The size of the world in y direction.
        @param num_objects: The number of objects.
        @param num_sensors: The number of sensors (4 or 8), defaults to 4.
        """
        if world is None:
            self.world = np.zeros(size_x, size_y)
            self.size_x = size_x
            self.size_y = size_y

            self.init_world(num_objects)
        else:
            self.world = world
            self.size_x = world.shape[0]
            self.size_y = world.shape[1]

        if pos is None:
            self.pos = np.array([0, 0])
            self.init_position()
        else:
            self.pos = np.array(pos)

        if not num_sensors in [4, 8]:
            self.num_sensors = 4
        self.num_sensors = num_sensors


    def init_world(self, num_objects: int):
        """
        Initializes the world with objects.

        @param size_x: The size of the world in x direction.
        @param size_y: The size of the world in y direction.
        @param num_objects: The number of objects.
        """
        for i in range(num_objects):
            obj_x = np.random.randint(self.size_x)
            obj_y = np.random.randint(self.size_y)

            while self.world[obj_x, obj_y] == 1:
                obj_x = np.random.randint(self.size_x)
                obj_y = np.random.randint(self.size_y)

            self.world[obj_x, obj_y] = 1

    def init_position(self, position: tuple = None) -> bool:
        """
        Initializes the position to the one given by the parameter, if provided and valid (free). In case the position
        parameter is None, a random position will be generated within the free space of the grid-world.

        @param position: The initial position of the robot (x, y).
        @return: True if cell at given position is valid (free), False otherwise (occupied).
        """

        if position is None:
            self.pos = np.array([np.random.randint(self.size_x), np.random.randint(self.size_y)])
            while self.world[tuple(self.pos)] > 0:
                self.pos = np.array([np.random.randint(self.size_x), np.random.randint(self.size_y)])
        else:
            if not self.cell_valid(*position):
                return False
            else:
                self.pos = np.array(position)

        return True

    def sense(self) -> list:
        """
        Measures the ranges around the robot with the predifined numbers of sensors. The output distances are measured
        clockwise, starting from South.

        @return: A list of float values containing the measured distances.
        """
        beams = [[self.pos[0], 0],              # S
                 [0, self.pos[1]],              # W
                 [self.pos[0], self.size_y-1],  # N
                 [self.size_x-1, self.pos[1]]]  # E

        if self.num_sensors == 8:
            beams_a = min(self.size_x - self.pos[0], self.size_y - self.pos[1])
            beams_b = min(self.size_x - self.pos[0], self.pos[1])
            beams_c = min(self.pos[0], self.pos[1])
            beams_d = min(self.pos[0], self.size_y - self.pos[1])
            beams = [beams[0],                                              # S
                     [self.pos[0] - beams_c, self.pos[1] - beams_c],        # SW
                     beams[1],                                              # W
                     [self.pos[0] - beams_d, self.pos[1] + beams_d],        # NW
                     beams[2],                                              # N
                     [self.pos[0] + beams_a, self.pos[1] + beams_a],        # NE
                     beams[3],                                              # E
                     [self.pos[0] + beams_b, self.pos[1] - beams_b]]        # SE

        measured_distances = []

        for beam_x, beam_y in beams:
            hit_cells = bresenham(self.pos[0], self.pos[1], beam_x, beam_y)

            measured_distance = -1
            cell_hit = False

            for cell in hit_cells:
                measured_distance += 1
                if self.world[cell] == 1:
                    measured_distances.append(measured_distance)
                    cell_hit = True
                    break

            if not cell_hit:
                measured_distances.append(measured_distance+1)

        return measured_distances

    def move(self, move_dir: str = None) -> bool:
        """
        Moves the robot either in the defined direction or in a random, free direction, in case move_dir is none.

        @param move_dir: The direction in which the robot is supposed to move (N, S, E, W)
        @return: True if the move was successful, False otherwise.
        """
        if move_dir is None:
            # Generate random movement while generated move is not valid
            move_id = np.random.randint(len(MOVE))

            while not self.cell_in_world(self.pos[0] + MOVE[move_id][0], self.pos[1] + MOVE[move_id][0]) \
                    or self.world[self.pos[0] + MOVE[move_id][0], self.pos[1] + MOVE[move_id][0]] > 0:
                move_id = np.random.randint(len(MOVE))
        
        else:
            move_id = MOVE_IDS[move_dir.upper()]

            if not self.cell_in_world(self.pos[0] + MOVE[move_id][0], self.pos[1] + MOVE[move_id][1]) \
                    or self.world[self.pos[0] + MOVE[move_id][0], self.pos[1] + MOVE[move_id][1]] > 0:
                return False

        self.pos[0] += MOVE[move_id][0]
        self.pos[1] += MOVE[move_id][1]

        return True

    def cell_in_world(self, x, y) -> bool:
        """
        Checks whether the cell with the given coordinates is located within the boundaries of the world.

        @param x: The x-coordinate of the cell.
        @param y: The y-coordinate of the cell.
        @return: True, if the cell lies within the world, False otherwise.
        """
        return 0 <= x < self.size_x and 0 <= y < self.size_y

    def cell_valid(self, x, y) -> bool:
        """
        Checks whether the cell with the given coordinates is located within the boundaries of the world and is not
        occupied (free).

        @param x: The x-coordinate of the cell.
        @param y: The y-coordinate of the cell.
        @return: True, if the cell lies within the world and is free, False otherwise.
        """
        return self.cell_in_world(x, y) and self.world[x, y] == 0