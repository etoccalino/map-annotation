import cPickle as pickle
import numpy


class TaggedNode:

    def __init__(self, num_total_tags):
        self.q = numpy.ones(num_total_tags) / num_total_tags

    def update_prob(tag):
        pass


class TaggedMap:

    @staticmethod
    def save_map(map_object, filename):
        """
        This method uses pickle to save the state of the map to a binary file.
        This file can be used to open the map later on.

        :param map_object: This is the object that wants to be saved. Usually
            the object that represents the entire map.
        :type map_object: Any python object that pickle can serialize.
        :param filename: The name of the file to be used to save the map.
        :type filename: string
        """
        dump_file = open(filename, 'wp')
        # Protocol 2 uses the new, more efficient, binary format.
        pickle.dump(map_object, dump_file, 2)

    @staticmethod
    def load_map(filename):
        """
        This method loads a map from a saved file and returns a DynamicMap
        object.

        :param filename: The filename of the saved data.
        :type filename: string
        :returns: The object that was pickled in
            filename.
        :rtype: TaggedMap
        """
        save_file = open(filename)
        dynmap = pickle.load(save_file)
        return dynmap

    def __init__(self, grid_size, num_total_tags):
        self.num_total_tags = num_total_tags
        self._grid_map = {}

    def _pose_to_cell(self, coordinate):
        """
        Calculates the cell corresponding to a given point according to the map
        resolution.

        :param coordinates: (x, y, t) coordinates.
        :type coordinates: tuple
        :returns: A cell coordinate formed by (x, y, t) coordinates
        :rtype: tuple
        """
        dim1, dim2, dim3 = coordinate
        dim1 = int(dim1 / self._grid_size)
        dim2 = int(dim2 / self._grid_size)
        dim3 = int(dim3 / self._grid_size)
        cell_xyt = (dim1, dim2, dim3)
        return cell_xyt

    def update_pose(self, coordinate, value):
        """
        Updates any point in the space. The point coordinates
        will be approximated to the nearest cell, and the the corresponding
        cell will be updated.

        :param coordinate: (x, y, theta) coordinates.
        :type coordinate: tuple
        """
        cell_xyz = self._pose_to_cell(coordinate)
        self._update_cell(cell_xyz, value)

    def _update_cell(self, pose, value):
        # If the node does not exist, create it
        # before updating its value
        try:
            node = self._grid_map[pose]
        except KeyError:
            node = TaggedNode(self.num_total_tags)
            self._grid_map[pose] = node

        node.update_prob(value)
