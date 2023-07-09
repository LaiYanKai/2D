#!/usr/bin/python3
import argparse
import pathlib
import numpy as np

THRES = 1e-8
class Scenario:
    id = 0  # starts from 1
    name = ""
    path = np.array([[]])
    nsec = np.nan
    cost = np.nan

    # path has to be numpy array of int, shape(nx2), where n is the number of turning points (incl. start and goal)
    def init(self, name, id, nsec, path):
        self.path = path
        self.nsec = nsec
        # print(id, ' -------')
        self.getPathCost()
        self.id = id
        self.name = name

    # from line of file
    def initFromLine(self, name, id, line):
        line = line.split()
        nsec = int(line[0])
        line.pop(0)
        if (len(line) == 0):
            path = np.array([[]])
            self.init(name, id, nsec, path)   
            self.cost = np.nan
        else:
            path = np.array([line], dtype=int)
            path = path.reshape(-1, 2)
            self.init(name, id, nsec, path)   

    # gets the differences between the coordinates, groups them into components and magnitudes, and calculate the costs based on the components and magnitudes
    # this method reduces floating point errors
    def getPathCost(self):
        # cost = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)) # the standard approach
        if self.path.size == 0:
            self.cost = np.nan    
        else:
            abs_diffs = np.abs(np.diff(self.path, axis=0))
            # remove all 0, 0 abs_diffs
            indices = np.sum(abs_diffs, axis=1) != 0
            abs_diffs = abs_diffs[indices, :]
            
            gcds = np.array([np.gcd.reduce(abs_diffs, axis=1)])
            gcds = np.repeat(gcds, 2, axis=0).T
            hcfs = np.divide(abs_diffs, gcds) # if this gives a warning, there may be a zero cost path or overlapping turning points
            unique_hcfs = np.unique(hcfs, axis=0) # unique rows 

            self.cost = 0
            for component in unique_hcfs:
                indices = np.where(np.all(hcfs==component, axis=1))
                magnitude = np.sum(gcds[indices, 0])
                self.cost += magnitude * np.linalg.norm(component)

        return self.cost

    def __repr__(self):
        path_str = '; '.join(','.join(f'{coord}' for coord in point) for point in self.path)
        return f"name( {self.name} )\tid( {self.id})\tnsec( {self.nsec} )\t$( {self.cost} )\tpoints( {np.shape(self.path)[0]} )\tpath( {path_str} )"


if __name__ == "__main__":
    pass