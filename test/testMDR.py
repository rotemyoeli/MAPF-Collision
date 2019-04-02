import unittest
import mdr
import numpy as np
import logging
import sys

logger = logging.getLogger()
logger.level = logging.DEBUG
logger.addHandler(logging.StreamHandler(sys.stdout))

class TestMDR(unittest.TestCase):


    def setUp(self):
        grid1=None
        with open('Room1.txt', 'rt') as infile:
            grid1 = np.array([list(line.strip()) for line in infile.readlines()])

        grid1[grid1 == '@'] = 1  # object on the map
        grid1[grid1 == 'T'] = 1  # object on the map
        grid1[grid1 == '.'] = 0  # free on map

        self.grid = np.array(grid1.astype(np.int))


    def test_astar_simple(self):
        # load the map file into numpy array


        start = ((16, 16),1)
        goal = (16, 21)

        result =  mdr.astar(self.grid, start, goal, 0, [],0)
        logging.info(result)
        self.assertEqual(6,len(result))

    def test_astar_hard(self):
        # load the map file into numpy array
        start = ((17, 16), 1)
        goal = (36, 16)
        result = mdr.astar(self.grid, start, goal, 0, [], 0)


if __name__ == '__main__':
    unittest.main()