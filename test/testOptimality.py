import unittest
import mdr
import numpy as np
import csv
import logging
import sys
import utils
import ast
from mdr import MDR

logger = logging.getLogger()
logger.level = logging.DEBUG
logger.addHandler(logging.StreamHandler(sys.stdout))

class TestOptimality(unittest.TestCase):


    def setUp(self):
        self.grid = utils.read_grid('Room1.txt')


    def test_not_optimal(self):
        main_route = []
        routes_file = "test//bug-1.csv"

        logging.info("Starting the test!")

        # reading csv file
        with open(routes_file, 'rt') as csvfile:
            # creating a csv reader object
            csvreader = csv.reader(csvfile)
            header = next(csvreader)
            # extracting each data row one by one
            for row in csvreader:
                tmp_route = []
                for y in range(0, len(row)):
                    if row[y]:
                        tmp_route.append(ast.literal_eval(row[y]))
                main_route.append(tmp_route)

            current_agent = 0
            temp_list_current = main_route[current_agent]
            temp_list_first = main_route[0]
            main_route[0] = temp_list_current
            main_route[current_agent] = temp_list_first

            mdr_solver = MDR()
            solution, route, total_expanded_nodes = mdr_solver.search_for_interrupt_plan(self.grid, main_route, 2)
            makespan_k_2 = MDR._compute_makespan(route)
            logging.info("k=%d, makespan = %d" % (2, makespan_k_2))

            solution, route, total_expanded_nodes = mdr_solver.search_for_interrupt_plan(self.grid, main_route, 3)
            makespan_k_3 = MDR._compute_makespan(route)
            logging.info("k=%d, makespan = %d" % (3, makespan_k_3))
            self.assertTrue(makespan_k_2<=makespan_k_3,"Old makespan (%d) is larger than new makespan (%d)"  % (makespan_k_2,makespan_k_3))




if __name__ == '__main__':
    unittest.main()