# MAPF-Collision

Use MAPF Setup.py in order to:
  1. Load the map_data.txt file.
  2. Enter the no. of agents.
  3. Find a collision free paths for all agents using A*.
  4. Write the paths into Results.xls file.
  
Next we will build a'Collision maximier' to do the following:
  1. Load the results.xls file into a list.
  2. Get the 'Maximun error steps' by input.
  3. For X in agents using A*:
      Given the 'Maximum error steps' Find the best new path (Start and Goal will remain as is) so agent (X) will cause the maximum sigma-g for all agents.
  4. Choose the agent that cause the maximum sigma-g and show run-time.
  
We will build the 'def heuristic(a, b):' to use:
  1. manhattan distance
  2. octile distance
  3. Enhanced Partial Expansion A* - Operator Selection Function (OSF) (The Challenge: to build an effective OSF)
      Advantages
                  Saves memory
                  Does not generate surlplus nodes
                  Generates every node only once

      Disadvantages
                  OSF and EPEA* are domain dependent
                  Sometimes not applicable
                  OSF incurs extra time/memory overhead

