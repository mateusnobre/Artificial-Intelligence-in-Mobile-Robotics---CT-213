from grid import Node, NodeGrid
from math import inf
import heapq as heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Dijkstra algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        pq = []
        start = self.node_grid.grid[start_position[0],start_position[1]]
        start.f = 0
        goal = self.node_grid.grid[goal_position[0],goal_position[1]]
        heapq.heappush (pq, (start.f,start))
        while not (len(pq) == 0):
            f, node = heapq.heappop(pq)
            node.closed = True
            if (node.i == goal_position[0]) and (node.j == goal_position[1]):
                goal.f = goal.parent.f + self.cost_map.get_edge_cost(goal, goal.parent)
                break
            for sucessor in self.node_grid.get_successors(node.i, node.j):
                sucessor = self.node_grid.grid[sucessor]
                if (sucessor.f > node.f + self.cost_map.get_edge_cost(node, sucessor)):
                    sucessor.f = node.f + self.cost_map.get_edge_cost(node,sucessor)
                    sucessor.parent = node
                    heapq.heappush(pq, (sucessor.f, sucessor))
        path = self.construct_path(goal)
        cost = goal.f
        self.node_grid.reset()
        return path, cost

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Greedy Search algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        pq = []
        start = self.node_grid.grid[start_position[0],start_position[1]]
        start.f = Node.distance_to(start, goal_position[0], goal_position[1])
        start.g = 0
        goal = self.node_grid.grid[goal_position[0],goal_position[1]]
        heapq.heappush (pq, (start.f,start))
        while not (len(pq) == 0):
            f, node = heapq.heappop(pq)      
            node.closed = True     
            if (node.i == goal_position[0]) and (node.j == goal_position[1]):
                goal.f = goal.parent.f + self.cost_map.get_edge_cost(goal, goal.parent)
                break 
            for sucessor in self.node_grid.get_successors(node.i, node.j):
                sucessor = self.node_grid.grid[sucessor] 
                if not (sucessor.closed == True):
                    sucessor.parent = node
                    sucessor.f = Node.distance_to(goal, sucessor.i, sucessor.j)
                    sucessor.g = sucessor.parent.g + self.cost_map.get_edge_cost(sucessor,sucessor.parent)
                    heapq.heappush(pq, (sucessor.f, sucessor))
                    self.node_grid.get_node(sucessor.i, sucessor.j).closed = True
        path = self.construct_path(goal)
        cost = goal.g
        self.node_grid.reset()
        return path, cost


    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        pq = []
        goal = self.node_grid.grid[goal_position[0],goal_position[1]]
        start = self.node_grid.grid[start_position[0],start_position[1]]
        start.g = 0
        start.f = Node.distance_to(start, goal_position[0],goal_position[1])
        heapq.heappush (pq, (start.f,start))
        while not (len(pq) == 0):
            f, node = heapq.heappop(pq)
            node.closed = True
            if (node.i == goal_position[0]) and (node.j == goal_position[1]):
                goal.g = goal.parent.g + self.cost_map.get_edge_cost(goal, goal.parent)
                goal.f = goal.g + 0
                break
            for sucessor in self.node_grid.get_successors(node.i, node.j):
                sucessor = self.node_grid.grid[sucessor]
                if (sucessor.f > node.g + self.cost_map.get_edge_cost(node, sucessor) + Node.distance_to(sucessor,goal_position[0],goal_position[1])):
                    sucessor.g = node.g + self.cost_map.get_edge_cost(node,sucessor)
                    sucessor.f = sucessor.g + Node.distance_to(sucessor,goal_position[0],goal_position[1])
                    sucessor.parent = node
                    heapq.heappush(pq, (sucessor.f, sucessor))
        path = self.construct_path(goal)
        cost = goal.f
        self.node_grid.reset()
        return path, cost