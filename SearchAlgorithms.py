import operator


class Node:
    id = None  # Unique value for each node.
    up = None  # Represents value of neighbors (up, down, left, right).
    down = None
    left = None
    right = None
    previousNode = None  # Represents value of neighbors.
    edgeCost = None  # Represents the cost on the edge from any parent to this node.
    gOfN = None  # Represents the total edge cost
    hOfN = None  # Represents the heuristic value
    heuristicFn = None  # Represents the value of heuristic function

    def __init__(self, value):
        self.value = value

    def copy(self):
        copied = Node(self.value)
        copied.id = self.id
        copied.up = self.up
        copied.down = self.down
        copied.left = self.left
        copied.right = self.right
        copied.previousNode = self.previousNode
        copied.hOfN = self.hOfN
        return copied


class SearchAlgorithms:
    """ * DON'T change Class, Function or Parameters Names and Order
        * You can add ANY extra functions,
          classes you need as long as the main
          structure is left as is """
    path = []  # Represents the correct path from start node to the goal node.
    fullPath = []  # Represents all visited nodes from the start node to the goal node.
    totalCost = -1  # Represents the total cost in case using UCS, AStar (Euclidean or Manhattan)

    def __init__(self, mazeStr, heristicValue=None):
        """ mazeStr contains the full board
         The board is read row wise,
        the nodes are numbered 0-based starting
        the leftmost node"""
        self.maze = [row.split(',') for row in mazeStr.split()]
        self.hv = heristicValue
        self.nodes = self.init_nodes()

    def init_nodes(self):
        nodes = []
        k = 0
        for i in range(len(self.maze)):
            nodes.append([])
            for j in range(len(self.maze[0])):
                node = Node(self.maze[i][j])
                node.id = i * len(self.maze[0]) + j
                if self.hv is not None:
                    node.hOfN = self.hv[k]
                k = k + 1
                if i > 0:
                    node.up = nodes[i - 1][j]
                    nodes[i - 1][j].down = node
                if j > 0:
                    node.left = nodes[i][j - 1]
                    nodes[i][j - 1].right = node
                nodes[i].append(node)
        return nodes

    def get_unvisited(self, node):
        def is_available(some_node):
            return some_node is not None and \
                   some_node != node.previousNode and \
                   some_node.value != '#'

        unvisited = []
        if is_available(node.up):
            unvisited.append(node.up)
        if is_available(node.down):
            unvisited.append(node.down)
        if is_available(node.left):
            unvisited.append(node.left)
        if is_available(node.right):
            unvisited.append(node.right)


        return unvisited

    def DLS(self):
        # Fill the correct path in self.path
        # self.fullPath should contain the order of visited nodes
        self.DLS_recur(self.nodes[0][0], 50)
        return self.path, self.fullPath

    def DLS_recur(self, node, limit):
        SUCCESS = 1
        CUTOFF = 0
        FAILURE = -1
        self.fullPath.append(node.id)
        self.path.append(node.id)
        if node.value == 'E':
            return SUCCESS
        elif limit == 0:
            self.path.pop()
            return CUTOFF
        else:
            next_nodes = self.get_unvisited(node)
            reached_cutoff = False
            for next_node in next_nodes:
                if not self.path.__contains__(next_node.id):
                    next_node.previousNode = node
                    ret = self.DLS_recur(next_node, limit - 1)
                    if ret == SUCCESS:
                        return SUCCESS
                    if ret == CUTOFF:
                        reached_cutoff = True

            self.path.pop()
            if reached_cutoff:
                return CUTOFF
            else:
                return FAILURE

    def BDS(self):
        # Fill the correct path in self.path
        # self.fullPath should contain the order of visited nodes
        self.path.clear()
        self.fullPath.clear()
        end_node = [node for row in self.nodes for node in row if node.value == 'E'][0]
        self.bds(self.nodes[0][0], end_node)
        return self.path, self.fullPath

    def bds(self, source, target):
        from_start = [source]
        from_goal = [target]

        visited_start = {source.id}
        visited_goal = {target.id}
        intersection = source.id

        while len(from_goal) > 0 and len(from_start) > 0:
            x = from_start[0]
            self.fullPath.append(x.id)
            if (x.id == target.id) or (x.id in [node.id for node in from_goal]):
                intersection = x.id
                break
            from_start.pop(0)
            next_nodes = self.get_unvisited(x)
            for next_node in next_nodes:
                if next_node.id not in visited_start:
                    visited_start.add(next_node.id)
                    next_node.previousNode = x
                    from_start.append(next_node.copy())

            x = from_goal[0]
            self.fullPath.append(x.id)
            if (x.id == source.id) or (x.id in [node.id for node in from_start]):
                intersection = x.id
                break
            from_goal.pop(0)
            next_nodes = self.get_unvisited(x)
            for next_node in next_nodes:
                if next_node.id not in visited_goal:
                    visited_goal.add(next_node.id)
                    next_node.previousNode = x
                    from_goal.append(next_node.copy())

        # Reconstruct path from intersection node
        left = [node for node in from_start if node.id == intersection][0]
        right = [node for node in from_goal if node.id == intersection][0]
        while left.previousNode is not None:
            self.path.append(left.previousNode.id)
            left = left.previousNode
        self.path.reverse()
        while right is not None:
            self.path.append(right.id)
            right = right.previousNode

    def BFS(self):
        # Fill the correct path in self.path
        # self.fullPath should contain the order of visited nodes
        self.path.clear()
        self.fullPath.clear()
        self.bfs(self.nodes[0][0])
        return self.path, self.fullPath, self.totalCost

    def bfs(self, node):
        no_solution = -1
        closed_list = []
        open_list = [node]
        while len(open_list) > 0:
            if len(open_list) == 0:
                return no_solution
            open_list.sort(key=operator.attrgetter('hOfN'))
            n = open_list[0]
            self.fullPath.append(n.id)
            if n.value == 'E':
                closed_list.append(n)
                break
            next_nodes = self.get_unvisited(n)
            for next_node in next_nodes:
                open_list.append(next_node)
                next_node.previousNode = n
            open_list.pop(0)
            closed_list.append(n)

        self.totalCost = 0
        last_node = closed_list[-1]
        # Reconstruct path from goal node to previous nodes
        while last_node is not None:
            self.totalCost = self.totalCost + last_node.hOfN
            self.path.append(last_node.id)
            last_node = last_node.previousNode
        self.path.reverse()


def main():
    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.')
    path, fullPath = searchAlgo.DLS()
    print('*DFS*\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\n\n')

    #######################################################################################

    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.')
    path, fullPath = searchAlgo.BDS()
    print('*BFS*\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\n\n')
    #######################################################################################

    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.',
                                  [0, 15, 2, 100, 60, 35, 30, 3
                                      , 100, 2, 15, 60, 100, 30, 2
                                      , 100, 2, 2, 2, 40, 30, 2, 2
                                      , 100, 100, 3, 15, 30, 100, 2
                                      , 100, 0, 2, 100, 30])
    path, fullPath, TotalCost = searchAlgo.BFS()
    print('* UCS *\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\nTotal Cost: ' + str(
        TotalCost) + '\n\n')
    #######################################################################################


main()