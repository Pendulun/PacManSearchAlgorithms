# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from sys import path
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def getPathTo(node):
    """
    Get every Action necessary to go to the goal state (node) via it's parent nodes.
    Must have a root node (a node with parentNode==None or lastActionTakenToGetToNode==None)
    All nodes must be dicts of {"position": (x,y), "lastActionTakenToGetToNode": "action", "parentNode": otherDict}
    """
    pathToNode = []
    parentNode = node["parentNode"]
    #While it is not the root node
    while not parentNode == None:
        pathToNode.insert(0, node["lastActionTakenToGetToNode"])
        node = parentNode
        parentNode = node["parentNode"]
    
    return pathToNode

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    startState = problem.getStartState()

    #Frontier is a Stack that contains dicts of {"position": (x,y),
    #                                            "lastActionTakenToGetToNode": "action",
    #                                            "parentNode": otherDict}

    frontier = util.Stack()
    frontier.push({
        "position": startState,
        "lastActionTakenToGetToNode": None,
        "parentNode": None
    })

    #explored is a dict of fully explored nodes which keys represent the 2-uple position of a Node already explored and None as a Value just because.
    #A reached node may or may not be at the frontier. It depends if it was explorated 
    explored = {}

    while not frontier.isEmpty():
        node = frontier.pop()

        if problem.isGoalState(node["position"]):
            return getPathTo(node)

        #getSuccessors return a list of triples ((x,y),"action",cost)
        for child in problem.getSuccessors(node["position"]):
            if child[0] not in explored:
                frontierNode = {
                    "position": child[0],
                    "lastActionTakenToGetToNode": child[1],
                    "parentNode": node
                }
                frontier.push(frontierNode)

        #Mark node as explored as we checked all it's children      
        explored[node["position"]] = None

    #returns a empty list of actions if could not find a goal
    return []

def nodeIsInQueue(myNode, myQueue):
    """
    Checks if any node, that is a dict with a 'position' key, 
    in the Queue has a node["position"] == myNode
    """
    for node in myQueue.list:
        if node["position"] == myNode:
            return True
    
    return False

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    startState = problem.getStartState()

    #Frontier is a Queue that contains dicts of {"position": (x,y),
    #                                            "lastActionTakenToGetToNode": "action",
    #                                            "parentNode": otherDict}
    #It represents nodes yet to be explored
    
    frontier = util.Queue()
    frontier.push({
        "position": startState,
        "lastActionTakenToGetToNode": None,
        "parentNode": None
    })

    #explored is a dict of fully explored nodes which Keys represent the 2-uple position of a Node already 
    # explored and None as a Value just because.
    explored = {}

    while not frontier.isEmpty():
        node = frontier.pop()

        #Late goal test
        if problem.isGoalState(node["position"]):
            return getPathTo(node)

        #getSuccessors return a list of triples ((x,y),"action",cost)
        for child in problem.getSuccessors(node["position"]):
            #If the child has not been explored and is not already in the queue to be explored
            if child[0] not in explored and not nodeIsInQueue(child[0], frontier):
                frontierNode = {
                    "position": child[0],
                    "lastActionTakenToGetToNode": child[1],
                    "parentNode": node
                }
                frontier.push(frontierNode)

        #Mark node as explored as we checked all it's children      
        explored[node["position"]] = None

    #returns a empty list of actions if could not find a goal
    return []


def getPathToDict(goalNode, nodesInfo):
    """
    Get every Action necessary to go to the goalNode via it's parent nodes.
    All nodes must be dicts of {"cost": 0, "parent":None, "lastActionTakenToGetToNode": None, "explored":False}
    Must exist a root node (a node with parentNode==None or lastActionTakenToGetToNode==None)
    and nodesInfo must be a dict of dicts in the form:
    {(x,y): {"cost": z, "parent":(x2, y2), "lastActionTakenToGetToNode": "action", "explored":True/False}}
    The parent of a node must also be in nodesInfo
    """
    thisNode = goalNode
    parentNode = nodesInfo[thisNode]['parent']
    pathToNode = []
    while not parentNode == None:
        pathToNode.insert(0, nodesInfo[thisNode]['lastActionTakenToGetToNode'])
        thisNode = parentNode
        parentNode = nodesInfo[thisNode]['parent']
    
    return pathToNode


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    startState = problem.getStartState()

    #PriorityQueue with (x,y) as keys and path cost to node as priority
    frontierQueue = util.PriorityQueue()
    frontierQueue.push(startState, 0)

    #nodesInfo is a dict of dicts for information of nodes:
    #{(x,y): {"cost": z, "parent":(x2, y2), "lastActionTakenToGetToNode": "action", "explored":True/False},}
    #Represents information about nodes. If a node is in it, it was reached ("explored": False) or fully explored ("explored":True)
    nodesInfo = {}
    nodesInfo[startState] = {"cost": 0, "parent":None, "lastActionTakenToGetToNode": None, "explored":False}

    while not frontierQueue.isEmpty():
        currentNode = frontierQueue.pop()
        nodeCost = nodesInfo[currentNode]["cost"]

        #Late goal test
        if problem.isGoalState(currentNode):
            return getPathToDict(currentNode, nodesInfo)

        #getSuccessors returns a list of triples ((x,y),"action",cost)
        for child in problem.getSuccessors(currentNode):
            totalCostPathToChild = nodeCost+child[2]

            #If the child has not been explored and is not already in the queue to be explored
            #this is, have not been reached
            if child[0] not in nodesInfo:
                thisChildInfo = {"cost": totalCostPathToChild,
                            "parent":currentNode, 
                            "lastActionTakenToGetToNode": child[1],
                            "explored": False
                }
                frontierQueue.push(child[0], totalCostPathToChild)
                nodesInfo[child[0]] = thisChildInfo

            #Child was already reached and it's cost is higher than the one from this parent node
            elif not nodesInfo[child[0]]['explored'] and nodesInfo[child[0]]['cost'] > totalCostPathToChild:
                nodesInfo[child[0]]['cost'] = totalCostPathToChild
                nodesInfo[child[0]]['parent'] = currentNode
                nodesInfo[child[0]]['lastActionTakenToGetToNode'] = child[1]
                frontierQueue.update(child[0], totalCostPathToChild)

        #Mark node as explored as we checked all it's children      
        nodesInfo[currentNode]['explored'] = True

    #returns a empty list of actions if could not find a goal
    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def greedySearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest heuristic first."""
    startState = problem.getStartState()

    #PriorityQueue with (x,y) as keys and path cost to node as priority
    frontierQueue = util.PriorityQueue()
    frontierQueue.push(startState, 0)
    #nodesInfo is a dict of dicts for information of nodes:
    #{(x,y): {"cost": z, "parent":(x2, y2), "lastActionTakenToGetToNode": "action", "explored":True/False},}
    #Represents information about nodes. If a node is in it, it was reached ("explored": False) or fully explored ("explored":True)
    nodesInfo = {}
    nodesInfo[startState] = {"parent":None, "lastActionTakenToGetToNode": None, "explored":False}

    while not frontierQueue.isEmpty():
        currentNode = frontierQueue.pop()

        #Late goal test
        if problem.isGoalState(currentNode):
            return getPathToDict(currentNode, nodesInfo)

        #getSuccessors returns a list of triples ((x,y),"action",cost)
        for child in problem.getSuccessors(currentNode):

            #If the child has not been explored and is not already in the queue to be explored
            #this is, have not been reached
            if child[0] not in nodesInfo:
                thisChildInfo = {"parent":currentNode, 
                                "lastActionTakenToGetToNode": child[1],
                                "explored": False
                }
                frontierQueue.push(child[0], heuristic(child[0], problem))
                nodesInfo[child[0]] = thisChildInfo

        #Mark node as explored as we checked all it's children      
        nodesInfo[currentNode]['explored'] = True

    #returns a empty list of actions if could not find a goal
    return []


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    startState = problem.getStartState()

    #PriorityQueue with (x,y) as keys and path cost to node as priority
    frontierQueue = util.PriorityQueue()
    frontierQueue.push(startState, 0)

    #nodesInfo is a dict of dicts for information of nodes:
    #{(x,y): {"cost": z, "parent":(x2, y2), "lastActionTakenToGetToNode": "action", "explored":True/False},}
    #Represents information about nodes. If a node is in it, it was reached ("explored": False) or fully explored ("explored":True)
    nodesInfo = {}
    nodesInfo[startState] = {"pathCost": 0, "parent":None, "lastActionTakenToGetToNode": None, "explored":False}

    while not frontierQueue.isEmpty():
        currentNode = frontierQueue.pop()
        nodeCost = nodesInfo[currentNode]["pathCost"]

        #Late goal test
        if problem.isGoalState(currentNode):
            return getPathToDict(currentNode, nodesInfo)

        #getSuccessors returns a list of triples ((x,y),"action",cost)
        for child in problem.getSuccessors(currentNode):
            #If node was not explored
            if child[0] not in nodesInfo or (child[0] in nodesInfo and not nodesInfo[child[0]]['explored']):
                heuristicCost = heuristic(child[0], problem)
                #if node was not reached
                if child[0] not in nodesInfo:
                    totalPathCostToChild = nodeCost+child[2]
                    costWithHeuristic = totalPathCostToChild + heuristicCost
                    nodeInfo = {"pathCost": totalPathCostToChild, 
                                    "parent": currentNode,
                                    "lastActionTakenToGetToNode": child[1],
                                    "explored": False}
                    nodesInfo[child[0]] = nodeInfo
                    frontierQueue.push(child[0], costWithHeuristic)
                elif nodeCost + child[2] < nodesInfo[child[0]]['pathCost']:
                    nodesInfo[child[0]]['pathCost'] = nodeCost + child[2]
                    nodesInfo[child[0]]['parent'] = currentNode
                    nodesInfo[child[0]]["lastActionTakenToGetToNode"] = child[1]
                    frontierQueue.update(child[0], nodesInfo[child[0]]['pathCost'] + heuristicCost)

        #Mark node as explored as we checked all it's children      
        nodesInfo[currentNode]['explored'] = True

    #returns a empty list of actions if could not find a goal
    return []


def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    position, foodGrid = state
    "*** YOUR CODE HERE ***"
    return 0


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
ucs = uniformCostSearch
gs = greedySearch
astar = aStarSearch
