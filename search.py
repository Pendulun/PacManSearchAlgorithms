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

def nodeIsInQueue(myNode, myQueue: util.Queue):
    """
    Checks if any node of type {"position": (x,y),
                                "lastActionTakenToGetToNode": "action",
                                "parentNode": otherDict}
    
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
            frontierNode = {
                    "position": child[0],
                    "lastActionTakenToGetToNode": child[1],
                    "parentNode": node
                }
            #If the child has not been explored and is not already in the queue to be explored
            if child[0] not in explored and not nodeIsInQueue(child[0], frontier):
                frontier.push(frontierNode)

        #Mark node as explored as we checked all it's children      
        explored[node["position"]] = None

    #returns a empty list of actions if could not find a goal
    return []


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def greedySearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


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
