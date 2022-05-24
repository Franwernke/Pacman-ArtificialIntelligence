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
    return  [s, s, w, s, w, w, s, w]

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
    visited = []
    solution = []
    solution = recursiveDfs((problem.getStartState(), ''), visited, problem)
    solution.reverse()
    return solution[1:]
    

def recursiveDfs(curState, visited, problem):
    if curState[0] in visited:
        return []
    visited.append(curState[0])
    if problem.isGoalState(curState[0]):
        return [curState[1]]
    for state in problem.getSuccessors(curState[0]):
        solution = recursiveDfs(state, visited, problem)
        if len(solution) > 0:
            solution.append(curState[1])
            return solution
    return []


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    if problem.isGoalState(problem.getStartState()):
        return []
    
    expanded = [problem.getStartState()]

    fifo = []
    fathers = {}
    for state in problem.getSuccessors(problem.getStartState()):
        fifo.append(state)
        fathers[str(state[0])] = (None, state[1])
        expanded.append(state[0])

    while len(fifo) > 0:
        curState = fifo[0]
        if problem.isGoalState(curState[0]):
            solution = recoverSolution(fathers, curState)
            solution.reverse()
            return solution
        fifo = fifo[1:]
        for state in problem.getSuccessors(curState[0]):
            if state[0] in expanded:
                continue
            if not (str(state[0]) in fathers):
                fathers[str(state[0])] = (curState[0], state[1])
            fifo.append(state)
            expanded.append(state[0])

def recoverSolution(fathers, state):
    father = fathers[str(state[0])]
    solution = [father[1]]
    while father[0] != None:
        father = fathers[str(father[0])]
        solution.append(father[1])
    return solution

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** ADD YOUR CODE HERE ***"
    if problem.isGoalState(problem.getStartState()):
        return []

    border = util.PriorityQueue()
    fathers = {}
    costs = {}
    costs[str(problem.getStartState())] = 0

    for edge in problem.getSuccessors(problem.getStartState()):
        fathers[str(edge[0])] = (None, edge[1])
        costs[str(edge[0])] = edge[2]
        border.push(edge, edge[2])

    while not border.isEmpty():
        curState = border.pop()
        if problem.isGoalState(curState[0]):
            solution = recoverSolution(fathers, curState)
            solution.reverse()
            return solution
        for edge in problem.getSuccessors(curState[0]):
            cost = calculateCost(edge, costs, curState[0])
            if str(edge[0]) in costs and cost >= costs[str(edge[0])]:
                continue
            costs[str(edge[0])] = cost
            fathers[str(edge[0])] = (curState[0], edge[1])
            border.push(edge, cost)

def calculateCost(state, costs, father):
    return costs[str(father)] + state[2]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** ADD YOUR CODE HERE ***"
    if problem.isGoalState(problem.getStartState()):
        return []

    border = util.PriorityQueue()
    fathers = {}
    costs = {}
    costs[str(problem.getStartState())] = 0

    for state in problem.getSuccessors(problem.getStartState()):
        fathers[str(state[0])] = (None, state[1])
        costs[str(state[0])] = state[2]
        border.push(state, state[2] + heuristic(state[0], problem))

    while not border.isEmpty():
        curState = border.pop()
        if problem.isGoalState(curState[0]):
            solution = recoverSolution(fathers, curState)
            solution.reverse()
            return solution
        for state in problem.getSuccessors(curState[0]):
            [totalCost, g] = calculateCostWithHeuristic(state, costs, curState[0], problem, heuristic)
            if str(state[0]) in costs and totalCost >= costs[str(state[0])]:
                continue
            costs[str(state[0])] = g
            fathers[str(state[0])] = (curState[0], state[1])
            border.push(state, totalCost)
    return []

def calculateCostWithHeuristic(state, costs, father, problem, heuristic):
    return [costs[str(father)] + state[2] + heuristic(state[0], problem), costs[str(father)] + state[2]]

def learningRealTimeAStar(problem, heuristic=nullHeuristic):
    """Execute a number of trials of LRTA* and return the best plan found."""
    "*** ADD YOUR CODE HERE ***"
    util.raiseNotDefined()

    # MAXTRIALS = ...
    

# Abbreviations 
# *** DO NOT CHANGE THESE ***
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
lrta = learningRealTimeAStar
