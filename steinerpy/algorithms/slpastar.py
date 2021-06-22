import steinerpy.config as cfg
from steinerpy.framework import Framework
from .common import Common
from steinerpy.library.logger import MyLogger 


class SLPAstarHS(Framework):
    """S* Merged with Heuristics (LPA*) """

    def __init__(self, G, T):
        Framework.__init__(self, G, T)
        for component in self.comps.values():
            start = component.start
            component.g[start] = float('inf')
            component.rhs[start] = 0
            component.currentP = (self.h_costs_func(start, component), 0)
            component.frontier.put(start, component.currentP)

    def g_costs_func(self, component, next):
        if next in component.g.keys():
            return component.g[next]
        return float('inf')

    def rhs_costs_func(self, component, next):
        """rhscost(n) = min(gcost(n') + cost(n, n')
        
        Parameters:
            component (GenericSearch): Generic Search class object (get access to all its variables)
            cost_so_far (dict): Contains all nodes with finite g-cost
            next (tuple): The node in the neighborhood of 'current' to be considered 
            neighbors (dict): The costs of next's neighbors 

        Returns:
            rhscost (float): The priority value for the node 'next'

        """
        if next in component.rhs.keys():
            return component.rhs[next]
        neighbors = component.graph.neighbors(next)
        rhsCost =  min(self.g_costs_func(component, x) + component.graph.cost(x, next) for x in neighbors)

        # Keep track of rhsCosts
        component.rhs[next] = rhsCost
        
        return rhsCost

    def h_costs_func(self, next, object_):
        """Heuristic costs for the node 'next', neighboring 'current'

        Parameters:
            next (tuple): The node in the neighborhood of 'current' to be considered 
            component (GenericSearch): Generic Search class object (get access to all its variables)

        Info:
            h_i(u) = min{h_j(u)}  for all j in Destination(i), and for some node 'u'
        
        """
        # If we don't have any goals...
        if not object_.goal:
            return 0

        # type_ = 'diagonal_nonuniform'
        type_ = cfg.Algorithm.sstar_heuristic_type
  
        # need to look at current object's destination...which changes
        # hju = list(map(lambda goal: htypes(type_, next, goal), terminals))
        # hju = list(map(lambda goal: htypes(type_, next, goal), [terminals[i] for i in comps[object_.id]['destinations']]))
        # hju = list(map(lambda goal: htypes(type_, next, goal), [dest for dest in object_.goal]))
        hju = list(map(lambda goal: Common.grid_based_heuristics(type_=type_, next=next, goal=goal), object_.goal.values()))
        
        minH = min(hju)
        minInd = hju.index(minH)
        minGoal = object_.goal[list(object_.goal)[minInd]]

        # Set current Goal
        object_.currentGoal = minGoal

        return minH

    def f_costs_func(self, component, cost_so_far, next):
        """fcost(n) = gcost(n) + hcost(n, goal)        
        
        Parameters:
            component (GenericSearch): Generic Search class object (get access to all its variables)
            cost_so_far (dict): Contains all nodes with finite g-cost
            next (tuple): The node in the neighborhood of 'current' to be considered 

        Returns:
            fcost (float): The priority value for the node 'next'

        """
        rhs = self.rhs_costs_func(component, next)
        fCost = min(cost_so_far[next], rhs) +  self.h_costs_func(next, component)

        # Keep track of Fcosts
        component.f[next] = fCost

        return fCost

    def calculateKey(self, node, component):
        rhs = self.rhs_costs_func(component, node)
        g = self.g_costs_func(component,node)
        return min(rhs, g) + self.h_costs_func(node, component), min(rhs, g)

    def updateVertex(self, node, component):
        if node != component.start:
            component.rhs[node] = self.rhs_costs_func(component, node)
        if node in component.frontier:
            component.frontier.delete(node)
        if self.g_costs_func(component, node) != self.rhs_costs_func(component, node):
            component.frontier.put(node, self.calculateKey(node, component))

    def update(self):
        """ 
        Compute Shortest Path from paper

        """
        MyLogger.add_message("performing update() ", __name__, "INFO")
        # CONSIDER USING TRY CATCH FOR THIS ENTIRE LOOP

        try:
            # TEST 
            #self.nodeQueue = PriorityQueue()
            # nodeQueue is empty!
            if self.nodeQueue.empty():
                print(self.terminals)
                raise Exception("nodeQueue is empty!")

            best_priority, best_ndx = self.nodeQueue.get_min()  
        except Exception as e:
            MyLogger.add_message("nodeQueue has an error", __name__, "ERROR", "exc_info=True")
            raise e
        
        # get best component object, and g cost of best node
        bestC = self.comps[best_ndx]
        bestCurrent = bestC.current
        bestGVal = bestC.g[bestCurrent]  
        bestPVal = bestC.currentP
        
        # Get parent (if possible), parent is a dict
        bestParent = bestC.parent.get(bestCurrent, None)

        # Store and return the selected node(s)
        self.selNode = bestCurrent
        self.selData = {t:{} for t in self.terminals} 
        self.selData.update({'to': bestParent, 'terminalInd': best_ndx, 'gcost': bestGVal, 'pcost':bestPVal, 'status': 'closed'})

        # find 'goal' node
        goal_priority = {}
        for t in bestC.goal.values():
            print(t, bestCurrent)
            key = self.calculateKey(t, bestC)
            goal_priority[key] = t
        bestGoalP = min(goal_priority.keys())
        bestGoal = goal_priority[bestGoalP]

        while bestC.frontier.get_min() < bestGoalP or bestC.rhs[bestGoal] != bestC.g[bestGoal]:
            # pop 
            self.nodeQueue.get()
            u = bestCurrent

            if bestC.g[u] > bestC.rhs[u]:
                bestC.g[u] = bestC.rhs[u]
            else:
                bestC.g[u] = float('inf')
                self.updateVertex(u, bestC)
            
            for s in bestC.graph.neighbors(u):
                self.updateVertex(s, bestC)

class SLPAstarHS0(Framework):
    pass

class SLPAstarBS(Framework):
    pass

class SLPAstarMM(Framework):
    """Meet-in-the-Middle implementation with heuristics """
    pass

class SLPAstarMM0(Framework):
    """Meet-in-the-Middle implementation without heuristics (Brute-force search)"""
    pass