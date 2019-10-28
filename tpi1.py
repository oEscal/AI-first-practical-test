# Pedro Escaleira, 88821

from tree_search import *
import math

class MySearchNode(SearchNode):
    def __init__(self, state, parent):
        self.depth = 0
        self.cost = 0
        self.evalfunc = 0
        self.children = None
        
        super(MySearchNode, self).__init__(state, parent)

    def convert_super_to_sub(node):
        node.__dict__ = MySearchNode(**node.__dict__).__dict__
        node.__class__ = MySearchNode


class MyTree(SearchTree):

    def __init__(self, problem, strategy='breadth', max_nodes=None): 
        self.solution_cost = 0
        self.solution_length = 0
        self.total_nodes = 1                                            # equals 1 because it already has root node
        self.non_terminal_nodes = 1                                     # root node is a non terminal node, except when the tree have just one node
        self.terminal_nodes = 0
        self.max_nodes = max_nodes or math.inf                          # if max_nodes == None -> self.max_nodes = inf.; if max_nodes == <number> -> self.max_nodes = <number>

        super(MyTree, self).__init__(problem, strategy)
        MySearchNode.convert_super_to_sub(self.root)                    # transform root of class SearchNode in object of class MySearchNode, in which the parameters which are already initialized are mantained
    
    # function to check the effective branching factor based on the definition
    def error_ebf(self, B, d):
        return (pow(B, d + 1) - 1)/(B - 1) - self.total_nodes

    def astar_add_to_open(self,lnewnodes):
        self.open_nodes = sorted (
            self.open_nodes + lnewnodes, key=lambda x:x.evalfunc
        )

    # minor error -> more aproximate result; higher jump -> the algoritm, initially, computes a result more 
    # different than the previous, but can be faster to reach the global minimum of the error; to a better result 
    # in the minimum delta time, it would be interesting to change the jump over multiple runs, and make a plot with 
    # the number of cycles required to reach the global minimum of the error for some value of max_error
    def effective_branching_factor(self, max_error=0.000000000001, jump=0.1):
        # first guess (to save computing time)
        result = pow(self.total_nodes, 1/self.solution_length)
        
        current_error = self.error_ebf(result, self.solution_length)    # guess error
        
        state = current_error <= 0                                      # state to know when the current change from positive to negative and vice versa
        while abs(current_error) > max_error:
            if current_error > 0:
                if state:
                    jump *= 0.1
                state = False
                result -= jump
            else:
                if not state:
                    jump *= 0.1
                state = True
                result += jump
            
            current_error = self.error_ebf(result, self.solution_length)
        return  result

    def update_ancestors(self, node):
        if node.children:
            node.evalfunc = min([c.evalfunc for c in node.children])
        if node.parent:
            self.update_ancestors(node.parent)

    def discard_worse(self):
        max_value = 0
        non_terminal_node = None
        for c in self.open_nodes:
            if (c.parent.evalfunc > max_value 
                and all(e in self.open_nodes for e in c.parent.children)):
                non_terminal_node = c.parent
                max_value = c.parent.evalfunc

        for c in non_terminal_node.children:
            self.open_nodes.remove(c)
            self.terminal_nodes -= 1

        self.non_terminal_nodes -= 1
        self.terminal_nodes += 1

        non_terminal_node.children = None
        self.open_nodes.append(non_terminal_node)

    # recursive function to verify if the node already have a parent with the state = target 
    def is_parent(self, node, target):
        if node.parent == None:
            return False
        return node.parent.state == target or self.is_parent(node.parent, target) 

    def search2(self):
        while self.open_nodes != []:
            # discard less promissing leaf nodes
            while self.terminal_nodes + self.non_terminal_nodes > self.max_nodes:
                self.discard_worse()

            node = self.open_nodes.pop(0)
            if self.problem.goal_test(node.state):
                self.non_terminal_nodes -= 1                            # because the solution node is a terminal node
                self.terminal_nodes += 1                                # because the solution node is a terminal node

                self.solution_cost = node.cost                          # as the variable name indicates, the solution cost is equal to the solution node cost
                self.solution_length = node.depth                       # the tree length is equal to the solution node depth

                return self.get_path(node)

            lnewnodes = []
            # check the children of the node
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                newnode = MySearchNode(newstate,node)

                # to avoid redundancy, the states that already are parent of the current node are discarded
                if not self.is_parent(node, newstate):
                    # newnode operations
                    newnode.depth = node.depth + 1                      # the new node depth is equal to the parent node depth plus one (trivial)
                    newnode.cost = node.cost + \
                        self.problem.domain.cost(node.state, a)         # the new node comulative cost is equal to the parent node cost plus the cost between the new node and parent node (trivial)
                    newnode.evalfunc = newnode.cost + \
                        self.problem.domain.heuristic(
                            newstate, 
                            self.problem.goal
                        )                                               # the eval func, being the A* evaluation function, is equal to the new node cost and the problem heuristic value between the new node and the solution

                    # node operations
                    if not node.children:
                        node.children = []
                    node.children.append(newnode)                       # every newnode is a children of the node (trivial)

                    # tree operations
                    self.terminal_nodes += 1                            # terminal nodes candidates (if some node, later, turns to not be a terminal node, this value is decremented)
                    self.total_nodes += 1                               # in each node visit, this value is obviously incremented
                    
                    lnewnodes.append(newnode)
                self.update_ancestors(node)                             # propagate the evaluation function upwards 
                
            if node.children != []:
                self.non_terminal_nodes += 1
                self.terminal_nodes -= 1
                
            self.add_to_open(lnewnodes)
        return None

    # shows the search tree in the form of a listing
    def show(self,heuristic=False,node=None,indent=''):
        if node==None:
            self.show(heuristic,self.root)
            print('\n')
        else:
            line = indent+node.state
            if heuristic:
                line += (' [' + str(node.evalfunc) + ']')
            print(line)
            if node.children==None:
                return
            for n in node.children:
                self.show(heuristic,n,indent+'  ')
