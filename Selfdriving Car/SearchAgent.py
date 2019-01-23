import random
import math
from environment import Agent, Environment
from simulator import Simulator
import sys
from searchUtils import searchUtils


class SearchAgent(Agent):
    """ An agent that self drives in the environment.
        This is the object you will be modifying. """ 

    def __init__(self, env,location=None):
        super(SearchAgent, self).__init__(env)     # Set the agent in the evironment 
        self.valid_actions = self.env.valid_actions  # The set of valid actions
        self.searchutil = searchUtils(env)
        self.action_sequence=[]

    def choose_action(self):
        """ The choose_action function is called when the agent is asked to choose
            which action to take next"""
        action=None
        if len(self.action_sequence) >=1:
            action = self.action_sequence[0]
            
        if len(self.action_sequence) >=2:
            self.action_sequence=self.action_sequence[1:]
        else:
            self.action_sequence=[]
        return action
        
    def drive(self,goalstates,inputs):
        """Write your algorithm for self driving car"""        
        openList = [] # My list used to prioritize gridlocations.
        resultList = [] # The list with actions that the function returns
        startState = self.state["location"]
        xLocation = startState[0]
        yLocation = startState[1]
        
        #note that these two for-loops also could have used the visibility_range variable and thereby scan the whole
        #visibility range. However this is unnecessary as the car only can move one step in x-direction.
        for k1 in range(self.env.xadd): #xadd is defined in environment.py as 2
            for k2 in range(self.env.yadd): #yadd is defined in environment.py as 3
                nextX = xLocation + k1
                nextNegX = xLocation - k1
                nextY = yLocation + k2   
                nextStateHeuristic = self.env.ymax - (nextY) #the distance from possible next state to a goal state 
                
                #everything in the if statement underneath is based on that the car would move forward or to the right.
                if(self.env.isvalidloc(nextX,nextY,inputs)): #Checks to see if the possible next state is on the grid
                    
                    if(inputs[nextX][nextY] == 1): #checks if a car is in that state
                        continue
                    if(inputs[nextX][nextY-1] == 1 and nextY-1 != yLocation): #checks if a car is in the way of getting to that state
                        continue
                        
                    nextState = {}
                    nextState["location"] = [nextX, nextY]
                    
                    if not (self.searchutil.isPresentStateInPriorityList(nextState,openList)): #if the state has not yet been discovered
                        self.searchutil.insertStateInPriorityQueue(openList,nextState,nextStateHeuristic)
                
                #Everything in the if statement underneath is based on that the car would move to the left.
                if(self.env.isvalidloc(nextNegX,nextY,inputs) and nextX != nextNegX): 
                    #nextX != nextNegX to avoid multiple checks at same location that is when nextX == 0 == nextNegX
                    
                    if(inputs[nextNegX][nextY] == 1): #checks if a car is in that state
                        continue
                    if(inputs[nextNegX][nextY-1] == 1 and nextY-1 != yLocation): #checks if a car is in the way of getting to that state
                        continue
                        
                    nextStateNeg = {}
                    nextStateNeg["location"] = [nextNegX, nextY]

                    if not (self.searchutil.isPresentStateInPriorityList(nextStateNeg,openList)): #if the state has not yet been discovered
                        self.searchutil.insertStateInPriorityQueue(openList,nextStateNeg,nextStateHeuristic)  
                       
        for i in range(len(openList)):
            nextState = openList[i][0] #openList[i][0] contains a state we're willing to go to.
            action = self.env.getAction(self.state,nextState) #
            if not(action == None): #this should never happend as we have already evaluated all the states in openList.
                resultList.append(action) 
                return resultList #returns the answer as fast as a solution is given

        return resultList 
    def update(self):
        """ The update function is called when a time step is completed in the 
            environment choose an action """
        startstate = self.state
        goalstates = self.env.getGoalStates() #list of goal states
        inputs = self.env.sense(self)
                
        self.action_sequence = self.drive(goalstates,inputs)
                
        action = self.choose_action()  # Choose an action
        self.state = self.env.act(self,action)        
        return
        

def run(filename):
    """ Driving function for running the simulation. 
        Press ESC to close the simulation, or [SPACE] to pause the simulation. """
    #change fixmovement to True to fix the movement sequence of other cars across runs

    env = Environment(config_file=filename,fixmovement=False)
    agent = env.create_agent(SearchAgent)
    env.set_primary_agent(agent)
    
    ##############
    # Create the simulation
    # Flags:
    #   update_delay - continuous time (in seconds) between actions, default is 2.0 seconds
    #   display      - set to False to disable the GUI if PyGame is enabled
    sim = Simulator(env, update_delay=2)
    
    ##############
    # Run the simulator
    ##############
    sim.run()


if __name__ == '__main__':
    run(sys.argv[1])
