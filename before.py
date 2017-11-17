# myTeam.py
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


from captureAgents import CaptureAgent
import random, time, util
import game
from game import Grid
import distanceCalculator
import random, time, util, sys
from game import Directions,Actions
from util import nearestPoint


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='DefenseAgent', second='OffenseAgent'):
    """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

    # The following line is an example only; feel free to change it.
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class BaseAgent(CaptureAgent):
    """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

    def registerInitialState(self, gameState):
        """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

        '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
        self.start = gameState.getAgentPosition(self.index)
        CaptureAgent.registerInitialState(self, gameState)

        '''
    Your initialization code goes here, if you need any.
    '''

    def getSuccessor(self, gameState, action):
        """
    Finds the next successor which is a grid position (location tuple).
    """
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def __init__(self, index):
        CaptureAgent.__init__(self, index)
        # Variables used to verify if the agent os locked
        self.foodNum = 999
        self.trappedTime = 0

    def evaluate(self, gameState, action):
        """
    Computes a linear combination of features and feature weights
    """
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    def getClosestGhosts(self, gameState):
        ghosts = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        myPos = gameState.getAgentState(self.index).getPosition()
        distToGhost = 200
        for ghost in ghosts:
            if ghost.getPosition():
                dist = self.getMazeDistance(myPos, ghost.getPosition())
                if distToGhost > dist:
                    distToGhost = dist
        return distToGhost

    def chooseAction(self, gameState):
        """
    Picks among the actions with the highest Q(s,a).
    """
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()

        values = [self.evaluate(gameState, a) for a in actions]
        # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        if foodLeft <= 2:
            bestDist = 9999
            for action in actions:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.aStarSearch(gameState, pos2)
                if dist < bestDist:
                    bestAction = action
                    bestDist = dist
            return bestAction

        return random.choice(bestActions)

    def aStarSearch(self, gameState, po2):
        true_cost = 0
        po1 = gameState.getAgentPosition(self.index)
        heu_cost = 0 + self.getMazeDistance(po1, po2)
        closed_states = []
        result = util.PriorityQueue()
        actions = []
        initial_state = (po1, actions, true_cost, heu_cost)
        result.push(initial_state, heu_cost)
        while not (result.isEmpty()):
            (state, actions, true_cost, hue_cost) = result.pop()
            if state == po2:
                return len(actions)
            if not state in closed_states:
                closed_states.append(state)
                legalActions = gameState.getLegalActions(self.index)
                for action in legalActions:
                    next_action = actions + [action]
                    next_true_cost = true_cost + 1
                    successor = self.getSuccessor(gameState, action)
                    if successor.getAgentState(self.index).getPosition()!=None:
                      myPos = successor.getAgentState(self.index).getPosition()
                      next_hu_cost = next_true_cost + self.getMazeDistance(myPos, po2)
                      next_state = (myPos, next_action, next_true_cost, next_hu_cost)
                      result.push(next_state, next_hu_cost)
                    else:
                      return self.getMazeDistance(po1,po2)

    def randomEvaluate(self, step, gameState):
        newState = gameState.deepCopy()
        while step > 0:
            actions = newState.getLegalActions(self.index)
            actions.remove(Directions.STOP)
            a = random.choice(actions)
            newState = newState.generateSuccessor(self.index, a)
            step -= 1
        return self.evaluate(newState, Directions.STOP)


class DefenseAgent(BaseAgent):
   
    def getMajorFoof(self,myPos,gameState,action):
      successor = self.getSuccessor(gameState, action)
      x = int(myPos[0])
      y = int(myPos[1])
      numFood = 0
      lBorder = 0
      rBorder = gameState.getWalls().width - 1
      bBorder = 0
      tBorder = gameState.getWalls().height - 1
      rangeSize = 3

      if x - rangeSize >= lBorder and x + rangeSize <= rBorder and y - rangeSize >= bBorder and y + rangeSize <= tBorder:
        for a in range(x - rangeSize, x + rangeSize):
          for b in range(y - rangeSize, y + rangeSize):
            pos = a, b
            if self.getFoodYouAreDefending(successor)[a][b] and self.distancer.getDistance(
                    successor.getAgentPosition(self.index), pos) < 10:
              numFood += 1
      return numFood
    def getFeatures(self, gameState, action):
      features = util.Counter()
      successor = self.getSuccessor(gameState, action)
      myState = successor.getAgentState(self.index)
      myPos = myState.getPosition()
      numFood=self.getMajorFoof(myPos,gameState,action)
      if myState.isPacman:
       features['isDefense'] = 0
      else:
       features['isDefense'] = 1
      opponents = [successor.getAgentState(i) for i in self.getOpponents(successor)]
      enemies = [i for i in opponents if i.isPacman and i.getPosition() != None]
      # features['numInvaders'] = len(invaders)

      if len(enemies) > 0:
        dist = [self.getMazeDistance(myPos, a.getPosition()) for a in enemies]
        if gameState.getAgentState(self.index).scaredTimer > 0:
            features['distToEnemies'] = -min(dist)
        else:
            features['distToEnemies'] = min(dist)


      if len(enemies) == 0:
        features['majorFoods'] = numFood
      else:
        features['majorFoods'] = 0

      # if action == Directions.STOP:
      #   features['stop'] = 1
      reverse = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
      if action == reverse:
        features['reverse'] = 1

      return features

    def getWeights(self, gameState, action):
      return { 'isDefense': 100, 'distToEnemies': -10, 'reverse': -10,
              'majorFoods': 10}


class OffenseAgent(BaseAgent):
    """
  A reflex agent that seeks food. This is an agent
  we give you to get an idea of what an offensive agent might look like,
  but it is by no means the best or only way to build an offensive agent.
  """

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        foodList = self.getFood(successor).asList()
        ghosts = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        powerDots = self.getCapsules(gameState)
        features['successorScore'] = -len(foodList)  # self.getScore(successor)

        # Compute distance to the nearest food

        if len(foodList) > 0:  # This should always be True,  but better safe than sorry
            myPos = successor.getAgentState(self.index).getPosition()
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        # compute the distance to nearest ghost
        distToGhost = 200
        for ghost in ghosts:
            if ghost.getPosition():
                dist = self.getMazeDistance(myPos, ghost.getPosition())
                if distToGhost > dist:
                    distToGhost = dist
                    nearestGhost = ghost
        if myState.isPacman:
            features['DistToNearestGhost'] = distToGhost

        # if len(powerDots) > 0:
        #     distToPower = min([self.getMazeDistance(myPos, dot) for dot in powerDots])
        #     print distToPower
        #     features['distanceToPower'] = distToPower

        if self.trappedTime > 50:
            features['goHome'] = 0
            if successor.getAgentState(self.index).isPacman:
                features['attack'] = 1
            else:
                features['attack'] = 0
        else:
            if distToGhost < 5 and nearestGhost.scaredTimer == 0 and not nearestGhost.isPacman:
                features['distanceToFood'] = 0
                features['successorScore'] = 0
                centerX = gameState.getWalls().width / 2
                if self.red:
                    centerX = centerX - 1
                else:
                    centerX = centerX + 1
                features['goHome'] = abs(myPos[0] - centerX)
                # features['distanceToPower'] = 0

        return features

    def isBadMove(self, gameState, action, step):
        distToGhost = self.getClosestGhosts
        successor = gameState.generateSuccessor(self.index, action)
        currentNumFood = len(self.getFood(gameState).asList())
        newNumFood = len(self.getFood(successor).asList())
        if step == 0:
            return False
        if currentNumFood > newNumFood and distToGhost >= 5:
            return False
        actions = successor.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        return_direction = Directions.REVERSE[successor.getAgentState(self.index).configuration.direction]
        if return_direction in actions:
            actions.remove(return_direction)
        if len(actions) == 0:
            return True
        for action in actions:
            if not self.isBadMove(successor, action, step - 1):
                return False
        return True

    def getWeights(self, gameState, action):

        return {'successorScore': 1000, 'distanceToFood': -20, 'DistToNearestGhost': 20,
                'goHome': -40, 'attack': 1000}

    def chooseAction(self, gameState):
        """
    Picks among the actions with the highest Q(s,a).
    """
        currentNumFood = len(self.getFood(gameState).asList())
        if self.foodNum != currentNumFood:
            self.foodNum = currentNumFood
            self.trappedTime = 0
        else:
            self.trappedTime += 1
        if gameState.getInitialAgentPosition(self.index) == gameState.getAgentState(self.index).getPosition():
            self.trappedTime = 0

        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        goodAction = []
        for a in actions:
            if not self.isBadMove(gameState, a, 5):
                goodAction.append(a)
        if len(goodAction) == 0:
            goodAction = actions
        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()

        # values = [self.evaluate(gameState, a) for a in actions]
        # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

        # maxValue = max(values)
        # bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        if foodLeft <= 2:
            bestDist = 9999
            for action in goodAction:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.getMazeDistance(self.start, pos2)
                if dist < bestDist:
                    bestAction = action
                    bestDist = dist
            return bestAction

        else:
            values = []
            for a in goodAction:
                newState = gameState.generateSuccessor(self.index, a)
                value = 0
                for i in range(1, 30):
                    value += self.randomEvaluate(1, newState)
                values.append(value)
            bestValue = max(values)
            bestActions = []
            for (value, action) in zip(values, goodAction):
                if value == bestValue:
                    bestActions.append(action)

        return random.choice(bestActions)




