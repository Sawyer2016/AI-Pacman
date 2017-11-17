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
from game import Directions
import distanceCalculator
import game
from util import nearestPoint


#################`
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='OffensiveAgent', second='DefensiveAgent'):
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class BaseAgnet(CaptureAgent):
    def registerInitialState(self, gameState):
        self.start = gameState.getAgentPosition(self.index)
        CaptureAgent.registerInitialState(self, gameState)

    def getBoundary(self, gameState):
        pos = []
        x = gameState.getWalls().width / 2
        y = gameState.getWalls().height / 2
        if self.red:
            x = x - 2
        else:
            x = x + 2
        while y >= 0:
            if gameState.hasWall(x, y) == False:
                pos.append((x, y))
            y -= 1
        # print pos
        return pos

    def chooseAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        values = [self.evaluate(gameState, a) for a in actions]
        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        if foodLeft <= 2:
            bestDist = 9999
            for action in actions:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.getMazeDistance(self.start, pos2)
                if dist < bestDist:
                    bestAction = action
                    bestDist = dist
            return bestAction

        return random.choice(bestActions)

    def getSuccessor(self, gameState, action):

        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def evaluate(self, gameState, action):

        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights


class OffensiveAgent(BaseAgnet):
    def getFeatures(self, gameState, action):
        features = util.Counter()

        successor = self.getSuccessor(gameState, action)
        boundary = self.getBoundary(gameState)
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        myState = successor.getAgentState(self.index)

        myPos = myState.getPosition()

        foodList = self.getFood(successor).asList()
        features['successorScore'] = -len(foodList)
        capsulesList = self.getCapsules(gameState)

        ## find the nearest food
        if len(foodList) > 0:
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        ## find the nearest capsule
        if len(capsulesList) > 0:
            minCapDistance = min([self.getMazeDistance(myPos, cap) for cap in capsulesList])
            features['distanceToCapsule'] = minCapDistance
            features['capScore'] = -len(capsulesList)

        ## keep distance to ghost!
        minEnemyDis = 100
        for enemy in enemies:
            if enemy.getPosition():
                enemyPos = enemy.getPosition()
                enemyDis = self.getMazeDistance(myPos, enemyPos)
                if minEnemyDis > enemyDis:
                    minEnemyDis = enemyDis
                    minEnemy = enemy
        if myState.isPacman:
            features['enemyDis'] = minEnemyDis

        if minEnemyDis < 5 and minEnemy.scaredTimer == 0 and not minEnemy.isPacman:
            features['successorScore'] = -len(capsulesList)
            features['distanceToFood'] = 0
            minBPos = self.getMazeDistance(myPos, self.start)
            features['backHomeDis'] = minBPos

        if minEnemyDis < 1 and minEnemy.scaredTimer > 0 and not minEnemy.isPacman:
            features['reverse'] = 100

        if action == Directions.STOP:
            features['stop'] = 1

        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev:
            features['reverse'] = 1

        return features

    def getWeights(self, gameState, action):
        return {'successorScore': 100, 'distanceToFood': -1, 'distanceToCapsule': -1, 'enemyDis': 1, 'stop': -100,
                'reverse': -2, 'backHomeDis': -2}


class DefensiveAgent(BaseAgnet):
    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)

        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()
        x, y = myPos
        x = int(x)
        y = int(y)
        count = 0
        left = 0
        right = gameState.getWalls().width - 1
        bot = 0
        top = gameState.getWalls().height - 1
        ran = 3

        if x - ran >= left and x + ran <= right and y - ran >= bot and y + ran <= top:
            for a in range(x - ran, x + ran):
                for b in range(y - ran, y + ran):
                    pos = a, b
                    if self.getFoodYouAreDefending(successor)[a][b] and self.distancer.getDistance(
                            successor.getAgentPosition(self.index), pos) < 10:
                        count += 1

        # Computes whether we're on defense (1) or offense (0)
        features['onDefense'] = 1
        if myState.isPacman:
            features['onDefense'] = 0

        # Computes distance to invaders we can see
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        features['numInvaders'] = len(invaders)

        if len(invaders) > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            features['invaderDistance'] = min(dists)

        if len(invaders) == 0:
            features['ourFoods'] = count
        else:
            features['ourFoods'] = 0

        if action == Directions.STOP:
            features['stop'] = 1
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev:
            features['reverse'] = 1

        return features

    def getWeights(self, gameState, action):
        return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2,
                'ourFoods': 2}

