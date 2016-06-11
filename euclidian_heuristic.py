import math


class EucliHeur:

    Obstacles = [];
    Start = [];
    Goal = [];
    Islands = [];

    def __init__(self, start, goal, obstacles, islands):
        self.Obstacles = obstacles;
        self.Start = start;
        self.Goal = goal;
        self.Islands = islands;

    def isInCollision(self, n):
        for i in self.Obstacles:
            if (i[0] == n[0] and i[1] == n[1]):
                return True;
        return False;

    def getDistance(self, n1, n2):
        x = (math.pow((n1[0] - n2[0]), 2)) + (math.pow((n1[1] - n2[1]), 2));
        return math.sqrt(x);

    def getHeuristic(self, n):
        x = (math.pow((n[0] - self.Goal[0]), 2)) + (math.pow((n[1] - self.Goal[1]), 2));
        if (self.isInCollision(n)):
            return 2000;
        else:
            return math.sqrt(x);

    def getCost(self, n):
        x = (math.pow((n[0] - self.Start[0]), 2)) + (math.pow((n[1] - self.Start[1]), 2));
        return x;

    def getFixedCost(self):
        return 1;

    def getMinH(self, n):
        idist = 2000;
        print "getiing min H"
        for i in self.Islands:
            if (self.getDistance(n, i) < idist):
                idist = self.getDistance(n, i) + self.getDistance(i, self.Goal);
                #idist = self.getDistance(self.Goal,i);
        return idist;