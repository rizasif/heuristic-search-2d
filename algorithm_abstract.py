
class Algorithm:
    OPEN1 = [];
    OPEN2 = [];
    CLOSED1 = [];
    CLOSED2 = [];

    Boundry = [[], []];

    start = [];  # x,y,F,g
    goal = [];

    Islands = [];

    obstacle = [];

    def isOpenEmpty(self):
        if (len(self.OPEN1) == 0 and len(self.OPEN2) == 0):
            return True;
        else:
            return False;

    def isGoal(self, n):
        if (n[0] == self.goal[0] and n[1] == self.goal[1]):
            return True;
        else:
            return False;

    def sortList(self, l):
        l.sort(key=lambda x: x[2]);

    def isInLimits(self, n):
        if (n[0] >= self.Boundry[0][0] and n[1] >= self.Boundry[0][1]):
            if (n[0] <= self.Boundry[1][0] and n[1] <= self.Boundry[1][1]):
                return True;
        return False;

    def isInList(self, n, li):
        for i in range(len(li)):
            if (li[i][0] == n[0] and li[i][1] == n[1]):
                return True;
        return False;

    def isIsland(self, n):
        try:
            self.Islands.index([n[0], n[1]]);
            return True;
        except:
            return False;

    #def getFromList(self, n, li):
    #    for node in li:
    #        if (node[0] == n[0] and node[1] == n[1]):
    #            return True, node;
    #    return False, n;

    def getFromList(self, n, li):
        for i in range(len(li)):
            if (li[i][0] == n[0] and li[i][1] == n[1]):
                return True, i;
        return False, 0;

    #Abstract
    def getNext(self):
        raise NotImplementedError("Please Implement this method");

    def updateFCost(n):
        raise NotImplementedError("Please Implement this method");

    def Expand(n):
        raise NotImplementedError("Please Implement this method");


