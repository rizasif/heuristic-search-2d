from algorithm_abstract import Algorithm
from display_grid_2d import display_2d;
from robot_class import Robot;
from euclidian_heuristic import EucliHeur;

class chakra (Algorithm):

    Dsp = [];
    Heur = [];

    def __init__(self):
        self.OPEN1 = [];
        self.OPEN2 = [];
        self.CLOSED1 = [];
        self.CLOSED2 = [];

        self.Boundry = [[0, 0], [20, 20]];

        self.start = [0, 0, 0, 0];  # x,y,F,g
        self.goal = [20, 20, 0, 20];

        self.Islands = [[9, 15], [15,17]];  #Identify islands here

        self.obstacle = [
            [12, 12], [11, 13], [10, 14], [13, 11], [14, 10],
            [13, 13], [12, 14], [11, 15], [14, 12], [15, 11],
            [11, 11], [10, 13], [12, 10], [13, 9],
            [13, 12], [12, 13], [11, 12], [12, 11]
        ];

        # Riz Objects
        self.Dsp = display_2d(self.Boundry);
        self.Heur = EucliHeur(self.start, self.goal, self.obstacle, self.Islands);

    def getNext(self):
        x = [];
        if (len(self.OPEN1) == 0):
            x = self.OPEN2.pop(0);
            if (not self.isInList(x, self.CLOSED2)):
                self.CLOSED2.append(x);
                return 2, x;
            else:
                return 0, x;
        elif (len(self.OPEN2) == 0):
            x = self.OPEN1.pop(0);
            if (not self.isInList(x, self.CLOSED1)):
                self.CLOSED1.append(x);
                return 1, x;
            else:
                return 0, x;
        else:
            if (self.OPEN1[0][2] > self.OPEN2[0][2]):
                x = self.OPEN2.pop(0);
                if (not self.isInList(x, self.CLOSED2)):
                    self.CLOSED2.append(x);
                    return 2, x;
                else:
                    return 0, x;
            else:
                x = self.OPEN1.pop(0);
                if (not self.isInList(x, self.CLOSED1)):
                    self.CLOSED1.append(x);
                    return 1, x;
                else:
                    return 0, x;

    def updateFCost(self, n):
        n[2] = self.Heur.getHeuristic(n) + self.Heur.getFixedCost();

    def Expand(self, n):
        expanded = []

        x = [n[0] + 1, n[1], 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        x = [n[0], n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        x = [n[0] + 1, n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1], 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        x = [n[0], n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        x = [n[0] + 1, n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isInList(x, self.CLOSED1) and not self.isInList(x, self.CLOSED2)):
            self.updateFCost(x);
            expanded.append(x);
            self.Dsp.addSubPoint(x);

        if (len(expanded) == 0):
            return False, expanded
        else:
            return True, expanded


########################################

def main():
    alg = chakra();

    alg.Dsp.addPoint(alg.start);
    alg.Dsp.addGoalPoint(alg.goal);
    alg.Dsp.addIslandPoints(alg.Islands);
    alg.Dsp.addObstaclePoints(alg.obstacle);

    solutionfound = False;

    # step1
    alg.start[2] = alg.Heur.getHeuristic(alg.start);
    alg.OPEN1.append(alg.start);

    while (1):

        # step2
        if (alg.isOpenEmpty()):
            print("OPEN is empty");
            solutionfound = False;
            break;

        # step3
        CONT, n = alg.getNext();

        if (CONT == 0):
            print("Node Already in Closed list");
            continue;

        print("\n New Point");
        print(n);

        # step4
        if (alg.isGoal(n)):
            solutionfound = True;
            break;

        # step5
        succ_exist, succ = alg.Expand(n);
        if (not succ_exist):
            print("No Successor Found");
            continue;
        for s in succ:
            s[3] = s[3] + alg.Heur.getFixedCost();

        # step6
        if (CONT == 2 or alg.isIsland(n)):
            for s in succ:
                inCL1, r = alg.getFromList(s, alg.CLOSED1);
                inCL2, r = alg.getFromList(s, alg.CLOSED2);
                inOP1, r = alg.getFromList(s, alg.OPEN1);
                inOP2, r = alg.getFromList(s, alg.OPEN2);

                # 6.1
                if (not inCL1 and not inCL2 and not inOP1 and not inOP2):
                    print("6: Not in any list");
                    s[3] = alg.Heur.getFixedCost();
                    s[2] = s[3] + alg.Heur.getHeuristic(s);
                    alg.OPEN2.append(s);

                # 6.2
                else:
                    # 6.2.1
                    if (s[3] > alg.Heur.getFixedCost()):
                        s[3] = alg.Heur.getFixedCost();

                    # 6.2.2
                    s[2] = s[3] + alg.Heur.getHeuristic(s);

                    # 6.2.3
                    if (inCL1):
                        print("6: CLOSED1")
                        alg.CLOSED1.append(s);
                    elif (inCL2):
                        print("6: CLOSED2")
                        alg.CLOSED2.append(s);
                    else:
                        print("6: OPEN2 or OPEN1")
                        alg.OPEN2.append(s);

        # step7
        elif (CONT == 1 or not alg.isIsland(n)):
            for s in succ:
                inCL1, r = alg.getFromList(s, alg.CLOSED1);
                inCL2, r = alg.getFromList(s, alg.CLOSED2);
                inOP1, r = alg.getFromList(s, alg.OPEN1);
                inOP2, r = alg.getFromList(s, alg.OPEN2);

                # 7.1
                if (not inCL1 and not inCL2 and not inOP1 and not inOP2):
                    print("7: Not in any list");
                    #s[3] = alg.Heur.getFixedCost();
                    s[3] = 1;
                    d = alg.Heur.getMinH(s) - alg.Heur.getHeuristic(s);
                    if (d > 0):
                        s[2] = s[3] + alg.Heur.getMinH(s);
                        alg.OPEN1.append(s);
                    else:
                        s[2] = s[3] + alg.Heur.getHeuristic(s);
                        alg.OPEN2.append(s);

                # 7.2
                elif (inOP1):
                    print("7: OPEN1");
                    if (s[3] > alg.Heur.getFixedCost()):
                        s[3] = alg.Heur.getFixedCost();
                        s[2] = s[3] + alg.Heur.getMinH(s);
                        alg.OPEN1.append(s);

                # 7.3
                elif (inOP2):
                    print("7: OPEN2");
                    s[3] = alg.Heur.getFixedCost();
                    s[2] = s[3] + alg.Heur.getHeuristic(s);
                    alg.OPEN2.append(s);

                elif (inCL1):
                    print("7: CLOSED1");
                    alg.CLOSED1.append(s);

                elif (inCL2):
                    print("7: CLOSED2");
                    alg.CLOSED2.append(s);

        alg.sortList(alg.OPEN1);
        alg.sortList(alg.OPEN2);
        alg.Dsp.addPoint(n);
        print(alg.OPEN1);
        print(alg.OPEN2);

    if (solutionfound):
        print("Solution Found");
    else:
        print("Solution NOT Found");

    alg.Dsp.BlockGraph();

if __name__ == "__main__":
    main()
