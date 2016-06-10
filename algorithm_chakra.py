from display_grid_2d import display_2d;
from robot_class import Robot;
from euclidian_heuristic import EucliHeur;


OPEN1 = [];
OPEN2 = [];
CLOSED1 = [];
CLOSED2 = [];

Boundry = [[0,0], [20,20]];

start = [0, 0, 0, 0]; #x,y,F,g
goal = [20, 20, 0, 20];

isl = [[9,15]];

obstacle = [
    [12,12], [11,13], [10,14], [13,11], [14,10],
    [13,13], [12,14], [11,15], [14,12], [15,11],
    [11,11], [10,13],  [12,10], [13,9],
    [13,12], [12,13], [11,12], [12, 11]
];

#Riz Objects
Dsp = display_2d(Boundry);
Bot = Robot(Boundry, start);
Heur = EucliHeur(start, goal, obstacle, isl);

def isOpenEmpty():
    if(len(OPEN1) == 0 and len(OPEN2) == 0):
        return True;
    else:
        return False;

def isGoal(n):
    if(n[0] == goal[0] and n[1] == goal[1]):
        return True;
    else:
        return False;

def sortList(l):
    l.sort(key=lambda x: x[2]);


def getNext():
    x = [];
    if (len(OPEN1) == 0):
        x = OPEN2.pop(0);
        if (not isInList(x, CLOSED2)):
            CLOSED2.append(x);
            return 2, x;
        else:
            return 0, x;
    elif (len(OPEN2) == 0):
        x = OPEN1.pop(0);
        if (not isInList(x, CLOSED1)):
            CLOSED1.append(x);
            return 1, x;
        else:
            return 0, x;
    else:
        if (OPEN1[0][2] > OPEN2[0][2]):
            x = OPEN2.pop(0);
            if (not isInList(x, CLOSED2)):
                CLOSED2.append(x);
                return 2, x;
            else:
                return 0, x;
        else:
            x = OPEN1.pop(0);
            if (not isInList(x, CLOSED1)):
                CLOSED1.append(x);
                return 1, x;
            else:
                return 0, x;


def isInLimits(n):
    if (n[0] >= Boundry[0][0] and n[1] >= Boundry[0][1]):
        if (n[1] <= Boundry[1][0] and n[1] <= Boundry[1][1]):
            return True;
    return False;

def updateFCost(n):
    n[2] = Heur.getHeuristic(n) + Heur.getFixedCost();


def Expand(n):
    expanded = []

    x = [n[0] + 1, n[1], 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    x = [n[0], n[1] + 1, 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    x = [n[0] + 1, n[1] + 1, 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    x = [n[0] - 1, n[1], 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    x = [n[0], n[1] - 1, 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    x = [n[0] - 1, n[1] - 1, 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    x = [n[0] + 1, n[1] - 1, 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    x = [n[0] - 1, n[1] + 1, 0, 1000]
    if (isInLimits(x) and not isInList(x, CLOSED1) and not isInList(x, CLOSED2)):
        updateFCost(x);
        expanded.append(x);
        Dsp.addSubPoint(x);

    if (len(expanded) == 0):
        return False, expanded
    else:
        return True, expanded


def isInList(n, li):
    for i in range(len(li)):
        if (li[i][0] == n[0] and li[i][1] == n[1]):
            return True;
    return False;


def isIsland(n):
    try:
        isl.index([n[0], n[1]]);
        return True;
    except:
        return False;


def getFromList(n, li):
    for node in li:
        if (node[0] == n[0] and node[1] == n[1]):
            return True, node;
    return False, n;


####################################################################
def main():
    Dsp.addPoint(start);
    Dsp.addGoalPoint(goal);
    Dsp.addIslandPoints(isl);
    Dsp.addObstaclePoints(obstacle);

    solutionfound = False;

    # step1
    start[2] = Heur.getHeuristic(start);
    OPEN1.append(start);

    while (1):

        # step2
        if (isOpenEmpty()):
            print("OPEN is empty");
            solutionfound = False;
            break;

        # step3
        CONT, n = getNext();

        if (CONT == 0):
            print("Node Already in Closed list");
            continue;

        print("\n New Point");
        print(n);

        # step4
        if (isGoal(n)):
            solutionfound = True;
            break;

        # step5
        succ_exist, succ = Expand(n);
        if (not succ_exist):
            print("No Successor Found");
            continue;
        for s in succ:
            s[3] = s[3] + Heur.getFixedCost();

        # step6
        if (CONT == 2 or isIsland(n)):
            for s in succ:
                inCL1, s = getFromList(s, CLOSED1);
                inCL2, s = getFromList(s, CLOSED2);
                inOP1, s = getFromList(s, OPEN1);
                inOP2, s = getFromList(s, OPEN2);

                # 6.1
                if (not inCL1 and not inCL2 and not inOP1 and not inOP2):
                    print("6: Not in any list");
                    s[3] = Heur.getFixedCost();
                    s[2] = s[3] + Heur.getHeuristic(s);
                    OPEN2.append(s);

                # 6.2
                else:
                    # 6.2.1
                    if (s[3] > Heur.getFixedCost()):
                        s[3] = Heur.getFixedCost();

                    # 6.2.2
                    s[2] = s[3] + Heur.getHeuristic(s);

                    # 6.2.3
                    if (inCL1):
                        print("6: CLOSED1")
                        CLOSED1.append(s);
                    elif (inCL2):
                        print("6: CLOSED2")
                        CLOSED2.append(s);
                    else:
                        print("6: OPEN2 or OPEN1")
                        OPEN2.append(s);

        # step7
        elif (CONT == 1 or not isIsland(n)):
            for s in succ:
                inCL1, s = getFromList(s, CLOSED1);
                inCL2, s = getFromList(s, CLOSED2);
                inOP1, s = getFromList(s, OPEN1);
                inOP2, s = getFromList(s, OPEN2);

                # 7.1
                if (not inCL1 and not inCL2 and not inOP1 and not inOP2):
                    print("7: Not in any list");
                    s[3] = Heur.getFixedCost();
                    d = Heur.getMinH(s) - Heur.getHeuristic(s);
                    if (d > 0):
                        s[2] = s[3] + Heur.getMinH(s);
                        OPEN1.append(s);
                    else:
                        s[2] = s[3] + Heur.getHeuristic(s);
                        OPEN2.append(s);

                # 7.2
                elif (inOP1):
                    print("7: OPEN1");
                    if (s[3] > Heur.getFixedCost()):
                        s[3] = Heur.getFixedCost();
                        s[2] = s[3] + Heur.getMinH(s);
                    OPEN1.append(s);

                # 7.3
                elif (inOP2):
                    print("7: OPEN2");
                    s[3] = Heur.getFixedCost();
                    s[2] = s[3] + Heur.getHeuristic(s);
                    OPEN2.append(s);

                elif (inCL1):
                    print("7: CLOSED1");
                    CLOSED1.append(s);

                elif (inCL2):
                    print("7: CLOSED2");
                    CLOSED2.append(s);

        sortList(OPEN1);
        sortList(OPEN2);
        Dsp.addPoint(n);
        print(CLOSED1);
        print(CLOSED2);

    if (solutionfound):
        print("Solution Found");
    else:
        print("Solution NOT Found");

    Dsp.BlockGraph();


if __name__ == "__main__":
    main()
