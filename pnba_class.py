from algorithm_abstract import Algorithm
from display_grid_2d import display_2d;
from euclidian_heuristic import EucliHeur;
from print_class import PrintClass;
from traj_publisher import TrajPub;

import threading;
import multiprocessing;

class pnba (Algorithm):
    Dsp = [];
    Heur = [];

    #Static
    M = [];
    L = 2000;
    solution_found = False;

    printfile = None;

    ThreadID = None;

    #staticFlags
    SOLUTION_FOUND = "solution_found"
    GOAL_FOUND = "goal_found"
    NO_SUCC = "no_succ"
    SUCC = "succ"
    UNDISCOVERED = "undisc"
    DISCOVERED = "disc"
    FALIURE = "faliure"

    def __init__(self, boundry, start, goal, obstacle,thread_id):
        self.OPEN1 = [];
        self.OPEN2 = [];
        self.CLOSED1 = [];
        self.CLOSED2 = [];

        self.Boundry = boundry;
        self.start = start;  # x,y,F,g
        self.goal = goal;

        #self.Islands = [[9, 15], [15, 17]];
        self.Islands=[[]];

        self.obstacle = obstacle;

        # Riz Objects
        #pnba.Dsp = display_2d(self.Boundry);
        self.Heur = EucliHeur(self.start, self.goal, self.obstacle , self.Islands);

        self.printfile = PrintClass(thread_id);

        self.ThreadID = thread_id;

    def isDiscovered(self, n):
        if(pnba.M[n[0]][n[1]] == self.ThreadID):
            return True;
        else:
            return False;

    def getNext(self):
        n = self.OPEN1.pop(0);
        if(pnba.M[n[0]][n[1]] > 0):
            if(pnba.M[n[0]][n[1]] == self.ThreadID):
                return pnba.DISCOVERED, n
            elif(self.isGoal(n)):
                return pnba.GOAL_FOUND, n
            elif(pnba.M[n[0]][n[1]] != self.ThreadID):
                return pnba.SOLUTION_FOUND, n
            else:
                return pnba.FALIURE, n
        elif(pnba.M[n[0]][n[1]] < 0):
            return pnba.UNDISCOVERED, n
        else:
            return pnba.FALIURE, n;

    def updateFCost(self, n):
        n[2] = self.Heur.getHeuristic(n) + self.Heur.getFixedCost();

    def Expand(self, n):
        expanded = []

        x = [n[0] + 1, n[1], 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        x = [n[0], n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        x = [n[0] + 1, n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1], 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        x = [n[0], n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        x = [n[0] + 1, n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba.Dsp.addSubPoint(x);

        if (len(expanded) == 0):
            return False, expanded
        else:
            return True, expanded

    def run(self):
        MyThread.threadLock.acquire();
        #pnba.Dsp.addPoint(self.start);
        #pnba.Dsp.addGoalPoint(self.goal);
        self.printfile.writeToFile(self.start);
        MyThread.threadLock.release();
        ##pnba.Dsp.addIslandPoints(self.Islands);
        ##pnba.Dsp.addObstaclePoints(self.obstacle);

        pnba.solution_found = False;

        # step1
        self.start[2] = self.Heur.getHeuristic(self.start);
        self.OPEN1.append(self.start);

        while(1):
            if (self.isOpenEmpty()):
                print("OPEN is empty");
                pnba.solution_found = False;
                break;

            t, x = self.getNext();

            if(t == pnba.GOAL_FOUND):
                print "GOAL_FOUND by: ", self.ThreadID
                break;
            elif(t == pnba.SOLUTION_FOUND):
                print "SOLUTION_FOUND by: ", self.ThreadID
                break;
            elif(t == pnba.DISCOVERED):
                continue;
            elif(t == pnba.FALIURE):
                print "FALIURE";

            MyThread.threadLock.acquire();
            self.printfile.writeToFile(x);
            MyThread.threadLock.release();


            if(x[2] <= pnba.L):
                succ_exist, succ = self.Expand(x);
                if (not succ_exist):
                    print("No Successor Found");
                    continue;

                for s in succ:
                    s[3] = s[3] + self.Heur.getFixedCost();

                    if(self.isInList(s,self.OPEN1)):
                        temp, element = self.getFromList(s, self.OPEN1);
                        self.OPEN1.pop(element);
                    self.OPEN1.append(s);

                    h = self.Heur.getHeuristic(s) + self.Heur.getCost(s);
                    if(h < pnba.L):
                        MyThread.threadLock.acquire();
                        h = self.Heur.getHeuristic(s) + self.Heur.getCost(s);
                        if (h < pnba.L):
                            pnba.L = h;
                        MyThread.threadLock.release();

            pnba.M[x[0]][x[1]] = self.ThreadID;

            self.sortList(self.OPEN1);

        self.printfile.file.close();


class MyThread(threading.Thread):
    threadLock = threading.Lock();

    Algo = [];

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def setParams(self, boundry, start, goal, obstacle):
        self.Algo = pnba(boundry,start,goal, obstacle, self.name);

    def run(self):
        self.Algo.run();


########################################
def main():
    threads = [];

    Boundry = [[0, 0], [20, 20]];

    start = [0, 0, 0, 0];  # x,y,F,g
    goal = [20, 20, 0, 20];

    obstacle = [
        [12, 12], [11, 13], [10, 14], [13, 11], [14, 10],
        [13, 13], [12, 14], [11, 15], [14, 12], [15, 11],
        [11, 11], [10, 13], [12, 10], [13, 9],
        [13, 12], [12, 13], [11, 12], [12, 11],
        [11, 14], [14, 11], [13, 10]
    ];

    pnba.M = [[-1 for x in range(Boundry[1][0] - Boundry[0][0] + 1)] for y in range(Boundry[1][0] - Boundry[0][0] + 1)];
    pnba.L = 2000;

    #pnba.Dsp = display_2d(Boundry);
    # use the multiprocessing module to perform the plotting activity in another process (i.e., on another core):
    #job_for_another_core = multiprocessing.Process(target=#pnba.Dsp, args=())
    #job_for_another_core.start()

    thread1 = MyThread(1, "Thread-1");
    thread2 = MyThread(2, "Thread-2");

    thread1.setParams(Boundry, start, goal, obstacle);
    thread2.setParams(Boundry,goal,start, obstacle);

    # Start new Threads
    thread1.start();
    thread2.start();

    # Add threads to thread list
    threads.append(thread1)
    threads.append(thread2)

    # Wait for all threads to complete
    for t in threads:
        t.join()
        print "Thread Length: ", len(threads)
    print "Exiting Both Thread"
    #pnba.Dsp.BlockGraph();

    print "Now Parsing Files"
    parser = TrajPub("Thread-1.txt");
    Thread1Sol = parser.parseFile();
    parser.File.close();
    print Thread1Sol;

    parser = TrajPub("Thread-2.txt");
    Thread2Sol = parser.parseFile();
    parser.File.close();
    print Thread2Sol

    print "Displaying Data"

    parser.display(Boundry,start,goal,obstacle,Thread1Sol, Thread2Sol);


    print "End"

if __name__ == "__main__":
    main()