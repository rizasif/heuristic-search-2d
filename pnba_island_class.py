from algorithm_abstract import Algorithm
from display_grid_2d import display_2d;
from euclidian_heuristic import EucliHeur;
from print_class import PrintClass;
from traj_publisher import TrajPub;

import threading;
import multiprocessing;

class pnba_isl (Algorithm):
    Dsp = [];
    Heur = [];

    #Static
    M = [];
    L = [ [0,0,0,3000], [1,1,1,3000] ];

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
        self.Islands=[];

        self.obstacle = obstacle;

        # Riz Objects
        #pnba_isl.Dsp = display_2d(self.Boundry);
        self.Heur = EucliHeur(self.start, self.goal, self.obstacle , self.Islands);

        self.printfile = PrintClass(thread_id);

        self.ThreadID = thread_id;

        pnba_isl.L[thread_id - 1] = [goal[0], goal[1], 2000, 2000];
        print "potential islands ", pnba_isl.L[thread_id - 1];

    def isDiscovered(self, n):
        if(pnba_isl.M[n[0]][n[1]] == self.ThreadID):
            return True;
        else:
            return False;

    #For deciding from two lists
    def getNextElement(self):
        x = [];
        if (len(self.OPEN1) == 0):
            x = self.OPEN2.pop(0);
            return 2, x;
        elif (len(self.OPEN2) == 0):
            x = self.OPEN1.pop(0);
            return 1, x;
        else:
            if (self.OPEN1[0][2] > self.OPEN2[0][2]):
                x = self.OPEN2.pop(0);
                return 2, x;
            else:
                x = self.OPEN1.pop(0);
                return 1, x

    def getNext(self):
        list_name, n = self.getNextElement();
        if(pnba_isl.M[n[0]][n[1]] > 0):
            if(pnba_isl.M[n[0]][n[1]] == self.ThreadID):
                return pnba_isl.DISCOVERED, list_name, n
            elif(self.isGoal(n)):
                return pnba_isl.GOAL_FOUND, list_name, n
            elif(pnba_isl.M[n[0]][n[1]] != self.ThreadID):
                return pnba_isl.SOLUTION_FOUND, list_name, n
            else:
                return pnba_isl.FALIURE, list_name, n
        elif(pnba_isl.M[n[0]][n[1]] < 0):
            return pnba_isl.UNDISCOVERED, list_name, n
        else:
            return pnba_isl.FALIURE, list_name, n;

    def updateFCost(self, n):
        n[2] = self.Heur.getHeuristic(n) + self.Heur.getFixedCost();

    def Expand(self, n):
        expanded = []

        x = [n[0] + 1, n[1], 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        x = [n[0], n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        x = [n[0] + 1, n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1], 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        x = [n[0], n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        x = [n[0] + 1, n[1] - 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        x = [n[0] - 1, n[1] + 1, 0, 1000]
        if (self.isInLimits(x) and not self.isDiscovered(x)):
            self.updateFCost(x);
            expanded.append(x);
            #pnba_isl.Dsp.addSubPoint(x);

        if (len(expanded) == 0):
            return False, expanded
        else:
            return True, expanded

    def run(self):
        MyThread.threadLock.acquire();
        #pnba_isl.Dsp.addPoint(self.start);
        #pnba_isl.Dsp.addGoalPoint(self.goal);
        self.printfile.writeToFile(self.start);
        pnba_isl.solution_found = False;
        MyThread.threadLock.release();
        ##pnba_isl.Dsp.addIslandPoints(self.Islands);
        ##pnba_isl.Dsp.addObstaclePoints(self.obstacle);

        # step1
        self.start[2] = self.Heur.getHeuristic(self.start);
        self.OPEN1.append(self.start);
        self.Islands.append(self.goal);
        self.Heur.Islands = self.Islands;

        while(not pnba_isl.solution_found):
            if (self.isOpenEmpty()):
                print("OPEN is empty");
                pnba_isl.solution_found = False;
                break;

            t, list_name, x = self.getNext();

            if(t == pnba_isl.GOAL_FOUND):
                print "GOAL_FOUND by: ", self.ThreadID
                break;
            elif(t == pnba_isl.SOLUTION_FOUND):
                print "SOLUTION_FOUND by: ", self.ThreadID
                MyThread.threadLock.acquire();
                pnba_isl.solution_found = True;
                MyThread.threadLock.release();
                break;
            elif(t == pnba_isl.DISCOVERED):
                print "Already Discovered"
                continue;
            elif(t == pnba_isl.FALIURE):
                print "FALIURE";

            MyThread.threadLock.acquire();
            self.printfile.writeToFile(x);
            MyThread.threadLock.release();

            if(x[2] <= pnba_isl.L[2 - self.ThreadID][3]):
                MyThread.threadLock.acquire();
                self.Islands = [];
                self.Islands.append(pnba_isl.L[self.ThreadID-1]);
                self.Heur.Islands = self.Islands;
                MyThread.threadLock.release();

                succ_exist, succ = self.Expand(x);
                if (not succ_exist):
                    print("No Successor Found");
                    continue;

                for s in succ:
                    #s[3] = s[3] + self.Heur.getFixedCost();

                    #if (not self.isInList(s, self.OPEN1) and not self.isInList(s, self.OPEN2)):
                    #    d = self.Heur.getMinH(s) - self.Heur.getHeuristic(s);
                    #    if (d > 0):
                    #        s[2] = self.Heur.getMinH(s);
                    #        self.OPEN1.append(s);
                    #    else:
                    #        s[2] = self.Heur.getHeuristic(s);
                    #        self.OPEN2.append(s);

                    #elif (self.isInList(s, self.OPEN1)):
                    #    temp, element = self.getFromList(s, self.OPEN1);
                    #    self.OPEN1.pop(element);
                    #    s[2] = self.Heur.getMinH(s);
                    #    self.OPEN1.append(s);

                    #elif (self.isInList(s, self.OPEN2)):
                    #    temp, element = self.getFromList(s, self.OPEN2);
                    #    self.OPEN2.pop(element);
                    #    s[2] = self.Heur.getHeuristic(s);
                    #    self.OPEN2.append(s);

                    #If Island or from OPEN2
                    if(list_name == 2 or self.isIsland(x)):
                        if (self.isInList(s, self.OPEN1)):
                           temp, element = self.getFromList(s, self.OPEN1);
                           self.OPEN1.pop(element);
                        elif (self.isInList(s, self.OPEN2)):
                           temp, element = self.getFromList(s, self.OPEN2);
                           self.OPEN2.pop(element);
                        self.OPEN2.append(s);

                    #If not Island or from OPEN1
                    if(list_name == 1 or not self.isIsland(x)):
                       if(not self.isInList(s, self.OPEN1) and not self.isInList(s, self.OPEN2)):
                           d = self.Heur.getMinH(s) - self.Heur.getHeuristic(s);
                           if (d > 0):
                               s[2] = self.Heur.getMinH(s);
                               self.OPEN1.append(s);
                           else:
                               s[2] = self.Heur.getHeuristic(s);
                               self.OPEN2.append(s);

                       elif(self.isInList(s, self.OPEN1)):
                           temp, element = self.getFromList(s, self.OPEN1);
                           self.OPEN1.pop(element);
                           s[2] = self.Heur.getMinH(s);
                           self.OPEN1.append(s);

                       elif(self.isInList(s, self.OPEN2)):
                           temp, element = self.getFromList(s, self.OPEN2);
                           self.OPEN2.pop(element);
                           s[2] = self.Heur.getHeuristic(s);
                           self.OPEN2.append(s);

                    #---------Old code---------

                    #if(self.isInList(s,self.OPEN1)):
                    #    temp, element = self.getFromList(s, self.OPEN1);
                    #    self.OPEN1.pop(element);
                    #self.OPEN1.append(s);

                    h = self.Heur.getHeuristic(s) + self.Heur.getCost(s);
                    if(h < pnba_isl.L[2-self.ThreadID][3]):
                        print "Locking Thread on ", self.ThreadID;
                        MyThread.threadLock.acquire();
                        h = self.Heur.getHeuristic(s) + self.Heur.getCost(s);
                        if (h < pnba_isl.L[2-self.ThreadID][3]):
                            pnba_isl.L[2 - self.ThreadID] = x;
                            pnba_isl.L[2 - self.ThreadID][3] = h;
                            pnba_isl.L[self.ThreadID-1][3] = h;
                        MyThread.threadLock.release();
            else:
                print "Not Entering Main Algorithm"


            pnba_isl.M[x[0]][x[1]] = self.ThreadID;

            self.sortList(self.OPEN1);
            self.sortList(self.OPEN2);

        self.printfile.file.close();


class MyThread(threading.Thread):
    threadLock = threading.Lock();

    Algo = [];

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def setParams(self, boundry, start, goal, obstacle):
        self.Algo = pnba_isl(boundry,start,goal, obstacle, self.threadID);

    def run(self):
        self.Algo.run();


########################################
def main():
    threads = [];

    Boundry = [[0, 0], [20, 20]];

    start = [3, 0, 0, 0];  # x,y,F,g
    goal = [20, 20, 0, 20];

    obstacle = [
        [12, 12], [11, 13], [10, 14], [13, 11], [14, 10],
        [13, 13], [12, 14], [11, 15], [14, 12], [15, 11],
        [11, 11], [10, 13], [12, 10], [13, 9],
        [13, 12], [12, 13], [11, 12], [12, 11],
        [11, 14], [14, 11], [13, 10]
    ];

    pnba_isl.M = [[-1 for x in range(Boundry[1][0] - Boundry[0][0] + 1)] for y in range(Boundry[1][0] - Boundry[0][0] + 1)];

    #pnba_isl.Dsp = display_2d(Boundry);
    # use the multiprocessing module to perform the plotting activity in another process (i.e., on another core):
    #job_for_another_core = multiprocessing.Process(target=#pnba_isl.Dsp, args=())
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
    #pnba_isl.Dsp.BlockGraph();

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