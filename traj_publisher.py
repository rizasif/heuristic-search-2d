from print_class import PrintClass;
from display_grid_2d import display_2d;


class TrajPub:

    Dsp = None;
    File = None;

    def __init__(self, filename):

        self.File = open(filename, 'r');

    def parseFile(self):
        print ("Parsing Trajectory File")
        ans = []
        for line in self.File:
            digit = ""
            array = []
            for i in range(len(line)):
                if (line[i] != " "):
                    digit = digit + line[i]
                else:
                    num = float(digit)
                    array.append(num)
                    digit = ""
            ans.append(array)

        return ans

    def display(self, Boundry, start, goal, obstacle, Thread1Sol, Thread2Sol):
        Dsp = display_2d(Boundry);
        Dsp.addPoint(start);
        Dsp.addPoint(goal);
        Dsp.addObstaclePoints(obstacle);

        sol1 = Thread1Sol;
        sol2 = Thread2Sol;

        total = len(sol1) + len(sol2);
        x1 = sol1.pop(0);
        x2 = sol2.pop(0);
        for i in range(total):
            if(x1[2] < x2[2]):
                Dsp.addCustomPoint(x1, 'bs', 15);
                if(len(sol1)>0):
                    x1 = sol1.pop(0);
                else:
                    x1[2] = 90000;
            elif(x2[2] <= x1[2]):
                Dsp.addCustomPoint(x2, 'gs', 15);
                if (len(sol2) > 0):
                    x2 = sol2.pop(0);
                else:
                    x2[2] = 90000;

            if(x1[2] == 90000 and x2[2] == 90000):
                break;

        Dsp.BlockGraph();

#Test
#parser = TrajPub("Thread-1.txt");
#ans = parser.parseFile();
#parser.File.close();
#print ans