import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

class display_2d:
    'Class to display the 2d grid on matplotlib'

    fig, ax = plt.subplots();

    def __init__(self, boundry):
        X = [x for x in range(boundry[1][0] - boundry[0][0])];
        Y = [0 for y in range(boundry[1][1] - boundry[0][1])];

        self.ax.plot(X, Y);
        self.ax.plot(Y, X);

        spacing = 1.0;  # This can be your user specified spacing.
        minorLocator = MultipleLocator(spacing);

        self.ax.yaxis.set_minor_locator(minorLocator);
        self.ax.xaxis.set_minor_locator(minorLocator);

        self.ax.grid(True);
        self.ax.grid(which='minor');

        plt.show(block=False);

    def addCustomPoint(self, point, marker, markerSize):
        self.ax.plot(point[0] - 0.5, point[1] - 0.5, marker, markersize=markerSize);
        plt.draw()
        plt.show(block=False);

    def addPoint(self, point):
        self.ax.plot(point[0] - 0.5, point[1] - 0.5, 'rs', markersize=15);
        plt.draw()
        plt.show(block=False);

    def addSubPoint(self, point):
        self.ax.plot(point[0] - 0.5, point[1] - 0.5, 'yo', markersize=8);
        plt.draw()
        plt.show(block=False);

    def addIslandPoint(self, point):
        self.ax.plot(point[0] - 0.5, point[1] - 0.5, 'g^', markersize=15);
        plt.draw()
        plt.show(block=False);

    def addGoalPoint(self, point):
        self.ax.plot(point[0] - 0.5, point[1] - 0.5, 'bs', markersize=15);
        plt.draw()
        plt.show(block=False);

    def addObstaclePoints(self, l):
        for point in l:
            self.ax.plot(point[0] - 0.5, point[1] - 0.5, 'x', markersize=15);
            plt.draw()
            plt.show(block=False);

    def BlockGraph(self):
        plt.show();


#test
#dsp = display_2d([[0,0], [20,20]]);
#dsp.addPoint([15,15]);
#dsp.addPoint([18,18]);