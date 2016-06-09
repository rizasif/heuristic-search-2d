class robot:

    Motion = {
        'N'  : [0,1],
        'NE' : [1,1],
        'E'  : [1,0],
        'SE' : [1,-1],
        'S'  : [0,-1],
        'SW' : [-1,-1],
        'W'  : [-1,0],
        'NW' : [-1,1]
    };

    Boundry = [];
    Position = [];

    def __init__(self, boundry, initialPosition):
        self.Boundry = boundry;
        self.Position = initialPosition;

    def pos(self):
        return self.Position;

    def getRobotMotions(self):
        return self.Motion;

    def getRobotMotionKeys(self):
        return self.Motion.keys();

    def isInLimits(self, n):
        if (n[0] >= self.Boundry[0][0] and n[1] >= self.Boundry[0][1]):
            if (n[1] <= self.Boundry[1][0] and n[1] <= self.Boundry[1][1]):
                return True;
        return False;

    def move(self, motion_key):
        m = self.Motion[motion_key];
        temp_pos = [
            self.Position[0] + m[0],
            self.Position[1] + m[1]
        ];
        if(self.isInLimits(temp_pos)):
            self.Position = temp_pos;
        else:
            print "Robot Out of Boundry";



#Test
#bot = robot([[0,0],[20,20]], [0,0]);
#motion = bot.getRobotMotions();
#motion_keys = bot.getRobotMotionKeys();
#print "Initial Position: ", bot.pos();
#bot.move('N');
#print "New Position: ", bot.pos();