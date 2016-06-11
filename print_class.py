from datetime import datetime

class PrintClass:

    file = None;

    def __init__(self, file_name):
        self.file = open("Thread-" + str(file_name) + ".txt", "wb")

    def writeToFile(self, data):
        now = datetime.now()
        seconds_since_midnight = (now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()
        self.file.write(str(data[0]) + " " + str(data[1]) + " " + str(seconds_since_midnight) + " ");
        self.file.write("\n");