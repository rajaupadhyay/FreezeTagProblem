import sys
from shapely.geometry import Polygon

class Robot:
    def __init__(self,x,y,awake):
        self.x = x
        self.y = y
        self.awake = awake

class Obstacle:
    def __init__(self,coords):
        self.coords = coords
        self.poly = Polygon(coords)

def main(argv):
    inputfile=open(argv[0],"r")
    outputfile=argv[1]
    for line in inputfile:
        print(line)
    


#args = inputfile, outputfile
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(message)s')
    if len(sys.argv) <= 3
        sys.exit(2)
    main(sys.argv[1:])





