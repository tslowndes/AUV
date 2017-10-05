import sys
sys.path.insert(0,'../gen')
from latlon_util import find_dir

pos1 = (-1.362773, 60.391747)
pos2 = (-1.3,60.391747)
print(find_dir(pos1,pos2))