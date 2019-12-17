from robolink.robolink import *  # API to communicate with RoboDK
from robodk.robodk import *  # robodk robotics toolbox
import time
import mailbox
#import MainProg

RDK = Robolink()
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception('No robot selected or available')


target3 = RDK.AddTarget('Home')
target3.setAsCartesianTarget()
target3.setPose(KUKA_2_Pose([1344.039,-31.142,410.631,-87.726,-0.481,0.737]))

tool_name1 = 'Scanner'
tool_name2 = 'PIT'
tool1 = RDK.Item(tool_name1, ITEM_TYPE_TOOL)
tool2 = RDK.Item(tool_name2, ITEM_TYPE_TOOL)
target1 = RDK.AddTarget('Start Target')
target1.setAsCartesianTarget()
target1.setPose(KUKA_2_Pose([1465.532,-10.148,309.727,-89.357,-0.213,2.691]))
target2 = RDK.AddTarget('End Target')
target2.setAsCartesianTarget()
target2.setPose(KUKA_2_Pose([1520.009,-9.527,312.287,-89.357,-0.213,2.691]))





robot.setTool(MainProg.tool1)
