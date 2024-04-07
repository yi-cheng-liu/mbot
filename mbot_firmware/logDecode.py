import sys
import lcm
from mbot_lcm_msgs.pose2D_t import pose2D_t
import numpy as np
log = lcm.EventLog("/home/mbot/mbot_ws/mbot_example_logs/botlab/accTest", "r")
X=[]
Y=[]
T=[]
Theta=[]

for event in log:
	if event.channel == "GROUND_TRUTH_POSE":
		msg = pose2D_t.decode(event.data)
		# print("Message:")
		# print(" timestamp = %s" % str(msg.utime))
		# print(" x = %s" % str(msg.x))
		# print(" y = %s" % str(msg.y))
		# print(" theta: %s" % str(msg.theta))
		# print("")
		X.append(msg.x)
		Y.append(msg.y)
		T.append(msg.utime)
		Theta.append(msg.theta)
X=np.array(X)
Y=np.array(Y)
T=np.array(T)
Theta=np.array(Theta)
np.savetxt('/home/mbot/mbot_ws/mbot_autonomy/checkpoint2/GTAcc/x.txt',X)
np.savetxt('/home/mbot/mbot_ws/mbot_autonomy/checkpoint2/GTAcc/y.txt',Y)
np.savetxt('/home/mbot/mbot_ws/mbot_autonomy/checkpoint2/GTAcc/t.txt',T)
np.savetxt('/home/mbot/mbot_ws/mbot_autonomy/checkpoint2/GTAcc/theta.txt',Theta)


