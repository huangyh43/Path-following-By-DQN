import sys
sys.path.append ('./VrepApi')
import vrep

from environment import *
from DQNAgent import *
import time
import random


def replay(batch_size):
	return

if __name__ == "__main__":
	# Intialization of the connector to V-Rep simulator
	clientID=vrep.simxStart("127.0.0.1",19997,1,1,2000,5);
	res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
	if clientID > -1:
		print ("Connect to Remote API server!")
	else:
		print ('Failed connecting to remote API server')
		sys.exit()

	env = environment()
	env.reset(clientID)

	for i in range(10):
		env.setCtrl(INIT_CORR_NUM)
		time.sleep(1)
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)