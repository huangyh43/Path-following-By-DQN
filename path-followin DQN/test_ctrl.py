import sys
sys.path.append ('./VrepApi')
import vrep

from environment import *
from DQNAgent import *
import time
import random

TEST_STEP = 3000


if __name__ == "__main__":
	# Intialization of the connector to V-Rep simulator
	clientID=vrep.simxStart("127.0.0.1",19997,1,1,2000,5);
	res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
	if clientID > -1:
		print ("Connect to Remote API server!")
	else:
		print ('Failed connecting to remote API server')
		sys.exit()

	env = environment(10, 10)
	action_num = env.vStateNum * env.aStateNum
	states_num = len(env.getState())
	print(action_num, ' --- ', states_num)
	agent = DQNAgent(states_num, action_num)
	agent.load("./save/dqn_mTT_121.h5")
	done = False
	
	env.reset(clientID)
	env.setCtrl(INIT_CORR_NUM)
	time.sleep(0.1)
	# Collecting the status information of  mobile robot
	# Produce the action ID for a robot status information to control robot
	tState = env.getState()
	tState = np.reshape(tState, [1, states_num])
	action = agent.act(tState)
	for tt in range(TEST_STEP):
		print ("Action --> ", action)
		next_state, reward, done = env.step(action)
		tState = next_state
		if done:
			print("Sorr to work faild")
			vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
		action = agent.act(tState)
		time.sleep(0.05)
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)