import sys
sys.path.append ('./VrepApi')
import vrep

from environment import *
from DQNAgent import *
import time
import random

EPISODES = 1000
TRAIN_STEP = 200
TEST_STEP = 50
# Using Batch Size to Q-learning
# Without learning in full dataset, 
# the time cost of learning process is controlled
def replay(batch_size, theAgent):
	minibatch = random.sample(theAgent.memory, batch_size)
	for state, action, reward, next_state, done in minibatch:
		target = reward
		if not done:
			target = (reward + theAgent.gamma *
				np.amax(theAgent.model.predict(next_state)[0]))
			target_f = theAgent.model.predict(state)
			target_f[0][action] = target
			theAgent.model.fit(state, target_f, epochs=1, verbose=0)
		if reward > 2:
			theAgent.epsilon_update(1)
		else:
			theAgent.epsilon_update(0)
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
	#Initializing the Robot information
	#Initializing the Learning Agent for DQN
	env = environment(10, 10) #Block number of linear velocity and the grad of angular velocity
	action_num = env.vStateNum * env.aStateNum
	states_num = len(env.getState())
	print(action_num, ' --- ', states_num)
	agent = DQNAgent(states_num, action_num)
	# Start Training
	done = False
	batch_size = 32
	for e in range(EPISODES):
		print("------------------------->  ", e)
		print(agent.epsilon)
		env.reset(clientID)
		env.setCtrl(INIT_CORR_NUM)
		time.sleep(1)
		# Collecting the status information of  mobile robot
		tState = env.getState()
		tState = np.reshape(tState, [1, states_num])
		# Produce the action ID for a robot status information
		action = agent.act(tState)
		for tt in range(TRAIN_STEP):
			# Using the produced action to control the robot
			# Collecting the learning information after controlling
			print(action)
			next_state, reward, done = env.step(action)
			reward = reward if not done else -3*MAX_DEVIATION
			next_state = np.reshape(next_state, [1, states_num])
			# Information Storing
			agent.remember(tState, action, reward, next_state, done)
			tState = next_state
			# “done == 1" means the task execution is failed
			if done:
				#print(agent.epsilon, end=' --- ')
				agent.epsilon_update(2)
				#print(agent.epsilon)
				print("episode: {}/{}, score: {}, e: {:.2}"
					.format(e, EPISODES, time, agent.epsilon))
				vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
				break
			if len(agent.memory) > batch_size:
				#print(agent.epsilon, end="  ---> ")
				replay(batch_size, agent)
				#print(agent.epsilon)
			time.sleep(0.05)
			if tt%20 == 0:
				action = agent.act(tState)		#??? may delete
	agent.save("./save/dqn_mTT2_121.h5")
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
	time.sleep(1)