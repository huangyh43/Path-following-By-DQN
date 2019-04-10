import random
import math
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

# Deep Reinforcement Learning
# From https://github.com/keon/deep-q-learning
# A minimal Deep Q-Learning (DQN)
class DQNAgent:
	def __init__(self, stateSize, actionSize):
		self.state_size = stateSize
		self.action_size = actionSize
		self.memory = deque(maxlen=2000)
		self.gamma = 0.95    # discount rate
		self.epsilon = 1.0  # exploration rate
		self.epsilon_min = 1e-4
		self.epsilon_decay = 0.995
		self.epsilon_back_step = math.pow(self.epsilon_decay, 100)
		print(' ---> ', self.epsilon_back_step)
		self.learning_rate = 0.001
		self.model = self._build_model()

	def _build_model(self):
		# Neural Net for Deep-Q learning Model
		model = Sequential()
		model.add(Dense(24, input_dim=self.state_size, activation='relu'))
		model.add(Dense(24, activation='relu'))
		model.add(Dense(self.action_size, activation='linear'))
		model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
		return model

	def remember(self, state, action, reward, next_state, done):
		self.memory.append((state, action, reward, next_state, done))

	def act(self, state):
		if np.random.rand() <= self.epsilon:
			return random.randrange(self.action_size)
		act_values = self.model.predict(state)
		return np.argmax(act_values[0])  # returns action
	# 1 --> Toward decay
	# 0 --> Backward
	# 2 --> Go back several step
	def epsilon_update(self, flag):
		if flag == 1:
			if self.epsilon > self.epsilon_min:
				self.epsilon *= self.epsilon_decay
		elif flag == 0:
			#self.epsilon = math.pow(self.epsilon, 1/self.epsilon_back_step)
			self.epsilon /= self.epsilon_decay
		else:
			self.epsilon /= self.epsilon_back_step
	def load(self, name):
		self.model.load_weights(name)

	def save(self, name):
		self.model.save_weights(name)