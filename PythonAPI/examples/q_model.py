import csv
import pandas as pd
import numpy as np
import sys
import os

try:
    sys.path.append(os.path.abspath('../../carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg'))
except IndexError:
    print("no carla")
    pass

import carla

waypoint_file = './waypoint_data0.csv'


class Agent(object):
	def __init__(self, waypoint_file, world, numEpi=100, alpha=0.5, gamma=1, epsilon = 0.1):
		self.data = pd.read_csv(waypoint_file, sep=',', header=0).to_numpy()
		self.id_list = self.data[:,0]
		self.x_list = self.data[:,1]
		self.y_list = self.data[:,2]
		self.z_list = self.data[:,3]
		self.yaw_list = self.data[:,4]
		self.visited_list = np.zeros_like(self.x_list, dtype=bool)
		self.world = world
		self.map = world.world.get_map()
		self.window_size = 5
		self.iter1 = 0
		self.iter2 = self.iter1 + self.window_size
		self.state_min_bounds = np.array([-10,-10,-10,-10,-10,-10,-10,-10,-10])
		self.state_max_bounds = np.array([10,10,10,10,10,10,10,10,10])
		self.state_step_size = np.array([5]*9)
		self.S = np.ones(len(self.state_step_size), dtype=int)
		self.A = 6
		for i in range(1,len(self.state_step_size)):
			self.S[i] = int((self.state_max_bounds[i-1]-self.state_min_bounds[i-1])/self.state_step_size[i-1]) * self.S[i-1]
		self.Q = np.zeros((self.S[-1]*int((self.state_max_bounds[-1]-self.state_min_bounds[-1])/self.state_step_size[-1]),self.A))
		self.numEpi = numEpi
		self.gamma = gamma
		self.alpha = alpha
		self.state0 = np.zeros(9)
		self.steps = 0
		self.epsilon = epsilon

	def get_closest_index(self, w_car):
		dist = np.zeros([self.window_size, 1])
		for i in range(self.iter1, min(self.iter2,self.data.shape[0])):
			ww = self.map.get_waypoint(carla.Location(self.x_list[i], self.y_list[i], self.z_list[i]))
			dist[i-self.iter1] = ww.transform.location.distance(w_car.transform.location)
		S_i = np.argmin(dist)
		if(S_i==0):
			nearest_index = self.iter1
		else:
			nearest_index = S_i + self.iter1
			self.iter1 += 1
			self.iter2 += 1
		return nearest_index


	def get_states(self, w_car):
		w_index = self.get_closest_index(w_car)

		xw_c = self.x_list[w_index] - w_car.transform.location.x
		yw_c = self.y_list[w_index] - w_car.transform.location.y
		vel_cx = self.world.player.get_velocity().x
		vel_cy = self.world.player.get_velocity().y
		yaww_c = self.yaw_list[w_index] - w_car.transform.rotation.yaw

		y10 = self.yaw_list[min(w_index+10, self.data.shape[0])] - self.yaw_list[w_index]
		y20 = self.yaw_list[min(w_index+20, self.data.shape[0])] - self.yaw_list[w_index]
		throttle = self.world.player.get_control().throttle
		steer = self.world.player.get_control().steer

		return (xw_c, yw_c, vel_cx, vel_cy, yaww_c, y10, y20, throttle, steer)

# (2.3,4.56,0.5,0.3,0.8,0.1,0.1)

	def state2disc(self, state):
		state_min_bounds = np.array([-10,-10,-10,-10,-10,-10,-10,-10,-10])
		state_max_bounds = np.array([10,10,10,10,10,10,10,10,10])
		state_step_size = np.array([5]*9)
		stateVector = np.floor(np.divide((np.clip(state, -10, 10) - state_min_bounds),state_step_size))
		index = self.S.T@stateVector
		return int(index)

	def discretize_states(self,w_car):
		state = self.get_states(w_car)
		state_min_bounds = np.array([-10,-10,-10,-10,-10,-10,-10,-10,-10])
		state_max_bounds = np.array([10,10,10,10,10,10,10,10,10])
		state_step_size = np.array([5]*9)
		stateVector = np.floor(np.divide((np.clip(state, -10, 10) - state_min_bounds),state_step_size))
		index = self.S.T@stateVector
		return int(index)


	def getAction(self,w_car):
		s = self.discretize_states(w_car)
		return np.argmax(self.Q[s]) if np.random.random()>self.epsilon else np.random.choice(range(self.A))

	def Qupdate(self, s, a, s_):
		self.steps += 1
		r = -100*(s_[0]**2 + s_[1]**2) - 500*(s_[4]**2) - ((s_[5]-s_[4])**2 + (s_[6]-s_[4])**2) - 0.5*(s_[7]**2) - 0.5*(s_[8]**2)
		# 
		self.Q[self.state2disc(s),a] += self.alpha*(r + self.gamma*np.max(self.Q[self.state2disc(s_)]) - self.Q[self.state2disc(s),a])
		# t += 1
		if(self.steps>10000):
			return -1
		else:
			return 0
				

		



def main():
	# mapWaypoints = MapWaypoints(waypoint_file, )
	print("yo")



if __name__ == '__main__':
	main()