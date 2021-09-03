#!/usr/bin/env python

from ptg import PTG
from helpers import Vehicle, show_trajectory



def main():
	vehicle = Vehicle([0,10,0, 0,0,0])
	predictions = {0: vehicle}
	target = 0  # is the id of the predictions cars,here is just one car
	delta = [-10, 0, 0, 0, 0, 0] # the delta value of [s, s_dot, s_ddot, d, d_dot, d_ddot]
	start_s = [10, 10, 0] # [s, s_dot, s_ddot
	start_d = [4, 0, 0]
	T = 5.0  # T - the duration of maneuver in seconds.
	best = PTG(start_s, start_d, target, delta, T, predictions)
	show_trajectory(best[0], best[1], best[2], vehicle)

if __name__ == "__main__":
	main()