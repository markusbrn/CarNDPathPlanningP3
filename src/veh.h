#ifndef VEH_H
#define VEH_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"


using namespace std;

class Vehicle {
public:

	double lane;
	double max_vel_inc;
	double vel_lim;
	double vel_cmd;

	//car state in xy and frenet coordinates
	string state;
	double x_act;
	double y_act;
	double s_act;
	double d_act;
	double yaw_act;
	double speed_act;

	// Previous path data given to the Planner
	vector<double> previous_path_x;
	vector<double> previous_path_y;
	// Previous path's end s and d values
	double end_path_s;
	double end_path_d;
	// Next path data for the Planner
	vector<double> next_path_x;
	vector<double> next_path_y;

	Eigen::VectorXd state_vector;
	vector< vector<double> > mpc_vars;
	MPC mpc;
	vector<double> mpc_path_x;
	vector<double> mpc_path_y;

	/**
	* Constructor
	*/
	Vehicle();

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	/**
	 * Member Functions
	 */
	void choose_next_state(const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	vector<string> successor_states();

	void generate_trajectory(string state, const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	void keep_lane_trajectory(const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	void predict(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	void MPC_plan(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	void JMT(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

};


#endif
