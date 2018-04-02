#ifndef VEH_H
#define VEH_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

	double lane;
	double max_vel_inc;
	double vel_lim;
	double vel_cmd;

	//car state in xy and frenet coordinates
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
	void JMT(vector<double> &next_path_x, vector<double> &next_path_y, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

};

#endif
