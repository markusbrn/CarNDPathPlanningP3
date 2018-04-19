#ifndef VEH_H
#define VEH_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"


using namespace std;

double bound(double min, double max, double in);

double polyeval(Eigen::VectorXd coeffs, double x);

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

class Vehicle {
public:

	double lane;
	double lane_cmd;
	double vel_cmd;
	double vel_max;
	double vel_inc_max;
	double vel_inc;

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
	vector<double> cand_path_x;
	vector<double> cand_path_y;

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
	bool check_collision(const vector<Vehicle> &predictions, const vector<double> lanes, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	void choose_next_state(const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	void predict(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

	void JMT(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

};


#endif
