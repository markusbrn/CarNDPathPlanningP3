#include <algorithm>
#include <iostream>
#include "veh.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"
//#include "cost.h"

#include "assist.h"

double lanes_available = 3;

double bound(double min, double max, double in) {
	double out=in;

	if(out<min) out=min;
	else if(out>max) out=max;

	return out;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


/**
* Initializes Vehicle
*/

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::choose_next_state(const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	float cost = pow(10,7);
	vector<string> s_states = this->successor_states();
	for(vector<string>::iterator it=s_states.begin(); it!=s_states.end(); ++it)	{
		this->generate_trajectory(*it, predictions, maps_s, maps_x, maps_y);
		//TODO: compare trajectory costs
	}
}


vector<string> Vehicle::successor_states() {
	/*
	Provides the possible next states given the current state for the FSM
	discussed in the course, with the exception that lane changes happen
	instantaneously, so LCL and LCR can only transition back to KL.
	*/
	vector<string> states;
	states.push_back("KL");
//	string state = this->state;
//	if (state.compare("KL") == 0) {
//		states.push_back("PLCL");
//		states.push_back("PLCR");
//	}
//	else if (state.compare("PLCL") == 0) {
//		if (lane != lanes_available - 1) {
//			states.push_back("PLCL");
//			states.push_back("LCL");
//		}
//	}
//	else if (state.compare("PLCR") == 0) {
//		if (lane != 0) {
//			states.push_back("PLCR");
//			states.push_back("LCR");
//		}
//	}
	//If state is "LCL" or "LCR", then just return "KL"
	return states;
}


void Vehicle::generate_trajectory(string state, const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	/*
	Given a possible next state, generate the appropriate trajectory to realize the next state.
	*/
	/*if (state.compare("CS") == 0) {
		trajectory = constant_speed_trajectory();
	}*/
	if (state.compare("KL") == 0) {
		keep_lane_trajectory(predictions, maps_s, maps_x, maps_y);
	}
	/*else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
		trajectory = lane_change_trajectory(state, predictions);
	}
	else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
		trajectory = prep_lane_change_trajectory(state, predictions);
	}*/
}


void Vehicle::keep_lane_trajectory(const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	/*
	Generate a keep lane trajectory.
	*/
	bool too_close;
	for(unsigned int i=0; i<predictions.size(); i++) {
		if((predictions[i].d_act > 2+4*lane-2) && (predictions[i].d_act < 2+4*lane+2)) {
			double check_car_s = predictions[i].s_act + previous_path_x.size()*0.02*predictions[i].speed_act;
			if((check_car_s > end_path_s) && (check_car_s-end_path_s < 30)) {
			 	too_close = true;
			}
		}
	}
	//if(too_close) vel_cmd -= max_vel_inc;
	//else if(vel_cmd < vel_lim) vel_cmd += max_vel_inc;

	//this->JMT(maps_s, maps_x, maps_y);
	/*
	* Calculate new trajectory using MPC.
	*/
	this->MPC_plan(maps_s, maps_x, maps_y);
}


void Vehicle::predict(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	//define containers for path planning polynomial
	vector<double> ptsx;
	vector<double> ptsy;

	//generate waypoints in 30, 60 and 90 meters distance from last already planned car position
	vector<double> next_wp0 = getXY(s_act+30, d_act, maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(s_act+60, d_act, maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(s_act+90, d_act, maps_s, maps_x, maps_y);

	//add waypoints to ptsx,y containers
	ptsx.push_back(x_act);
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsy.push_back(y_act);
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	//transform ptsx,y from global to vehicle fixed KOS (makes spline interpolation easier)
	for(unsigned int i=0; i<ptsx.size(); i++) {
		double shift_x = ptsx[i] - x_act;
		double shift_y = ptsy[i] - y_act;

		ptsx[i] = shift_x * cos(-yaw_act) - shift_y * sin(-yaw_act);
		ptsy[i] = shift_x * sin(-yaw_act) + shift_y * cos(-yaw_act);
	}

	//compute spline interpolation through ptsx,y
	tk::spline s;
	s.set_points(ptsx,ptsy);

	vector< vector<double> > next_path_xy;

	//define a path made up of (x,y) points that the car will visit sequentially every .02 seconds; look ahead for 1s
	next_path_x.clear();
	next_path_y.clear();

	double pos_inc = speed_act * 0.02;
	if(pos_inc < 1.e-5) pos_inc = 1.e-5;

	double x_point = 0;
	double y_point = 0;
	for(unsigned i=0; i<=50; i++) {
		x_point += pos_inc; //increment x-position
		y_point = s(x_point); //compute y-position with spline function

		//derotate to global KOS and push on next_xy_vals containers
		next_path_x.push_back(x_act + x_point*cos(yaw_act) - y_point*sin(yaw_act));
		next_path_y.push_back(y_act + x_point*sin(yaw_act) + y_point*cos(yaw_act));
	}
}


void Vehicle::MPC_plan(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	double ref_x = 0.;
	double ref_y = 0.;
	double ref_s = 0;
	double ref_yaw = 0.;
	for(unsigned int i=0; i<previous_path_x.size(); i++) {
			cout<<"previous_path_x: "<<previous_path_x[i]<<endl;
	}
	if(previous_path_x.empty() == false) {
		for(unsigned int i=0; i<mpc_path_x.size(); i++) {
			if(fabs(int(mpc_path_x[i]*100) - int(previous_path_x[previous_path_x.size()-1]*100))<2) {
				cout << "i: " << i << endl;
				this->state_vector << mpc_vars[0][i-1], mpc_vars[1][i-1], mpc_vars[2][i-1], mpc_vars[3][i-1], mpc_vars[4][i-1], mpc_vars[5][i-1];
				cout << "x0: " << state_vector[0] << endl;
				cout << "y0: " << state_vector[1] << endl;
				cout << "psi0: " << state_vector[2] << endl;
				cout << "v0: " << state_vector[3] << endl;
				cout << "cte0: " << state_vector[4] << endl;
				cout << "epsi0: " << state_vector[5] << endl;
			}
		}
		ref_x = previous_path_x[previous_path_x.size()-1];
		ref_y = previous_path_y[previous_path_y.size()-1];
		ref_s = end_path_s;
		ref_yaw = yaw_act + state_vector[2];
		cout << "ref_yaw: " << ref_yaw << endl;
	} else {
		this->state_vector << 0., 0., 0., this->speed_act, 0., 0.;
		ref_x = x_act;
		ref_y = y_act;
		ref_s = s_act;
		ref_yaw = yaw_act;
	}

	//define containers for path planning polynomial
	vector<double> ptsx;
	vector<double> ptsy;

	//generate waypoints in 30, 60 and 90 meters distance from last already planned car position
	vector<double> next_wp0 = getXY(ref_s+30, 2+4*this->lane, maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(ref_s+60, 2+4*this->lane, maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(ref_s+90, 2+4*this->lane, maps_s, maps_x, maps_y);

	//add waypoints to ptsx,y containers
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	//transform ptsx,y from global to vehicle fixed KOS (makes spline interpolation easier)
	for(unsigned int i=0; i<ptsx.size(); i++) {
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] =  shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw);
		ptsy[i] = -shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw);
	}

	// fit polynomial through trajectory points
	Eigen::VectorXd xvals = Eigen::VectorXd::Map(ptsx.data(),ptsx.size());
	Eigen::VectorXd yvals = Eigen::VectorXd::Map(ptsy.data(),ptsy.size());
	auto coeffs = polyfit(xvals,yvals,2);


	mpc_vars = this->mpc.Solve(state_vector, coeffs);
	vector<double> x = mpc_vars[0];
	vector<double> y = mpc_vars[1];
	vector<double> psi = mpc_vars[2];
	vector<double> v = mpc_vars[3];
	vector<double> cte = mpc_vars[4];
	vector<double> epsi = mpc_vars[5];


	next_path_x.clear();
	next_path_y.clear();
	mpc_path_x.clear();
	mpc_path_y.clear();
	for(unsigned int i=0; i<previous_path_x.size();i++) {
		next_path_x.push_back(previous_path_x[i]);
		next_path_y.push_back(previous_path_y[i]);
	}
	for(unsigned i=0; i<=15-previous_path_x.size(); i++) {
		//derotate to global KOS and push on next_xy_vals containers
		mpc_path_x.push_back(ref_x + mpc_vars[0][i]*cos(ref_yaw) - mpc_vars[1][i]*sin(ref_yaw));
		mpc_path_y.push_back(ref_y + mpc_vars[0][i]*sin(ref_yaw) + mpc_vars[1][i]*cos(ref_yaw));
		next_path_x.push_back(ref_x + mpc_vars[0][i]*cos(ref_yaw) - mpc_vars[1][i]*sin(ref_yaw));
		next_path_y.push_back(ref_y + mpc_vars[0][i]*sin(ref_yaw) + mpc_vars[1][i]*cos(ref_yaw));
	}
	for(unsigned int i=0; i<next_path_x.size(); i++) {
			cout<<"next_path_x: "<<next_path_x[i]<<endl;
	}
}


void Vehicle::JMT(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	//define containers for path planning polynomial
	vector<double> ptsx;
	vector<double> ptsy;

	//store current car position and yaw angle; (this is only used if no previously planned path exists; otherwise the last position from the
	//previously planned path is used for the reference position
	double ref_x = x_act;
	double ref_y = y_act;
	double ref_yaw = yaw_act;
	double prev_size = previous_path_x.size();
	double ref_s = s_act;
	if(prev_size > 0) ref_s = end_path_s;

	//initialize ptsx,y with last two positions from previously planned path (if available) or generate previous car position
	//from actual position and yaw angle
	if(prev_size <2) {
		double ref_x_prev = ref_x - cos(ref_yaw);
		double ref_y_prev = ref_y - sin(ref_yaw);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	} else {
		ref_x = this->previous_path_x[prev_size-1];
		ref_y = this->previous_path_y[prev_size-1];

		double ref_x_prev = this->previous_path_x[prev_size-2];
		double ref_y_prev = this->previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	//generate waypoints in 30, 60 and 90 meters distance from last already planned car position
	vector<double> next_wp0 = getXY(ref_s+30, 2+4*this->lane, maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(ref_s+60, 2+4*this->lane, maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(ref_s+90, 2+4*this->lane, maps_s, maps_x, maps_y);

	//add waypoints to ptsx,y containers
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	//transform ptsx,y from global to vehicle fixed KOS (makes spline interpolation easier)
	for(unsigned int i=0; i<ptsx.size(); i++) {
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
		ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
	}

	//compute spline interpolation through ptsx,y
	tk::spline s;
	s.set_points(ptsx,ptsy);

	vector< vector<double> > next_path_xy;

	//define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

	//push already planned path in next_xy_vals containers
	next_path_x.clear();
	next_path_y.clear();
	for(unsigned int i=0; i<previous_path_x.size();i++) {
		next_path_x.push_back(previous_path_x[i]);
		next_path_y.push_back(previous_path_y[i]);
	}

	double target_x = 30.; //set lookahead distance (in vehicle fixed KOS! x-direction) to 30m
	double target_y = s(target_x); //compute y values with previously computed spline
	double target_dist = sqrt(pow(target_x,2)+pow(target_y,2)); //compute actual distance

	double x_point = 0;
	double y_point = 0;

	for(unsigned i=0; i<=50-previous_path_x.size(); i++) {
		double N = (target_dist/(0.02*vel_cmd)); //compute number of position increments to cover target_dist when driving with vel_cmd
		x_point += target_x/N; //increment x-position
		y_point = s(x_point); //compute y-position with spline function

		//derotate to global KOS and push on next_xy_vals containers
		next_path_x.push_back(ref_x + x_point*cos(ref_yaw) - y_point*sin(ref_yaw));
		next_path_y.push_back(ref_y + x_point*sin(ref_yaw) + y_point*cos(ref_yaw));
	}
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Vehicle::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}
