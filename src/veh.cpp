#include <algorithm>
#include <iostream>
#include "veh.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"
//#include "cost.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
* Initializes Vehicle
*/

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::JMT(vector<double> &next_path_x, vector<double> &next_path_y, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	//define containers for path planning polynomial
	vector<double> ptsx;
	vector<double> ptsy;

	//store current car position and yaw angle; (this is only used if no previously planned path exists; otherwise the last position from the
	//previously planned path is used for the reference position
	double ref_x = x_act;
	double ref_y = y_act;
	double ref_yaw = deg2rad(yaw_act);
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
	for(unsigned int i=0; i<this->previous_path_x.size();i++) {
		next_path_x.push_back(this->previous_path_x[i]);
		next_path_y.push_back(this->previous_path_y[i]);
	}

	double target_x = 30.; //set lookahead distance (in vehicle fixed KOS! x-direction) to 30m
	double target_y = s(target_x); //compute y values with previously computed spline
	double target_dist = sqrt(pow(target_x,2)+pow(target_y,2)); //compute actual distance

	double x_point = 0;
	double y_point = 0;

	for(unsigned i=0; i<=50-this->previous_path_x.size(); i++) {
		double N = (target_dist/(0.02*vel_cmd)); //compute position increment during 20ms time interval when driving with vel_cmd
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

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}
