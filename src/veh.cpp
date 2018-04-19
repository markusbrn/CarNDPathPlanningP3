#include <algorithm>
#include <iostream>
#include "veh.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"

#include "assist.h"

double lanes_available = 3;

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
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

bool Vehicle::check_collision(const vector<Vehicle> &predictions, const vector<double> lanes, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	bool collision = false;
	//check for all vehicles in specific lanes
	for(unsigned int i=0; i<predictions.size(); i++) {
		for(unsigned int j=0; j<lanes.size(); j++) {
			if((predictions[i].lane == lanes[j]) && (0 < fabs(predictions[i].s_act-s_act) < 50)) {
				for(unsigned int k=0; k<cand_path_x.size(); k++) {
					double pos_cand = sqrt(pow(cand_path_x[k],2) + pow(cand_path_y[k],2));
					double pos_pred = sqrt(pow(predictions[i].next_path_x[k],2) + pow(predictions[i].next_path_y[k],2));
					if(fabs(pos_cand - pos_pred) < 10 ) {
						collision = true;
					}
				}
			}

		}
	}

	return collision;
}

void Vehicle::choose_next_state(const vector<Vehicle> &predictions, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	double vel_save = vel_cmd;
	bool found = false;
	if(lane_cmd == lane) {
		vector<double> lanes = {lane_cmd};
		vel_cmd = vel_save;
		vel_inc = vel_inc_max;
		this->JMT(maps_s, maps_x, maps_y);
		//check if collision for KL
		if(!check_collision(predictions, lanes, maps_s, maps_x, maps_y)) {
			found = true;
			next_path_x.clear();
			next_path_y.clear();
			for(unsigned int i=0; i<cand_path_x.size(); i++) {
				next_path_x.push_back(cand_path_x[i]);
				next_path_y.push_back(cand_path_y[i]);
			}
		}
		//if collision try LCL (only if not in leftmost lane)
		if(lane != 0 && found == false) {
			lane_cmd = lane-1;
			vector<double> lanes = {lane_cmd};
			vel_cmd = vel_save;
			vel_inc = 0.; //keep velocity during lane change
			this->JMT(maps_s, maps_x, maps_y);
			if(!check_collision(predictions, lanes, maps_s, maps_x, maps_y)) {
				found = true;
				next_path_x.clear();
				next_path_y.clear();
				for(unsigned int i=0; i<cand_path_x.size(); i++) {
					next_path_x.push_back(cand_path_x[i]);
					next_path_y.push_back(cand_path_y[i]);
				}
			}
		}
		if(lane != 2 && found == false) {
			lane_cmd = lane+1;
			vector<double> lanes = {lane_cmd};
			vel_cmd = vel_save;
			vel_inc = 0.; //keep velocity during lane change
			this->JMT(maps_s, maps_x, maps_y);
			if(!check_collision(predictions, lanes, maps_s, maps_x, maps_y)) {
				found = true;
				next_path_x.clear();
				next_path_y.clear();
				for(unsigned int i=0; i<cand_path_x.size(); i++) {
					next_path_x.push_back(cand_path_x[i]);
					next_path_y.push_back(cand_path_y[i]);
				}
			}
		}
		if(found == false) {
			lane_cmd = lane;
			vel_cmd = vel_save;
			vel_inc = -vel_inc_max; //keep velocity during lane change
			this->JMT(maps_s, maps_x, maps_y);
			next_path_x.clear();
			next_path_y.clear();
			for(unsigned int i=0; i<cand_path_x.size(); i++) {
				next_path_x.push_back(cand_path_x[i]);
				next_path_y.push_back(cand_path_y[i]);
			}
		}
	} else {
		this->JMT(maps_s, maps_x, maps_y);
		next_path_x.clear();
		next_path_y.clear();
		for(unsigned int i=0; i<cand_path_x.size(); i++) {
			next_path_x.push_back(cand_path_x[i]);
			next_path_y.push_back(cand_path_y[i]);
		}
	}
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
	for(unsigned i=0; i<=80; i++) {
		x_point += pos_inc; //increment x-position
		y_point = s(x_point); //compute y-position with spline function

		//derotate to global KOS and push on next_xy_vals containers
		next_path_x.push_back(x_act + x_point*cos(yaw_act) - y_point*sin(yaw_act));
		next_path_y.push_back(y_act + x_point*sin(yaw_act) + y_point*cos(yaw_act));
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
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		double ref_x_prev = this->previous_path_x[prev_size-2];
		double ref_y_prev = this->previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	//generate waypoints in 30, 60 and 90 meters distance from last already planned car position
	vector<double> next_wp0 = getXY(s_act+50,  2+4*lane_cmd, maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(s_act+80,  2+4*lane_cmd, maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(s_act+120, 2+4*lane_cmd, maps_s, maps_x, maps_y);

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

	//define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

	//push already planned path in next_xy_vals containers
	cand_path_x.clear();
	cand_path_y.clear();
	for(unsigned int i=0; i<previous_path_x.size();i++) {
		cand_path_x.push_back(previous_path_x[i]);
		cand_path_y.push_back(previous_path_y[i]);
	}

	double target_x = 30.; //set lookahead distance (in vehicle fixed KOS! x-direction) to 30m
	double target_y = s(target_x); //compute y values with previously computed spline
	double target_dist = sqrt(pow(target_x,2)+pow(target_y,2)); //compute actual distance

	double x_point = 0;
	double y_point = 0;

	for(unsigned i=0; i<=50-previous_path_x.size(); i++) {
		if(vel_cmd > 0.2*vel_max) {
			vel_cmd = bound(0.2*vel_max+0.224, vel_max, vel_cmd + vel_inc);
		} else {
			vel_cmd = bound(0.001, vel_max, vel_cmd + vel_inc);
		}
		double N = (target_dist/(0.02*vel_cmd)); //compute number of position increments to cover target_dist when driving with vel_cmd
		x_point += target_x/N; //increment x-position
		y_point = s(x_point); //compute y-position with spline function

		//derotate to global KOS and push on next_xy_vals containers
		cand_path_x.push_back(ref_x + x_point*cos(ref_yaw) - y_point*sin(ref_yaw));
		cand_path_y.push_back(ref_y + x_point*sin(ref_yaw) + y_point*cos(ref_yaw));
	}
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*M_PI - angle, angle);

  if(angle > M_PI/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}
