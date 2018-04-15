#include <math.h>

#include "assist.h"

double deg2rad(double x) {
	double out = x * M_PI / 180;
	while(out >  M_PI) out -= 2*M_PI;
	while(out < -M_PI) out += 2*M_PI;
	return out;
}

double rad2deg(double x) { return x * 180 / M_PI; }

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double bound(double min, double max, double in) {
	double out=in;

	if(out<min) out=min;
	else if(out>max) out=max;

	return out;
}
