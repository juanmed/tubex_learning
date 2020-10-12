#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;

int main(){

	double dt = 0.01;
	Interval tdomain(0,3);

	Trajectory x1(tdomain, TFunction("10*cos(t) + t"), dt);
	Trajectory x2(tdomain, TFunction("5*sin(2*t) + t"), dt);
	Trajectory h(tdomain, TFunction("atan2( -10*sin(t) + 1, 10*cost(2*t) +1)"), dt);
	Trajectory s(tdomain, TFunction("sqrt( sqr(-10*sin(t) + 1) + sqr(10*cost(2*t) +1) )"), dt);


	Tube h_meas(h, dt);
	h_meas.inflate(0.01);
	Tube s_meas(s, dt);
	s_meas.inflate(0.01);

	return 0;
}