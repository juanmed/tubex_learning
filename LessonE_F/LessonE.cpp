#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;

/*
Mobile robot motion is usually described by an evolution function f involved in a 
differential equation. The interval state estimation then amounts to estimating the 
set of feasible trajectories that are solutions of this differential equation. 
The difficulty is to propagate the uncertainties in time, and to consider observations 
from asynchronous measurements in the same time. We will see that these difficulties 
are easily handled with constraint propagations on tubes.
*/

int main()
{
	double dt = 0.01;
	Interval tdomain(0,5);
	TrajectoryVector actual_x(tdomain, TFunction("(2*cos(t) ; sin(2*t))"), dt);

	Vector b{0.5,1};	//landmark

	//TrajectoryVector b_distance(tdomain, TFunction(b,"sqrt( (2*cos(t) - b[0])^2 + (sin(2*t) -b[1])^2 ) ,t)", dt));
	Trajectory dist = sqrt(sqr(actual_x[0]-b[0])+sqr(actual_x[1]-b[1])); // simple operations between traj.
	RandTrajectory n(tdomain, dt, Interval(-0.5,0.5));
	Trajectory time(tdomain, TFunction("t"), dt);
	// TrajectoryVector as a list of scalar trajectories
	TrajectoryVector actual_y({
	    dist + n,
	    time
	  });

	TubeVector x(actual_x, dt);
	x.inflate(0.2);
	Tube y(dist+n, dt);
	//y.inflate(0.2);


	vibes::beginDrawing();
	VIBesFigMap fig_map("Map");
  fig_map.set_properties(100, 100, 600, 300);
  fig_map.axis_limits(-2.5,2.5,-0.1,0.1, true);	
  fig_map.add_trajectory(&actual_x, "x*", 0, 1);
  fig_map.add_trajectory(&actual_y, "y*", 1, 0);
	fig_map.add_tube(&x, "x", 0, 1);
	//fig_map.add_tube(&y, "y", 1, 0);
  fig_map.add_beacon(b, 0.1); 
  //fig_map.draw_box(b, "yellow");
  fig_map.show(0.5);  

	ContractorNetwork cn;
	cn.add(ctc::dist, {x, b,y});	
	//cn.contract();

	//fig_map.show(0.5); 

	// speed
	TrajectoryVector actual_v(tdomain, TFunction("(-2*sin(t) ; 2*cos(2*t))"), dt);
	TubeVector v(actual_v, dt);
	v.inflate(0.03);

	cn.add(ctc::deriv, {x, v});
	cn.contract();
	fig_map.add_tube(&v, "v", 0, 1);
	fig_map.show(0.5);  

}