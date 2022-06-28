#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;


int main(){

	double dt = 0.01;
	Interval tdomain(0,5);

	// Lesson F
	// empty tube of 2 elements
	TubeVector x2({
    Tube(tdomain, dt),
    Tube(tdomain, dt)
  });

	TrajectoryVector actual_v2(tdomain, TFunction("(-2*sin(t) ; 2*cos(2*t))"), dt);
	TubeVector v2(actual_v2, dt);
	v2.inflate(0.03);

	// measurement
	double t1 = 2;
	IntervalVector y1({{-0.84,-0.83},{-0.76,-0.75}});
	double t2 = 1;
	IntervalVector y2({{1.07,1.09},{0.89,0.91}});

  ContractorNetwork cn2;
  cn2.add(ctc::eval, {t1, y1, x2, v2});
  cn2.add(ctc::eval, {t2, y2, x2, v2});
  cn2.contract();

	vibes::beginDrawing();
	VIBesFigMap fig_map2("Map2");
  fig_map2.set_properties(100, 100, 600, 300);
  fig_map2.axis_limits(-2.5,2.5,-0.1,0.1, true);
  fig_map2.add_tube(&x2, "x2",0,1);
  
  fig_map2.draw_box(y1, "red[yellow]");
  fig_map2.draw_box(y2, "red[yellow]");

  fig_map2.show(0.5); // argument is robot size

  return 0;
}