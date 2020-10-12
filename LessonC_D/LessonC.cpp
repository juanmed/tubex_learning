#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;

int main()
{
	// True poses
	Vector x_truth({2,1,M_PI/6.});
	Vector y_truth({6,M_PI/6.});
	Vector m_truth({5,6.2});

	// Interval vectors for states, measurements and landmarks
	IntervalVector x(3);
	x[2] = Interval(x_truth[2]); // ahhh! the heading is known
	IntervalVector y{{y_truth[0],y_truth[0]},{y_truth[1],y_truth[1]}};
	IntervalVector m{{m_truth[0],m_truth[0]},{m_truth[1],m_truth[1]}};
	// incrase set by uncertainty
	y[0].inflate(0.3); //
	y[1].inflate(0.1);
	m.inflate(0.2);

	cout << " State x: " << x << endl;
	cout << " Measurement y: " << y << endl;
	cout << "	Landmark m : " << m << endl;

	// Intermediate variables
	Interval theta;
	IntervalVector d(2);

	ContractorNetwork cn;
	CtcFunction ctc_addition(Function("a","b","c","a+b-c")); 
	CtcFunction ctc_subtraction(Function("a","b","c","a-b-c"));

	cn.add(ctc_addition, {x[2], y[1], theta});
	//cn.add(ctc_addition, {x[0], d[0], m[0]});
	//cn.add(ctc_addition, {x[1], d[1], m[1]});
  cn.add(ctc_subtraction, {m[0], x[0], d[0]});
  cn.add(ctc_subtraction, {m[1], x[1], d[1]});
	cn.add(ctc::polar, {d, y[0],theta});

	cn.contract();


	vibes::beginDrawing();

	VIBesFigMap fig_map("Map");
	fig_map.set_properties(100,100,500,500);
	fig_map.axis_limits(0,7,0,7);
	fig_map.draw_vehicle(x_truth, 1);
	fig_map.draw_box(m, "red");
	fig_map.draw_box(x.subvector(0,1)); // does not display anything if unbounded
	fig_map.draw_pie(x_truth[0],x_truth[1],y[0],y[1]+x_truth[2]);
	fig_map.draw_pie(x_truth[0],x_truth[1], (Interval(0.1)|y[0]), y[1]+x_truth[2], "lightGray");
	fig_map.show();

	vibes::endDrawing();

}