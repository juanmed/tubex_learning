#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;

int main(){

	double dt = 0.01;
	Interval tdomain(0,3);


	// ground true trajectory
	Trajectory x1(tdomain, TFunction("10*cos(t) + t"), dt);
	Trajectory x2(tdomain, TFunction("5*sin(2*t) + t"), dt);
	Trajectory h(tdomain, TFunction("atan2((10*cos(2*t)+1),(-10*sin(t)+1))"), dt);
	Trajectory s(tdomain, TFunction("sqrt((-10*sin(t)+1)^2+(10*cos(2*t)+1)^2)"), dt);

	
	Tube h_meas(h, dt);
	//h_meas.inflate(0.01);
	Tube s_meas(s, dt);
	//s_meas.inflate(0.01);

	// true position tube vector
	TubeVector x_true({
		Tube (x1,dt),
		Tube (x2,dt)
	});	

	
	// state tube vector
	TubeVector x({
		Tube(tdomain, dt),
		Tube(tdomain, dt),
		h_meas,
		s_meas
	});	

	// input tube vector
	TubeVector u({
		Tube(tdomain, dt),
		Tube(tdomain, dt)
	});

	
	// state derivative tube vector
	TubeVector f({
		s_meas*cos(h_meas),
		s_meas*sin(h_meas),
		u[0],
		u[1]
	});
	

	// measurements: time and value
	Interval t1(0.1,0.4);
	Interval t2(1.5,1.5);
	Interval t3(2.,2.);

	Interval y1(1.9,1.9);
	Interval y2(3.6, 3.6);
	Interval y3(2.8,2.8);
	y1.inflate(0.1);
	y2.inflate(0.1);
	y3.inflate(0.1);

	// landmarks
	IntervalVector b1({{8,8},{3,3}});
	IntervalVector b2({{0,0},{5,5}});
	IntervalVector b3({{-2,-2},{1,1}});
	b1.inflate(0.1);
	b2.inflate(0.1);
	b3.inflate(0.1);

	cout << x.subvector(0,1)(t1) << endl;
	ContractorNetwork cn;
	cn.add(ctc::deriv, {x, f});
	IntervalVector& k1 = cn.create_dom(IntervalVector(4));
	cn.add(ctc::dist, {cn.subvector(k1,0,1),b1, y1});
	cn.add(ctc::eval, {t1, k1, x, f });

	IntervalVector& k2 = cn.create_dom(IntervalVector(4));
	cn.add(ctc::dist, {cn.subvector(k2,0,1),b2, y2});
	cn.add(ctc::eval, {t2, k2, x, f });

	IntervalVector& k3 = cn.create_dom(IntervalVector(4));
	cn.add(ctc::dist, {cn.subvector(k3,0,1),b3, y3});
	cn.add(ctc::eval, {t3, k3, x, f });
	
	cn.contract();

	cout << x.subvector(0,1)(t1) << endl;
	

	// create figure
	vibes::beginDrawing();
	VIBesFigMap fig_map("Map");
	fig_map.set_properties(100, 100, 600, 300);
	fig_map.axis_limits(-8,10,-4,8, true);

	fig_map.draw_box(b1, "red[yellow]");
	fig_map.draw_box(b2, "red[yellow]");
	fig_map.draw_box(b3, "red[yellow]");
	fig_map.draw_ring(b1[0].mid(), b1[1].mid(), y1, "gray");
	fig_map.draw_ring(b2[0].mid(), b2[1].mid(), y2, "gray");
	fig_map.draw_ring(b3[0].mid(), b3[1].mid(), y3, "gray");


	fig_map.add_tube(&x_true, "x_true",0,1);
	fig_map.add_tube(&x,"x_est",0,1);

	// display true trajectory for viz purposes
	//fig_map.add_trajectory(TrajectoryVector (tdomain, , dt));
	fig_map.show(0.5);
	
	VIBesFigTube fig_dist("Control inputs");
	fig_dist.set_properties(100, 100, 600, 300);
	fig_dist.add_tube(&u[0], "u0");
	fig_dist.add_tube(&u[1], "u1");

	fig_dist.show();


	return 0;
}