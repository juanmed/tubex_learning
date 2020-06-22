#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;

int main()
{
	Tube x(Interval(0,10), 0.01, TFunction("cos(t)+abs(t-5)*[-0.1,0.1]"));
	IntervalVector a{{3,4},{4,6}};

	cout << "Tube: " << x << endl;
	cout << "a: " << a << endl;

	/*
	vibes::beginDrawing();
	VIBesFigTube fig("LessonA Tube");
	fig.add_tube(&x, "x");
	fig.show();
	vibes::endDrawing();
	*/

	Interval m(8,10);
	Interval n(1,2);
	cout << " - Exercise A.1: " << endl;
	cout << " m: " << m << ", n: " << n << endl;
	cout << " m/n: " << m/n << endl;

	
	cout << " - Exercise A.2: " << endl;

	m = Interval(-2,4);
	n = Interval(1,3);
	cout << " m: " << m << ", n: " << n << endl;
	cout << " m*n: " << m*n << endl;

	m = Interval(8,10);
	n = Interval(-1,0);
	cout << "m: " << m << ", n: " << n << endl;
	cout << " m/n: " << m/n << endl;

	m = Interval(-2,4);
	n = Interval(6,7);
	Interval t = m|n;
	cout << "m: " << m << ", n: " << n << endl;
	cout << " m|n: " << t << endl;

	m = Interval(2,7);
	n = Interval(1,9);
	cout << "m: " << m << ", n: " << n << endl;
	cout << " max(m,n): " << max(m,n) << endl;	

	m = Interval::empty_set();
	n = Interval(1,2);
	cout << "m: " << m << ", n: " << n << endl;
	cout << " max(m,n): " << max(m,n) << endl;

	m = Interval();
	cout << "m: " << m << endl;
	cout << " cos(m): " << cos(m) << endl;

	m = Interval(-1,4);
	cout << "m: " << m << endl;
	cout << " sqr(m): " << sqr(m) << endl;

	m = Interval(1,2);
	n = Interval(-1,3);
	Interval o = Interval(1,3);
	Interval p = Interval(6,7);
	Interval q = Interval(1,2);
	cout << "m: " << m << endl;
	cout << " m*n + max(o U p, q): " << m*n + max(o&p, q) << endl;

	cout << " - Exercise A.3: " << endl;
	//Interval pi = Interval::pi();
	double pi = M_PI;
	IntervalVector k{{0,pi},{-pi/6, pi/6}};
	cout << "k: " << k << endl;
	cout << " abs(k): " << abs(k) << endl;

	cout << " - Exercise A.4: " << endl;
 	IntervalVector y{{0,0},{0,0}};
 	IntervalVector b{{3,4},{2,3}};
 	IntervalVector cont = cart_prod(y,b);
 	Function f("x[4]", "sqrt( (x[0] - x[2])^2 + (x[1] -x[3])^2 )");
 	cout << "y U b: " << cont << endl;
 	cout << "g(y,b) = " << (f.eval(cont)) << endl;

 	cout << " - Exercise A.5, A.6: " << endl;
 	vibes::beginDrawing();
 	VIBesFigMap fig("Map");
 	fig.set_properties(50,50,400,400);
 	fig.draw_circle(0,0,f.eval(cont).ub(), "red[yellow]");
 	fig.draw_circle(0,0,f.eval(cont).lb(), "red[white]");
 	fig.draw_box(y);
 	fig.draw_box(b, "red[yellow]");
 	fig.show();
 	vibes::endDrawing();

 	y.inflate(0.1);
	cont = cart_prod(y,b);
 	cout << "y U b: " << cont << endl;
 	Interval result = f.eval(cont);
 	cout << "g(y,b) = " << result << endl;
 	cout << " - Exercise A.7: " << endl;
 	vibes::beginDrawing();
 	VIBesFigMap fig2("Map2");
 	fig2.set_properties(50,50,400,400);
 	fig2.draw_circle(0,0,result.ub(), "red[yellow]");
 	fig2.draw_circle(0,0,result.lb(), "red[white]");
 	fig2.draw_box(y);
 	fig2.draw_box(b, "red[yellow]");
 	fig2.show();
 	vibes::endDrawing();

 	cout << " - Exercise A.8: " << endl;
 	CtcFunction ctc_dist(Function("x[2]","y[2]","d","sqrt( (x[0] - y[0])^2 + (x[1] -y[1])^2 ) - d"));
 	Interval d(7,8);
 	y = IntervalVector{{0,0},{0,0}};
 	IntervalVector b1{{1.5, 2.5},{4, 11}};
 	IntervalVector b2{{3, 4},{4, 6.5}};
 	IntervalVector b3{{5, 7},{5.5, 8}};

 	vibes::beginDrawing();
 	VIBesFigMap fig3("Before");
 	fig3.set_properties(50,50,400,400);
 	fig3.draw_circle(0,0,d.ub(), "red[yellow]");
 	fig3.draw_circle(0,0,d.lb(), "red[white]");
 	fig3.draw_box(b1, "blue[white]");
  fig3.draw_box(b2, "blue[white]");
 	fig3.draw_box(b3, "blue[white]");
 	fig3.show();
 	vibes::endDrawing();

 	ContractorNetwork cn;
 	cn.add(ctc_dist, {y,b1,d});
 	cn.add(ctc_dist, {y,b2,d});
 	cn.add(ctc_dist, {y,b3,d});

 	cn.contract();

 	vibes::beginDrawing();
 	VIBesFigMap fig4("After");
 	fig4.set_properties(50,50,400,400);
 	fig4.draw_circle(0,0,d.ub(), "red[yellow]");
 	fig4.draw_circle(0,0,d.lb(), "red[white]");
 	fig4.draw_box(b1, "blue[white]");
  fig4.draw_box(b2, "blue[white]");
 	fig4.draw_box(b3, "blue[white]");
 	fig4.show();
 	vibes::endDrawing();

}