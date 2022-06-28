#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;


int main(){
// True values
    double dt = 0.01;
    Interval tdomain(0.,5.); // temporal domain [t0,tf]    // create trajectory x*(.), named actual_x
    TrajectoryVector actual_x(tdomain, TFunction("(2*cos(t) ; sin(2*t))"), dt);    // compute the tube of actual_x
    //TubeVector x(tdomain, dt, TFunction("(2*cos(t) ; sin(2*t))"));
    TubeVector x({
    Tube(tdomain, dt),
    Tube(tdomain, dt)
    });
    //x.inflate(0.2);    // new values for the the temporal evaluation of [x](·)
    Interval ti(1.8,2.2 );
    //double ti=2;
    IntervalVector yi({{-0.84, -0.83}, {-0.76, -0.75}});    // create a tube y for enclosing the trajectory of distances between the robot and the landmark
    

    TubeVector v(tdomain, dt, TFunction("(-2*sin(t) ; 2*cos(2*t))"));
    v.inflate(0.03);    // create contractors working on tubes
    ContractorNetwork cn;
    cn.add(ctc::eval, {ti, yi, x, v});    // Draw Map
    vibes::beginDrawing();    VIBesFigMap fig_map("Map");
    fig_map.set_properties(100, 100, 600, 300);    
    fig_map.add_trajectory(&actual_x, "x*", 0, 1);
//    fig_map.add_tube(&x, "x", 0, 1);
//    fig_map.show(0.5); // argument is robot size
    cn.contract();
    fig_map.add_tube(&x, "x", 0, 1);             // after contracted
    fig_map.draw_box(yi, "red[yellow]");    fig_map.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map.show(0.5); // argument is robot size    vibes::endDrawing(); 
    return 0;

}