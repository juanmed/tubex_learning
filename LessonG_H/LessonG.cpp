#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;


int main()
{
    double dt = 0.05;
    Interval tdomain(0,6);    IntervalVector x(3); // estimated state vector
    x[2] = Interval(M_PI/6.); // the heading is known    // actual_x but unknown truth
    TrajectoryVector actual_x(tdomain, TFunction("(10*cos(t) + t ; 5*sin(2*t)+t )"),dt);    // Creating random map of landmarks
    int nb_landmarks = 150;
    IntervalVector map_area(actual_x.codomain().subvector(0,1));
    map_area.inflate(2);
    vector<IntervalVector> v_map =
    DataLoader::generate_landmarks_boxes(map_area, nb_landmarks);    // Generating observations obs=(t,range,bearing) of these landmarks
    int max_nb_obs = 20;
    Interval visi_range(0,4); // [0m,75m]
    Interval visi_angle(-M_PI/4,M_PI/4); // frontal sonar
    vector<IntervalVector> v_obs =
    DataLoader::generate_observations(actual_x, v_map, max_nb_obs,
                                        true, visi_range, visi_angle);    // Draw a Map
    vibes::beginDrawing();
    // set a name of a map
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(100,100,900,500); //upper left (100, 100), size (300, 300)
    fig_map.add_trajectory(&actual_x, "x*", 0, 1);    // draw landmarks
    for(int i=0; i<nb_landmarks; i++)
    {
        fig_map.add_beacon(v_map[i].mid(), 0.2, "red[orange]");
    }
    // Uncertainties of range & bearing
    for(int i=0; i<max_nb_obs; i++)
    {
      (v_obs[i])[1].inflate(0.1); // range
      (v_obs[i])[2].inflate(0.04); // bearing
    }
    fig_map.add_observations(v_obs, &actual_x, "black");    fig_map.axis_limits(fig_map.view_box(), true, 0.1);
    fig_map.show(1);    
    vibes::endDrawing();
    return 0;
} 