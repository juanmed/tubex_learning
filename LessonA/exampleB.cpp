#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;

int main()
{
	// Truth pose
	Vector x_truth({0.,0.,M_PI/6.}); // (x,y,heading)

	// Creating random map of landmarks
	int nb_landmarks = 3;
	IntervalVector map_area(2, Interval(-5,5));
	vector<IntervalVector> v_b = DataLoader::generate_landmarks_boxes(map_area, nb_landmarks);
	for (int i = 0 ; i < nb_landmarks ; i++)
	{
		cout << "landmark " << i << " : " << v_b[i] << endl;
	}

  // The following function generates a set of [range]x[bearing] values
  vector<IntervalVector> v_obs = DataLoader::generate_static_observations(x_truth, v_b, false);	

  // We keep range-only observations from v_obs, and add uncertainties
  vector<Interval> v_d;
  for(auto& obs : v_obs)
  {
    v_d.push_back(obs[0].inflate(0.1)); // adding uncertainties: [-0.1,0.1]
    cout << "range: " << (obs[0]) << " bearing: " << (obs[1]) << endl; 
  }
	for (int i = 0 ; i < nb_landmarks ; i++)
	{
		cout << "Observation " << i << " : " << v_d[i] << endl;
	}

  // Set of feasible positions for x: x ϵ [-∞,∞]×[-∞,∞]
  IntervalVector x(2);

  // Create constraints based on measurements and functions
  // and add them to the network
  ContractorNetwork cn;
 	CtcFunction ctc_dist(Function("x[2]","y[2]","d","sqrt( (x[0] - y[0])^2 + (x[1] -y[1])^2 ) - d"));
  for (int i =0; i < nb_landmarks; i++)
  {
  	cn.add(ctc_dist, {x,v_b[i],v_d[i]});
  }

  // This will reduce the valid interval for X, the true position,
  // to an interval that respects all constraints
  cn.contract();

  // Graph
  vibes::beginDrawing();

  VIBesFigMap fig("Map");
  fig.set_properties(50, 50, 600, 600);

  for(const auto& b : v_b)
    fig.add_beacon(b.mid(), 0.2);

  for(int i = 0 ; i < nb_landmarks ; i++)
    fig.draw_ring(v_b[i][0].mid(), v_b[i][1].mid(), v_d[i], "gray");

  fig.draw_vehicle(x_truth, 0.7); // last param: vehicle size
  //fig.draw_box(x); // estimated position
  fig.show();

  vibes::endDrawing();

}