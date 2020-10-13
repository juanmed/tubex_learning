#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;

class MyCtc : public ibex::Ctc
{
  public:

    MyCtc(const std::vector<ibex::IntervalVector>& M_)
      : ibex::Ctc(2), // the contractor acts on 2d boxes
        M(M_)         // attribute needed later on for the contraction
    {

    }

    void contract(ibex::IntervalVector& a)
    {
      IntervalVector b = IntervalVector::empty(2);
      for(auto& Mi : M)
      b |= (a & Mi);
      a = b;
    }

    void contract2(ibex::IntervalVector& a)
    {
      // Insert contraction formula here (question D.2)
      vector<IntervalVector> intersections;
      for(const auto mj : M)
      {
        // compute intersection and add to vector
        IntervalVector intersection = a & mj; 
        //a &= mj;
        intersections.push_back(intersection);
      }
      /*
      It is necessary to initialize union with any element
      from all intersections... and only afterwards compute
      union of all intersections
      */
      IntervalVector squared_union = intersections[0];
      for(const auto sj : intersections)
      {
        squared_union |= sj;
      }

      a = squared_union;

    }

  protected:

    const std::vector<ibex::IntervalVector> M;
};	

int main()
{
  
  // define constellation of landmark measurements	
  vector<IntervalVector> M;
  M.push_back(IntervalVector {{1.5,1.5}, {2.5,2.5}});
  M.push_back(IntervalVector {{3,3}, {1,1}});
  M.push_back(IntervalVector {{2,2}, {2,2}});
  M.push_back(IntervalVector {{2.5,2.5}, {3,3}});
  M.push_back(IntervalVector {{3.5,3.5}, {2,2}});
  M.push_back(IntervalVector {{4,4}, {1,1}});
  M.push_back(IntervalVector {{1.5,1.5}, {0.5,0.5}});

  //cout << "M before: " << M << endl;
  for(auto& Mi : M)
  {
    Mi.inflate(0.05);
    cout << "M after : " << Mi << endl;
  }

  // define boxes to be contracted
  IntervalVector a1{{1.25,3},{1.6,2.75}};
  IntervalVector a2{{2,3.5},{0.6,1.2}};
  IntervalVector a3{{1.1,3.25},{0.2,1.4}};
  cout << "Before a1: " << a1 << endl;
  cout << "Before a2: " << a2 << endl;
  cout << "Before a3: " << a3 << endl;

  // draw landmarks and initial intervals
  vibes::beginDrawing();

  VIBesFigMap fig_map("Map");
  fig_map.set_properties(100,100,500,500);
  fig_map.axis_limits(0,4.5,0,3.5);

  for(auto& Mi : M)
  {
    fig_map.draw_box(Mi, "red");
  }
  fig_map.draw_box(a1, "blue");
  fig_map.draw_box(a2, "blue");
  fig_map.draw_box(a3, "blue");

  MyCtc ctc_constellation(M);
  ContractorNetwork cn;

  cn.add(ctc_constellation, {a1});
  cn.add(ctc_constellation, {a2});
  cn.add(ctc_constellation, {a3});

  cn.contract();

  cout << "After a1: " << a1 << endl;
  cout << "After a2: " << a2 << endl;
  cout << "After a3: " << a3 << endl;

  fig_map.draw_box(a1, "green");
  fig_map.draw_box(a2, "green");
  fig_map.draw_box(a3, "green");

  fig_map.show();

  vibes::endDrawing();

  // Define true position and observations
  Vector x_truth({2,1,M_PI/6.});
  vector<IntervalVector> v_obs = DataLoader::generate_static_observations(x_truth, M, false);
  // Adding uncertainties on the measurements
  for(auto& obs : v_obs)
  {
    obs[0].inflate(0.02); // range
    obs[1].inflate(0.02); // bearing
  }

  vibes::beginDrawing();
  VIBesFigMap fig_map2("Map2");
  fig_map2.set_properties(100,100,500,500);
  fig_map2.draw_vehicle(x_truth,0.5);
  for(const auto& Mi : M)
  {
    fig_map2.draw_box(Mi, "red[orange]");
  }
  for(const auto& obs : v_obs)
  {
    fig_map2.draw_pie(x_truth[0],x_truth[1], obs[0], obs[1] + x_truth[2]);
    fig_map2.draw_pie(x_truth[0],x_truth[1], (Interval(0.1)|obs[0]), obs[1] + x_truth[2], "lightGray");
  }
  //fig_map2.axis_limits(fig_map2.view_box(), true, 0.1);
    

  // Solve state estimation problem assuming known landmark - measurement relation is known (association)

  ContractorNetwork cn2;
  CtcFunction ctc_addition(Function("a","b","c","a+b-c")); 
  CtcFunction ctc_subtraction(Function("a","b","c","a-b-c"));

  // This is the state of the body
  IntervalVector x(3);
  x[2] = x_truth[2];

  for(int i = 0 ; i < v_obs.size() ; i++) // for each measurement
  {
    // Define intermediate variables
    Interval& theta =  cn2.create_dom(Interval());
    IntervalVector& d = cn2.create_dom(IntervalVector(2));

    // Get landmark and ASSOCIATED maesurement
    IntervalVector y = M[i];
    IntervalVector m = v_obs[i];


    // Add contractors and related domains
    cn2.add(ctc_addition, {x[2], y[1], theta});
    cn2.add(ctc_subtraction, {m[0], x[0], d[0]});
    cn2.add(ctc_subtraction, {m[1], x[1], d[1]});
    cn2.add(ctc::polar, {d, y[0],theta});

  }

  // Contract the interval for X = solve the estimation problem
  cn2.contract();  
  cout << "x after contraction without data association: " << x << endl;
  fig_map2.draw_box(x.subvector(0,1), "red"); // draw possible state

  /*
  Now solve the estimation problem with 
  data association
  */
  ContractorNetwork cn3;
  x = IntervalVector(3);
  x[2] = x_truth[2];
  // Association set (possible identities)
  vector<IntervalVector> m(v_obs.size(), IntervalVector(2));
  for(int i = 0 ; i < v_obs.size() ; i++)
  {
    cout << "v obs " << i << " : " << v_obs[i] << endl;
    cout << "m " << i << " : " << m[i] << endl;
  }

  for(int i = 0 ; i < v_obs.size() ; i++) // for each measurement
  {
    // Define intermediate variables
    Interval& theta =  cn3.create_dom(Interval());
    IntervalVector& d = cn3.create_dom(IntervalVector(2));

    // Get landmark and ASSOCIATED maesurement
    IntervalVector y = m[i];
    IntervalVector meas = v_obs[i];

    // Add contractors and related domains
    cn3.add(ctc_addition, {x[2], y[1], theta});
    cn3.add(ctc_constellation, {y});
    //cn.add(ctc_addition, {x[1], d[1], m[1]});
    cn3.add(ctc_subtraction, {meas[0], x[0], d[0]});
    cn3.add(ctc_subtraction, {meas[1], x[1], d[1]});
    cn3.add(ctc::polar, {d, y[0],theta});

  }

  cn3.contract();  
  cout << "x after contraction with data association: " << x << endl;
  fig_map2.draw_box(x.subvector(0,1), "green"); // draw possible state

  vibes::endDrawing();


  return 0;
}