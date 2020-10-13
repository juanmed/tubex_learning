#include <tubex.h>
#include <math.h>

using namespace std;
using namespace tubex;
class MyCtc : public ibex::Ctc
{
	public:
	MyCtc(const std::vector<ibex::IntervalVector>& M_)
	: ibex::Ctc(2), // the contractor acts on 2d boxes
	M(M_) // attribute needed later on for the contraction
	{

	}
	void contract(ibex::IntervalVector& a)
	{
		IntervalVector b = IntervalVector::empty(2);
		for(auto& Mi : M)
		b |= (a & Mi);
		a = b;
	}
	protected:
		const std::vector<ibex::IntervalVector> M;
};

int main()
{
	Vector x_truth({2,1,M_PI/6});
	IntervalVector x(3); // estimated state vector
	x[2] = x_truth[2];
	vector<IntervalVector> M;
	M.push_back(IntervalVector( {{1.5},{2.5}}));
	M.push_back(IntervalVector( {{3},{1}}));
	M.push_back(IntervalVector( {{2},{2}}));
	M.push_back(IntervalVector( {{2.5},{3}}));
	M.push_back(IntervalVector( {{3.5},{2}}));
	M.push_back(IntervalVector( {{4},{1}}));
	M.push_back(IntervalVector( {{1.5},{0.5}}));
	for(auto& Mi : M)
	{
		Mi.inflate(0.05);
	}

	vector<IntervalVector> v_obs = DataLoader::generate_static_observations(x_truth, M, false);
	for(auto& obs : v_obs)
	{
		obs[0].inflate(0.02); // range
		obs[1].inflate(0.02); // bearing
	}

	// Association set (possible identities)
	vector<IntervalVector> m(v_obs.size(), IntervalVector(2));

	// unknown association for each observation

	// Contractors
	CtcFunction ctc_plus(Function("a", "b", "c", "a+b-c")); // a+b=c
	CtcFunction ctc_minus(Function("a", "b", "c", "a-b-c")); // a-b=c
	MyCtc ctc_asso(M);
	ContractorNetwork cn;
	//cout << v_obs << endl;

	for(int i = 0 ; i < v_obs.size() ; i++) // for each measurement
	{
		// Intermediate variables
		Interval& theta = cn.create_dom(Interval());
		IntervalVector& d = cn.create_dom(IntervalVector(2));

		// Solver
		cn.add(ctc_asso, {m[i]});
		cn.add(ctc_plus, {v_obs[i][1], x[2], theta});
		cn.add(ctc_minus, {m[i][0], x[0], d[0]});
		cn.add(ctc_minus, {m[i][1], x[1], d[1]});
		cn.add(ctc::polar, {d, v_obs[i][0], theta});
	}
	cn.contract();
	cout<<x<<endl; 
	
	vibes::beginDrawing();


	VIBesFigMap fig_map("Map");
	fig_map.set_properties(100,100,500,500);
	fig_map.draw_vehicle(x_truth,0.5);

	for(int i = 0 ; i < M.size() ; i++)
	{
		fig_map.draw_box(M[i], "red[orange]");
	}

	for(auto& y : v_obs)
	{
		fig_map.draw_pie(x_truth[0],x_truth[1],(Interval(0.05)|y[0]),y[1]+x_truth[2],"lightGray");
		fig_map.draw_pie(x_truth[0],x_truth[1],y[0],y[1]+x_truth[2]);
	}

	// Draw the result of next questions here
	fig_map.draw_box(x.subvector(0,1)); // displays ([x_1],[x_2])
	fig_map.axis_limits(fig_map.view_box(), true, 0.1);
	for(const auto& mi : m)
	{
		if(mi.max_diam() <= 0.10001) // if identified
		fig_map.draw_circle(mi[0].mid(), mi[1].mid(), 0.02, "blue[blue]");
	}
	vibes::endDrawing();
	
}