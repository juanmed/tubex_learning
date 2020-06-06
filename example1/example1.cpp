#include <tubex.h>

using namespace std;
using namespace tubex;

int main()
{
	Tube x(Interval(0,10), 0.01, TFunction("cos(t)+abs(t-5)*[-0.1,0.1]"));
	
	vibes::beginDrawing();
	VIBesFigTube fig("My first tube");
	fig.add_tube(&x, "x");
	fig.show();
	vibes::endDrawing();	
}