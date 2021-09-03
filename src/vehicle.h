#ifndef VEHICLE
#define VEHICLE

#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using namespace std;

class Vehicle
{
public:

    vector<double> start_state(6);

    Vehicle();
    Vehicle(vector<double> start_state);
    virtual ~Vehicle();

    vector<double> state_in(double time);

};
#endif
