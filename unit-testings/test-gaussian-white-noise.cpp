#include <state-observation/noise/gaussian-white-noise.hpp>
#include <iostream>
using namespace std;

int main(){
cout << "Testing" <<endl;
GaussianWhiteNoise V(1);
cout << "Dimension = " <<V.getDimension()<<endl;

return 0;
}
