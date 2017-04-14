#include <state-observation/noise/gaussian-white-noise.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Cholesky>
using namespace std;
using namespace stateObservation;

int main(){
cout << "Testing" <<endl;
GaussianWhiteNoise V(0);
V.setDimension(3);
cout << "Dimension = " <<V.getDimension()<<endl;
return 0;
}
