#include"../src/gaussian-white-noise.cpp"
#include<iostream>
using namespace std;

int main(){
cout << "Testing" <<endl;
GaussianWhiteNoise V(1);
cout << "Dimension = " <<V.getDimension()<<endl;

return 0;
}
