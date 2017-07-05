#include <iostream>
#include <Eigen/Dense>
#include <state-observation/tools/definitions.hpp>
#include <pyrene-state-observation/tools/pyrene.hpp>

using namespace stateObservation;
using namespace pyreneStateObservation;
using namespace std;

int main()
{

  Matrix m(2,2);
  m(0,0)=m(0,1)=m(1,0)=m(1,1)=3;
  cout << "matrix m:\n" << m <<endl;
  Vector3 v(3);
  v(0) = 4;
  v(1) = v(0) ;
  v(2)= v(1) ;
  cout << "vector v:\n" << v << endl;


  cout << "Identity matrix is: \n" <<   Matrix::Identity(3,3) <<endl;


  cout << "Vector m*g.uz is \n" << pyrene::m*Vector3(9.8,0,0)<< endl; //changer les components du vecteur





}
