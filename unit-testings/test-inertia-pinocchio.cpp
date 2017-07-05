#include <iostream>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"


using namespace std;


//filename : "/opt/openrobots/share/talos_data/urdf/talos_reduced.urdf"


int main()
{
    const std::string filename= "/opt/openrobots/share/talos_data/robots/talos_reduced.urdf";
    cout << "Parse Filename : " << filename << endl;
    se3::Model model;
    se3::urdf::buildModel(filename, model, false);
    cout << "Build urdf model" <<endl;
    return 0;

}
