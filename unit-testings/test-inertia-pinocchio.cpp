#include <iostream>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

using namespace se3;
//filename : "/opt/openrobots/share/talos_data/urdf/talos_reduced.urdf"


int main()
{
    const std::string filename= "/opt/openrobots/share/talos_data/robots/talos_reduced.urdf";
    Model model;
    std::cout << "Build urdf model" <<std::endl;
    se3::urdf::buildModel(filename, model, false);
    se3::Data data(model);
    Eigen::MatrixXd M(model.nv,model.nv);
    data.M.fill(0);

    std::cout << "Inertia matrix:\n " <<data.M <<std::endl;

    return 0;

}

