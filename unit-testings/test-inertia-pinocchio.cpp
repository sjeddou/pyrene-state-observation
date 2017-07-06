#include <iostream>
#include <Eigen/Cholesky>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/urdf.hpp"

using namespace se3;
//filename : "/opt/openrobots/share/talos_data/urdf/talos_reduced.urdf"


int main()
{
    const std::string filename= "/opt/openrobots/share/talos_data/robots/talos_reduced.urdf";
    se3::Model model;
    std::cout << "Build urdf model" <<std::endl;
    se3::urdf::buildModel(filename, model);
    se3::Data data(model);
    data.M.fill(0);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    se3::crba(model,data,q);
    Eigen::MatrixXd M(model.nv,model.nv);
    std::cout << "nv is :" << model.nv <<std::endl;
    std::cout << "Inertia matrix:\n " <<data.M <<std::endl;
    std::cout << "Vector q is of size : " <<q.size() <<std::endl;

    return 0;

}

