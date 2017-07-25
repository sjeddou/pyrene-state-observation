#include <iostream>
#include <Eigen/Cholesky>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"


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

    se3::Data data_rnea(model);
    data_rnea.v[0]= se3::Motion::Zero();
    data_rnea.a[0]= - model.gravity;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    /*q << 4.5602459497473546e-07,
            0.010000798151007295,
            -5.9587344392042496e-08,
            -3.9025709153150956e-07,
            8.810254481517177e-08,
            1.3699828629435318,
            -4.684769811413858e-06,
            -4.62359888153685e-08,
            -0.02999950333221646,
            3.4041550955876687e-07,
            -8.689979227316513e-07,
            -5.134472060852602e-07,
            -1.3699650774613428,
            -5.829689669556437e-06,
            -0.003068088797816962,
            -2.158056195966651e-07,
            0.0024758518651930017,
            -0.0018095237511768225,
            7.644330057843263e-09,
            -1.8380244881655017e-07,
            -8.139998541878413e-07,
            7.730623901025524e-07,
            -9.199855850461347e-07,
            4.3136626626250063e-07,
            -7.300262039322547e-07,
            2.0347067319903545e-07,
            -1.59790459974829e-07,
            2.8465524604115444e-07,
            -8.407964151579961e-07,
            3.7138743191889034e-07,
            -1.1037892040394875e-07,
            7.996485583807236e-07 ; */

   q << 0.00011231249077070515,
           0.010989959422929907,
           0.00132108136622211,
           -0.0030520532869915407,
           0.0003476306248000327,
           1.367927328687765,
           -0.001917047485375356,
           -0.0003531797145370774,
           -0.030010577260628057,
           0.0007093271609104335,
           -0.005362131851583074,
           0.00014425598965095432,
           -1.372145716372477,
           -0.0007670250450800831,
           -0.003068088797816962,
           -2.158056195966651e-07,
           0.005369280515764879,
           -0.0018082986970751058,
           0.00040747129111240454,
           6.3422606180196794e-06,
           -0.0011263724029655364,
           0.002534493870847143,
           -0.0014740209109516123,
           -7.81187019249785e-05,
           -0.0003115145338297154,
           7.774496548229592e-05,
           -0.0007901598962209988,
           0.0016602851606499769,
           0.00167790942832724,
           0.00016168380088554812,
           -4.229485058733235e-05,
          -7.781686682061233e-05;






    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);


    se3::crba(model,data,q);


    std::cout << "nv is :" << model.nv <<std::endl;
    std::cout << "Inertia matrix M : \n" <<data.M <<std::endl;
    std::cout << "The inverse of Matrix M is :\n" <<data.M.inverse() <<std::endl;
    std::cout << "\nVector q is of size : " <<q.size() <<std::endl;

    Eigen::VectorXd tau_rnea = Eigen::VectorXd::Zero(model.nv);
    tau_rnea= se3::rnea(model, data_rnea, q, v, a);


    ///Test matrix B (non linear effects)
    std::cout << "Configuration : \n" << q <<std::endl;

    std::cout << "size of configuration is \n:" << q.size() <<std::endl;


    std::cout << "Tau_rnea is : \n " <<tau_rnea << std::endl;

    std::cout << "size of tau_rnea is \n:" << tau_rnea.size() <<std::endl;


    return 0;

}

