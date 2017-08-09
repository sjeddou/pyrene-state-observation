#include <unsupported/Eigen/MatrixFunctions> //Pour utiliser l'exponentielle d'une matrice
#include <iostream>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

using namespace stateObservation ;
using namespace Eigen;

VectorXd compute(const VectorXd& x, Vector3d omega, int q_0j_size, int q_ft_size, float T, int num_iter){


    int q_fj_size=10;



    VectorXd q_0j(q_0j_size), q_0j_first(q_0j_size), q_0j_second(q_0j_size);
    VectorXd q_ft(q_ft_size), q_ft_first(q_ft_size), q_ft_second(q_ft_size);
    VectorXd q_fr(3), q_fr_first(3), q_fr_second(3);

    VectorXd q_fj(q_fj_size), q_fj_first(q_fj_size), q_fj_second(q_fj_size);




    assert(omega.size() == 3);

    MatrixXd A_exp(3,3), B_exp(3,3);
    A_exp(0,1) = -omega(2);
    A_exp(0,2) = omega(1);
    A_exp(1,2) = -omega(0);



    A_exp -= A_exp.transpose().eval();
    A_exp = A_exp.exp().eval();


    MatrixXd omega_T(1,3);


    omega_T(0,0) = omega(0);
    omega_T(0,1) = omega(1);
    omega_T(0,2) = omega(2);

    VectorXd xk_1=x;




    for(int i=0; i<num_iter; i++){


        q_0j = x.segment(0, q_0j_size);
        q_0j_first = x.segment(q_0j_size, q_0j_size);
        q_0j_second.segment(0,q_0j_size-1) = q_0j.segment(1,q_0j_size-1).eval()-q_0j.segment(0,q_0j_size-1).eval();
        q_0j_second.segment(0,q_0j_size-1) = q_0j_second.segment(1,q_0j_size-1).eval()-q_0j_second.segment(0,q_0j_size-1).eval();

        q_ft = x.segment(2*q_0j_size, q_ft_size);
        q_ft_first.segment(0,q_ft_size-1) = q_ft.segment(1,q_ft_size-1).eval()-q_ft.segment(0,q_ft_size-1).eval();
        q_ft_second.segment(0,q_ft_size-1) = q_ft_first.segment(1,q_ft_size-1).eval()-q_ft_first.segment(0,q_ft_size-1).eval();

        q_fr = x.segment(2*q_0j_size+q_ft_size, 3);
        q_fr_first.segment(0,2) = q_fr.segment(1,2).eval()-q_fr.segment(0,2).eval();
        B_exp(0,1) = -q_fr_first(2);
        B_exp(0,2) = q_fr_first(1);
        B_exp(1,2) = -q_fr_first(0);
        B_exp -= B_exp.transpose().eval();
        B_exp = B_exp.exp().eval();


        q_fj= x.segment(2*q_0j_size+q_ft_size+3,q_fj_size);
        q_fj_first.segment(0,q_fj_size-1) =q_fj.segment(1,q_fj_size-1).eval()-q_fj.segment(0,q_fj_size-1).eval();
        q_fj_second.segment(0,q_fj_size-1) =q_fj_first.segment(1,q_fj_size-1).eval()-q_fj_first.segment(0,q_fj_size-1).eval();



        xk_1.segment(0,q_0j_size) = q_0j+T*q_0j_first+0.5*T*T*q_0j_second;
        xk_1.segment(q_0j_size,q_0j_size) = q_0j_first+T*q_0j_second;
        xk_1.segment(2*q_0j_size,q_ft_size) = q_ft+T*q_ft_first+0.5*T*T*q_ft_second;
        xk_1.segment(2*q_0j_size+q_ft_size, 3) = (omega_T*B_exp*A_exp).eval().row(0);
        xk_1.segment(2*q_0j_size+q_ft_size+3,q_fj_size)=q_fj_first+T*q_fj_second;
    }
    return xk_1;
}

int main(){
    VectorXd omega = VectorXd::Random(3);
    int q_0j_size = 32;
    int q_ft_size = 19;

    VectorXd x = VectorXd::Random(2*q_0j_size+32+32);
    std::cout << compute(x, omega, q_0j_size, q_ft_size, 0.5, 100) << std::endl;
    VectorXd xk_1 = compute(x, omega, q_0j_size, q_ft_size, 0.5, 100);
    std::cout << "size of xk_1 is  : "  << xk_1.size() <<std::endl;
}
