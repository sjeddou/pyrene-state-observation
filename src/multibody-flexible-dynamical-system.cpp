#include <iostream>
#include <Eigen/Cholesky>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/rnea.hpp"

#include <pyrene-state-observation/pyrene-flexibility-estimation/multibody-flexible-dynamical-system.hpp>
#include <stdexcept>



using namespace Eigen;

using namespace stateObservation;
namespace pyreneStateObservation
{
namespace pyreneFlexibilityEstimation
{






   MultibodyFlexibleDynamicalSystem::MultibodyFlexibleDynamicalSystem():
       robotMass_(pyrene::m),
       robotMassInv_(1/pyrene::m)
   {
       Kpj_=pyrene::linKe*Matrix3d::Identity();
       Kdj_=pyrene::linKv*Matrix3d::Identity();

   }
   MultibodyFlexibleDynamicalSystem::~MultibodyFlexibleDynamicalSystem()
   {
   }
   void MultibodyFlexibleDynamicalSystem::Start()
   {
   std::cout <<"Starting" <<std::endl;
   }
   void MultibodyFlexibleDynamicalSystem::setRobotMass(double m)
   {
      robotMass_ = m;
      robotMassInv_=1/m;
   }

   double MultibodyFlexibleDynamicalSystem::getRobotMass() const
   {
      return robotMass_;
   }

   void MultibodyFlexibleDynamicalSystem::setContactModel(unsigned nb)
   {
        contactModel_=nb;
   }
   void MultibodyFlexibleDynamicalSystem::setSamplingPeriod(double dt)
   {
        dt_=dt;
   }

   unsigned MultibodyFlexibleDynamicalSystem::getStateSize() const
   {
       return stateSize_;
   }

   inline Matrix3 &MultibodyFlexibleDynamicalSystem::computeRotation_(const Vector3 & x, int i)
   {
        Vector3d & oriV = op_.orientationVector(i);
        Matrix3d & oriR = op_.curRotation(i);
        if (oriV!=x)
           {
               oriV = x;
               oriR = stateObservation::kine::rotationVectorToAngleAxis(x).toRotationMatrix(); //error line
           }

        return oriR;
   }
   ///The dynamics of actuators


    Vector MultibodyFlexibleDynamicalSystem::actuatorDynamics(const Vector& x, const Vector& u, unsigned k)
   {
        assertStateVector_(x);
        assertInputVector_(u);
        xk_=x;
        uk_=u;



        ///Getting Inertia Matrix with data.M : computed with Composite Rigid Body Algorithm (CRBA)

        /// Inverse of Matrix M : data.M.inverse()
        const std::string filename = "/opt/openrobots/share/talos_data/robots/talos_reduced.urdf";
        se3::Model model;
        se3::urdf::buildModel(filename, model);
        se3::Data data(model);
        data.M.fill(0);
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
        se3::crba(model,data,q);
        Eigen::MatrixXd M(model.nv,model.nv);

        ///Getting Non linear interia matrix
        se3::Data data_rnea(model);
    	data_rnea.v[0]= se3::Motion::Zero();
    	data_rnea.a[0]= - model.gravity;

        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    	Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd tau_rnea=Eigen::VectorXd::Zero(model.nv);

        tau_rnea=se3::rnea(model,data_rnea,q,v,a);

        ///Initializing estimation
 
        op_.qfddot_init=VectorXd::Zero(model.nq);


        //qfddot = - q0ddot-(guz 0 0)- inv(Inertia)*B (
        op_.qfddot=op_.qfddot_init+pyrene::m*Vector3(9.8,0,0)-data.M;




       xk1_=x;

       ///Description of state vector

       xk1_.segment<1>(state::jointConf)=op_.positionJoint;
       xk1_.segment<1>(state::jointVel)=op_.positionJointDot;


       xk1_.segment<1>(state::linFlexConf)=op_.confFlex.segment<1>(0);
       xk1_.segment<2>(state::angFlexConf)=op_.confFlex.segment<2>(1);
       xk1_.segment<1>(state::linFlexVel)=op_.confVelFlex.segment<1>(0);
       xk1_.segment<2>(state::angFlexVel)=op_.confVelFlex.segment<2>(1);
       xk1_.segment<3>(state::jointTorque)=op_.t;
       xk1_.segment<3>(state::contactForces)=op_.f;





       
   }




   
/*
//Implement state dynamic equations

   Vector MultibodyFlexibleDynamicalSystem::stateDynamics(const Vector& x,const Vector& u, const Vector& omega, int q_0j_size, int q_ft_size, float T, int num_iter)
        {
            assertStateVector_(x);
            assertInputVector_(u);

            xk_=x;
            uk_=u;


            Vector q_0j(q_0j_size), q_0j_first(q_0j_size), q_0j_second(q_0j_size);
            Vector q_ft(q_ft_size), q_ft_first(q_ft_size), q_ft_second(q_ft_size);
            Vector q_fr(3), q_fr_first(3), q_fr_second(3);

                assert(q_ft_size < 29);
                int q_fj_size;
                q_fj_size = 29 - q_ft_size;
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

                    x.segment(0,q_0j_size) = q_0j+T*q_0j_first+0.5*T*T*q_0j_second;
                    x.segment(q_0j_size,q_0j_size) = q_0j_first+T*q_0j_second;
                    x.segment(2*q_0j_size,q_ft_size) = q_ft+T*q_ft_first+0.5*T*T*q_ft_second;
                    x.segment(2*q_0j_size+q_ft_size, 3) = (omega_T*B_exp*A_exp).eval().row(0);
                }
                return x;


         }
*/

   
   
   
   
}
}
