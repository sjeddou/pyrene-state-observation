#include <iostream>
#include <Eigen/Cholesky>
#include <vector>


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

 /// State dynamics & computeRotation & IterateDynamicsEuler
   VectorXd MultibodyFlexibleDynamicalSystem::stateDynamics(const VectorXd& x, const VectorXd& u, unsigned k)
        {
            assertStateVector_(x);
            assertInputVector_(u);

            xk_=x;
            uk_=u;
            //Getting flexibility
            op_.positionFlex=x.segment(state::flexConf,12);
            op_.orientationFlex=x.segment(state::flexConf,15);
            op_.velocityFlex=x.segment(state::flexVel,18);
            op_.angularVelocityFlex=x.segment(state::flexVel,21);

                
            for (int i=0; i<pyrene::contact::nbModeledMax; ++i)
                        op_.efforts.setValue(x.segment(state::contactForces+6*i,6),i);

            unsigned nbContacts(getContactsNumber());

            for (unsigned i = 0; i<nbContacts ; ++i)
            {
             // Positions
             op_.contactPosV.setValue(u.segment<3>(input::contacts + 12*i),i);
             op_.contactOriV.setValue(u.segment<3>(input::contacts +12*i+3),i);

             // Velocities
             op_.contactVelArray.setValue(u.segment<3>(input::contacts + 12*i +6),i);
             op_.contactAngVelArray.setValue(u.segment<3>(input::contacts +12*i +9),i);

             //
             
            }

         }
*/

   
   
   
   
}
}
