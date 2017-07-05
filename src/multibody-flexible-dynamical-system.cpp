#include <iostream>
#include <vector>
#include <pyrene-state-observation/pyrene-flexibility-estimation/multibody-flexible-dynamical-system.hpp>

#include <stdexcept>

using namespace std;
using namespace Eigen;
using namespace stateObservation;
namespace pyreneStateObservation
{
namespace pyreneFlexibilityEstimation
{



///Actuator's Dynamics



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
   cout <<"Starting" <<endl;
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
   {   assertStateVector_(x);
       assertInputVector_(u);
        xk_=x;
        uk_=u;


       ///Getting flexibility estimation

       
        /// op_.f : model of the initial state
       
        op_.flexDynamics.segment<3>(0)=op_.f+pyrene::m*Vector3(9.8,0,0)-stateObservation::Matrix::Identity(3,3)*stateObservation::Matrix::Identity(3,3);

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
