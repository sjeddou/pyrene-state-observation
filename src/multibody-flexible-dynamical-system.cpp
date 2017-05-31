#include <iostream>
#include <vector>
#include <pyrene-state-observation/pyrene-flexibility-estimation/multibody-flexible-dynamical-system.hpp>
#include <stdexcept>

using namespace std;
using namespace Eigen;
namespace pyreneStateObservation
{
namespace pyreneFlexibilityEstimation
{
    using namespace pyreneStateObservation;

   MultibodyFlexibleDynamicalSystem::MultibodyFlexibleDynamicalSystem():
       robotMass_(pyrene::m),
       robotMassInv_(1/pyrene::m)
   {
       Kpj_=40000*Matrix3f::Identity();
       Kdj_=600*Matrix3f::Identity();

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




   /*VectorXf MultibodyFlexibleDynamicalSystem::getMomentaFromForces(const VectorXf& x, const VectorXf& u)
   {
           // Getting joint configuration
           op_.jointPosition=x.segment(state::jointPos,3);
           op_.jointVelocity=x.segment(state::jointVel,3);
           // Getting flexibility
           op_.positionFlex=x.segment(state::flexConf,3);
           op_.FlexV=x.segment(state::flexConfDot,3);
           // Getting contact forces
           for (int i=0; i<pyrene::contact::nbModeledMax; ++i)
           op_.efforts.setValue(x.segment(state::contactForcesc+6*i,6),i);

           op_.fm=x.segment(state::unmodeledForces,3);
           op_.tm=x.segment(state::unmodeledForces+3,3);
           for (int i=0; i<pyrene::contact::nbModeledMax; ++i)
                  {
                      fc_.segment<3>(3*i) = op_.efforts[i].block<3,1>(0,0);
                      tc_.segment<3>(3*i) = op_.efforts[i].block<3,1>(3,0);
                  }

            // Getting contact positions
            unsigned nbContacts(getContactsNumber());
            for (unsigned i = 0; i<nbContacts ; ++i)
                  {
                    // Positions
                    op_.contactPosV.setValue(u.segment<3>(input::contacts + 12*i),i);
                    op_.contactOriV.setValue(u.segment<3>(input::contacts +12*i+3),i);
                  }

            computeContactWrench(op_.rFlex, op_.positionFlex, op_.contactPosV, op_.contactOriV,fc_, tc_, op_.fm, op_.tm);

                  op_.momenta.segment<3>(0) = op_.f - hrp2::m*cst::gravity;
                  op_.momenta.segment<3>(3) = op_.t - kine::skewSymmetric(op_.rFlex*op_.positionCom+op_.positionFlex)*hrp2::m*cst::gravity;

                  return op_.momenta;

   }
*/



    ///State Dynamics
   /*Vector MultibodyFlexibleDynamicalSystem::stateDynamics(const Vector& x, const Vector& u, unsigned k)
        {
            assertStateVector_(x);
            assertInputVector_(u);

            xk_=x;
            uk_=u;

            op_.positionJoint=x.segment(state::jointPos,3);
            op_.jointVelocity=x.segment(state::jointVel,3);
            op_.positionFlex=x.segment(state::flexConf,3);
            op_.velocityFlex=x.segment(state::flexConfDot,3);

            unsigned nbContacts(getContactsNumber());

            for (unsigned i = 0; i<nbContacts ; ++i)
            {// Positions
                op_.contactPosV.setValue(u.segment<3>(input::contacts + 12*i),i);
                op_.contactOriV.setValue(u.segment<3>(input::contacts +12*i+3),i);

                op_.contactVelArray.setValue(u.segment<3>(input::contacts + 12*i +6),i);
                op_.contactAngVelArray.setValue(u.segment<3>(input::contacts +12*i +9),i);

              }

              for (int i=0; i<hrp2::conrobotMass_(hrp2::m),
        robotMassInv_(1/hrp2::m),tact::nbModeledMax; ++i)
              {
                  fc_.segment<3>(3*i) = op_.efforts[i].block<3,1>(0,0);
                  tc_.segment<3>(3*i) = op_.efforts[i].block<3,1>(3,0);
              }

              const int subsample=1;
              for (int i=0; i<subsample; ++i)
              {
                iterateDynamicsEuler (

                                op_.contactPosV, op_.contactOriV,
                                op_.positionFlex, op_.velocityFlex,
                                op_.orientationFlexV, op_.angularVelocityFlex,
                                tc_, op_.fm, op_.tm,
                                dt_/subsample);
              }
              //x_{k+1}

              for (int i=0; i<hrp2::contact::nbModeledMax; ++i)
              {
                  op_.efforts[i].block<3,1>(0,0) = fc_.segment<3>(3*i);
                  op_.efforts[i].block<3,1>(3,0) = tc_.segment<3>(3*i);
              }

              xk1_=x;

              xk1_.segment<3>(state::pos) = op_.positionFlex;
              xk1_.segment<3>(state::ori) =  op_.orientationFlexV;

              xk1_.segment<3>(state::linVel) = op_.velocityFlex;
              xk1_.segment<3>(state::angVel) = op_.angularVelocityFlex;

              for (int i=0; i<hrp2::contact::nbModeledMax; ++i)
                  xk1_.segment(state::fc+6*i,6) = op_.efforts[i].col(0);

              // xk1_.segment<2>(state::comBias) = op_.positionComBias.head<2>();

              xk1_.segment<3>(state::unmodeledForces) = scallingFactor_*op_.fm;
              xk1_.segment<3>(state::unmodeledForces+3) = scallingFactor_*op_.tm;

              if (processNoise_!=0x0)
                  return processNoise_->addNoise(op_.xk1);
              else
                  return xk1_;
          }

        */


}
}
