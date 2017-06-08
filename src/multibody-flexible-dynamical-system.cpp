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
       Kpj_=40000*Matrix3d::Identity();
       Kdj_=600*Matrix3d::Identity();

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

   inline Matrix3d& MultibodyFlexibleDynamicalSystem::computeRotation_(const Vector3d & x, int i)
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


 /// State dynamics & computeRotation
   VectorXd MultibodyFlexibleDynamicalSystem::stateDynamics(const VectorXd& x, const VectorXd& u, unsigned k)
        {
            assertStateVector_(x);
            assertInputVector_(u);

            xk_=x;
            uk_=u;
            op_.positionFlex=x.segment(state::flexConf,12);
            op_.orientationFlex=x.segment(state::flexConf,15);
            op_.velocityFlex=x.segment(state::flexVel,18);
            op_.angularVelocityFlex=x.segment(state::flexVel,21);

            for (int i=0; i<pyrene::contact::nbModeledMax; ++i)
                        op_.efforts.setValue(x.segment(state::contactForces+6*i,6),i);



         }


}
}
