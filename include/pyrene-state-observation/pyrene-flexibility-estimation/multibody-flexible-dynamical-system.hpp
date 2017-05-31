#ifndef MULTIBODYFLEXIBLEDYNAMICALSYSTEM_HPP_
#define MULTIBODYFLEXIBLEDYNAMICALSYSTEM_HPP_
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <pyrene-state-observation/tools/pyrene.hpp>
#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include <state-observation/dynamical-system/algorithm/rigid-body-kinematics.hpp>
using namespace std;
using namespace Eigen;

namespace pyreneStateObservation
{
    namespace pyreneFlexibilityEstimation
    {
  class MultibodyFlexibleDynamicalSystem:
        public stateObservation::DynamicalSystemFunctorBase,
        private stateObservation::algorithm::RigidBodyKinematics
 {
  public:
      //Constructor
      explicit MultibodyFlexibleDynamicalSystem();
      //Destructor
      virtual ~MultibodyFlexibleDynamicalSystem();
      //Starting function
      virtual void Start();




      struct input
            {
            ///indexes of the different components of an input vector
              //vector of contact positions/orientations
              //typedef vector<double> contacts;
             static const unsigned current = 0;
             static const unsigned contacts = 1;
            };
      struct state
            {
          ///indexes of different components of a satet vector
              static const unsigned jointPos = 0;
              static const unsigned jointVel = 3;
              static const unsigned flexConf = 6;
              static const unsigned flexConfDot = 9;
              static const unsigned jointTorque = 12;
              static const unsigned contactForces =  24;

            };
      struct contactModel
      {
        /// Indexes of the different components of a vector of the input state
        static const unsigned elasticContact= 1;
       };

      virtual void setRobotMass(double m);

      virtual double getRobotMass() const;

      virtual void setContactModel(unsigned nb);

      virtual void setSamplingPeriod(double dt);

      ///Gets the state vector Size
      virtual unsigned getStateSize() const;

      ///Gets the number of contacts
            inline unsigned getContactsNumber(void) const
            {
               return  nbContacts_;
            }




      //virtual VectorXf getMomentaFromForces(const VectorXf& x, const VectorXf& u);


      /// Description of the state dynamics
     // virtual std::vector<long> stateDynamics(const stateObservation::vector& x, const stateObservation::vector& u,unsigned k);
     //virtual pyreneStateObservation::vector stateDynamics(const pyreneStateObservation::vector& x, const pyreneStateObservation::vector& u,unsigned k);

    private:

    protected:

      double robotMass_;
      double robotMassInv_;
      double dt_;
      unsigned contactModel_;
      static const unsigned stateSize_=26;

      Matrix3f Kpj_, Kdj_;
      VectorXf fc_;
      Vector3f fm, tm ;
      unsigned nbContacts_;





  };

    }
}

#endif // MULTIBODYFLEXIBLEDYNAMICALSYSTEM_HPP_
