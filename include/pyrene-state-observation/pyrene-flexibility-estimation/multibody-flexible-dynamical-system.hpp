#ifndef MULTIBODYFLEXIBLEDYNAMICALSYSTEM_HPP_
#define MULTIBODYFLEXIBLEDYNAMICALSYSTEM_HPP_
#include<iostream>
#include <vector>
#include <Eigen/Cholesky>


namespace pyreneStateObservation
{
    namespace pyreneFlexibilityEstimation
    {
  class MultibodyFlexibleDynamicalSystem
     /* public stateObservation::DynamicalSystemFunctorBase,
      private stateObservation::algorithm::RigidBodyKinematics*/
 {
  public:
      //Constructor
      MultibodyFlexibleDynamicalSystem();
      //Destructor
      virtual ~MultibodyFlexibleDynamicalSystem();
      virtual void Start();


    private:

    protected:

  };

    }
}

#endif // MULTIBODYFLEXIBLEDYNAMICALSYSTEM_HPP_
