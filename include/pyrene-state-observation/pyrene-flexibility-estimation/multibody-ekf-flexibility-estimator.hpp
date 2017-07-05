#ifndef MULTIBODYEKFFLEXIBILITYESTIMATOR_H
#define MULTIBODYEKFFLEXIBILITYESTIMATOR_H

#include <state-observation/flexibility-estimation/ekf-flexibility-estimator-base.hpp>
#include <pyrene-state-observation/pyrene-flexibility-estimation/multibody-flexible-dynamical-system.hpp>

using namespace stateObservation;

namespace pyreneStateObservation
{
namespace pyreneFlexibilityEstimation
{

class MultibodyEKFFlexibilityEstimator :
                                        public stateObservation::flexibilityEstimation::FlexibilityEstimatorBase,
                                        private boost::noncopyable

{
public:
    ///The constructor : requires the value of the sampling period
    explicit MultibodyEKFFlexibilityEstimator(double dt=0.005);

    /// Virtual destructor
    virtual ~MultibodyEKFFlexibilityEstimator();

    /// Gets an estimation of the flexibility in the form of a homogeneous matrix
    virtual Matrix4 getFlexibility();





protected :




private:

};




}

}


#endif
