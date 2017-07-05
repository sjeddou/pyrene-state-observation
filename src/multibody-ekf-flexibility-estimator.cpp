#include <pyrene-state-observation/pyrene-flexibility-estimation/multibody-ekf-flexibility-estimator.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

using namespace std;
using namespace stateObservation;

const double dxFactor = 1.0e-8;
const int stateSize = 35;

namespace pyreneStateObservation
{
namespace pyreneFlexibilityEstimation
{

/// Create an alias for State
typedef MultibodyFlexibleDynamicalSystem::state state;
///Constructor
MultibodyEKFFlexibilityEstimator::MultibodyEKFFlexibilityEstimator(double dt)
{
    cout <<"Strating" <<endl;
}
///Destructor
MultibodyEKFFlexibilityEstimator::~MultibodyEKFFlexibilityEstimator()
{

}






}

}
