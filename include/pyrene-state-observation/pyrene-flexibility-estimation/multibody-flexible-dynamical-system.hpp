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
          ///indexes of different components of a state vector
              static const unsigned jointConf = 0;
              static const unsigned jointVel = 6;
              static const unsigned flexConf = 12;
              static const unsigned flexVel = 18;
              static const unsigned jointTorque =24;
              static const unsigned contactForces =30;

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



      /// Description of the state dynamics
      //virtual std::vector<long> stateDynamics(const stateObservation::vector& x, const stateObservation::vector& u,unsigned k);
      //virtual VectorXd stateDynamics(const pyreneStateObservation::VectorXd& x, const pyreneStateObservation::VectorXd& u,unsigned k);
      virtual stateObservation::Vector stateDynamics(const VectorXd& x, const VectorXd& u,unsigned k);

    private:

    protected:

      double robotMass_;
      double robotMassInv_;
      double dt_;
      unsigned contactModel_;
      static const unsigned stateSize_=26;
      Matrix3d Kpj_, Kdj_;
      unsigned nbContacts_;
      Matrix3d& computeRotation_(const Vector3d & x, int i);

      VectorXd xk_;
      VectorXd uk_;
      struct Optimization

      {     ///StateDyanmics

                  Vector3d positionFlex;
                  Vector3d orientationFlex;
                  Vector3d velocityFlex;
                  Vector3d angularVelocityFlex;
                  stateObservation::IndexedMatrixArray efforts;
                  stateObservation::IndexedMatrixArray contactPosV;
                  stateObservation::IndexedMatrixArray contactOriV;
                  stateObservation::IndexedMatrixArray contactVelArray;
                  stateObservation::IndexedMatrixArray contactAngVelArray;



            ///ComputeRotation

                  Matrix3d curRotation0;
                  Vector3d orientationVector0;
                  Matrix3d curRotation1;
                  Vector3d orientationVector1;
                  Matrix3d curRotation2;
                  Vector3d orientationVector2;
                  Matrix3d curRotation3;
                  Vector3d orientationVector3;

                  Optimization()
                    :
                    curRotation0(Matrix3d::Identity()),
                    orientationVector0(Vector3d::Zero()),
                    curRotation1(Matrix3d::Identity()),
                    orientationVector1(Vector3d::Zero()),
                    curRotation2(Matrix3d::Identity()),
                    orientationVector2(Vector3d::Zero()),
                    curRotation3(Matrix3d::Identity()),
                    orientationVector3(Vector3d::Zero())
                  {}

                  inline Vector3d& orientationVector(int i)
                  {
                    if (i==0)
                      return orientationVector0;
                    if (i==1)
                      return orientationVector1;
                    if (i==2)
                      return orientationVector2;

                    return orientationVector3;
                  }

                  inline Matrix3d& curRotation(int i)
                  {
                    if (i==0)
                      return curRotation0;
                    if (i==1)
                      return curRotation1;
                    if (i==2)
                      return curRotation2;

                    return curRotation3;
                  }


      }op_;



  };

    }
}

#endif // MULTIBODYFLEXIBLEDYNAMICALSYSTEM_HPP_
