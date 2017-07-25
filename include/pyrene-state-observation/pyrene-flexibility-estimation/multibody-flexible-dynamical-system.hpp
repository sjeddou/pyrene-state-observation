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
using namespace stateObservation;

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
              static const unsigned jointConf = 0; //joint configuration (angle) before flexibility
              static const unsigned jointVel = 1;  //derivative of joint configuration before flexibility
              static const unsigned linFlexConf = 4; //linear component of flexibility configuration
              static const unsigned angFlexConf = 5; //angular component of flexibility configuration
              static const unsigned linFlexVel = 6;
              static const unsigned angFlexVel = 7;
              static const unsigned jointTorque = 12; 
              static const unsigned contactForces =15;

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



      ///The dynamics of actuators


      virtual Vector actuatorDynamics(const Vector& x, const Vector& u, unsigned k);
         






      /// Description of the state dynamics

      /* virtual Vector stateDynamics(const VectorXd& x, const VectorXd& u,unsigned k);*/









      /// Description of measurment


    private:

    protected:

      double robotMass_;
      double robotMassInv_;
      double dt_;
      unsigned contactModel_;
      static const unsigned stateSize_=26;
      Matrix3  Kpj_, Kdj_;
      unsigned nbContacts_;
      Matrix3& computeRotation_(const Vector3 &x, int i);

      VectorXd xk_;
      VectorXd uk_;
      Vector xk1_;
      struct Optimization

      {
          ///Actuator Dynamics

          VectorXd qfddot ;
          VectorXd qfddot_init;



          ///State Vector
          	  Vector positionJoint;
          	  Vector positionJointDot;
		  Vector3 confFlex;
		  Vector3 confVelFlex;
		  Vector3 f;
                  Vector3 t;



            ///ComputeRotation

                  Matrix3 curRotation0;
                  Vector3 orientationVector0;
                  Matrix3 curRotation1;
                  Vector3 orientationVector1;
                  Matrix3 curRotation2;
                  Vector3 orientationVector2;
                  Matrix3 curRotation3;
                  Vector3 orientationVector3;

                  Optimization()
                    :
                    curRotation0(Matrix3::Identity()),
                    orientationVector0(Vector3::Zero()),
                    curRotation1(Matrix3::Identity()),
                    orientationVector1(Vector3::Zero()),
                    curRotation2(Matrix3::Identity()),
                    orientationVector2(Vector3::Zero()),
                    curRotation3(Matrix3::Identity()),
                    orientationVector3(Vector3::Zero())
                  {}

                  inline Vector3& orientationVector(int i)
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
