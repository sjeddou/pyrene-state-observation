/**
 * \file      gaussian-white-noise.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief       Defines the class that implements a gaussian white noise
 *
 *
 *
 */
#include "noise-base.hpp"
#ifndef SENSORSIMULATIONGAUSSIANWHITENOISEHPP
#define SENSORSIMULATIONGAUSSIANWHITENOISEHPP

#include "noise-base.hpp"
#include<iostream>
//namespace stateObservation
//{

    /*/**
     * \class  GaussienWhiteNoise
     * \brief The class derivates the NoiseBase class to implement a gaussian
     *          white noise with a given covariance matrix, and bias
     *
     * \details
     *
     */

    class GaussianWhiteNoise : public NoiseBase
    {
    public:

        ///Virtual destructor
        virtual ~GaussianWhiteNoise(){std::cout<<"Derivative Destructor called\n";}

        ///The constructor that provides the dimension of the noise vector
        GaussianWhiteNoise(unsigned dimension);

        ///The default constructor
        GaussianWhiteNoise();

        ///Adds the noise to a given vector it is only an addition of a given vector
        ///and a gaussian white noise
        virtual Vector addNoise(const Vector &);

        ///Sets the standard deviation of the Gaussian white noise.
        /// The covariance matrix is then std*std.transpose()
        virtual void setStandardDeviation(const Matrix & std);

        ///Sets the covariance matrix, the covariance matrix must be positive semi
        ///definite. This method makes a cholesky decomposition to recompute the
        ///standard deviation
        virtual void setCovarianceMatrix(const Matrix & cov);

        ///sets the bias of the white noise
        virtual void setBias(const Vector & bias);

        ///sets the dimension of the noise vector
        virtual void setDimension(unsigned dim_);

        ///Gets the dimension of the noise vector
        virtual unsigned getDimension() const;

    protected:
        virtual void checkMatrix_(const Matrix & m) const ;

        virtual void checkVector_(const Vector & v) const;

        unsigned dim_;

        Matrix std_;

        Vector bias_;

    };

//}

#endif //SENSORSIMULATIONGAUSSIANWHITENOISEHPP
