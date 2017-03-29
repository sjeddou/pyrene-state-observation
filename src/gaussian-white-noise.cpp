#include <eigen3/Eigen/Cholesky>
#include <boost/assert.hpp>
#include <boost/static_assert.hpp>
#include "../include/state-observation/tools/definitions.hpp"
#include "../include/state-observation/noise/noise-base.hpp
#include <iostream>
#include "../include/state-observation/noise/gaussian-white-noise.hpp"
#include "../include/state-observation/tools/probability-law-simulation.hpp"
using namespace std;
//namespace stateObservation {




   GaussianWhiteNoise::GaussianWhiteNoise(unsigned dimension):
            dim_(dimension),
            std_(Matrix::Identity(dimension, dimension)),
            bias_(Vector::Zero(dimension,1))
    {
    }

  GaussianWhiteNoise::GaussianWhiteNoise():
            dim_(0)
    {
    }
    ///tools has not been declared
   /*Vector GaussianWhiteNoise::addNoise(const Vector & v)
    {
        checkVector_(v);

        return v+tools::ProbabilityLawSimulation::getWGNoise(std_, bias_,dim_);

    }*/
    void GaussianWhiteNoise::setStandardDeviation(const Matrix & std)
    {
        checkMatrix_(std);
        std_=std;
    }

    void GaussianWhiteNoise::setCovarianceMatrix(const Matrix & cov)
    {
        checkMatrix_(cov);
        Matrix L( cov.llt().matrixL());
        std_=L;
    }

    void GaussianWhiteNoise::setBias(const Vector & bias)
    {
        checkVector_(bias);
        bias_=bias;
    }

    unsigned GaussianWhiteNoise::getDimension() const
    {
        return dim_;
    }

    void GaussianWhiteNoise::setDimension(unsigned dim)
    {
        dim_=dim;
        bias_=Vector::Zero(dim,1);
        std_=Matrix::Identity(dim, dim);
    }

    void GaussianWhiteNoise::checkMatrix_(const Matrix & m) const
    {
        (void)m;//avoid warning
        BOOST_ASSERT(unsigned(m.rows())==dim_ && unsigned(m.cols())==dim_ && "ERROR: Matrix incorrecly dimemsioned");
    }

    void GaussianWhiteNoise::checkVector_(const Vector & v) const
    {
        (void)v;//avoid warning
        BOOST_ASSERT(unsigned(v.rows())==dim_ && unsigned(v.cols())==1 && "ERROR: Vector incorrecly dimemsioned");
    }

//}
