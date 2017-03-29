/**
 * \file      probability-law-simulation.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief
 *
 *
 *
 */


#ifndef PROBABILITYLAWSIMULATION_HPP
#define PROBABILITYLAWSIMULATION_HPP

#include <boost/random.hpp>

#include "definitions.hpp"


namespace stateObservation
{
    namespace tools
    {
        class ProbabilityLawSimulation
        {
        public:

            ///gets White Gaussian Noise
            ///having a given bias and standard deviation(std)
            static Matrix getWGNoise( const Matrix & std, const Matrix & bias,
                unsigned rows, unsigned cols=1);

        protected:
            static boost::lagged_fibonacci1279 gen_;

        };

    }
}



#endif //PROBABILITYLAWSIMULATION_HPP
