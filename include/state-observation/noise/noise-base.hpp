/**
 * \file      noise-base.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief
 */
#include "../tools/definitions.hpp"
#include<iostream>
#ifndef SENSORSIMULATIONNOISEBASEHPP
#define SENSORSIMULATIONNOISEBASEHPP



//namespace stateObservation
//{

    /**
     * \Abstract class  NoiseBase
     * \brief
     *
     * \details
     *
     */

    class NoiseBase
    {
    public:
        /// Virtual destructor
        virtual ~NoiseBase(){std::cout<<"Base Destructor called\n";}

        /// The method to overload to add a noise to a given vector
        virtual Vector addNoise(const Vector &)=0;


    protected:

    };

//}

#endif //SENSORSIMULATIONNOISEBASEHPP
