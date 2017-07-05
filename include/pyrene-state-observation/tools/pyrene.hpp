#ifndef PYRENECONSTANTS
#define PYRENECONSTANTS



#include <vector>
#include <deque>

#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
#include <iostream>
#endif

#include <boost/assert.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <deque>

#include <fstream>




namespace pyreneStateObservation
{

    namespace pyrene
    {
        /// mass of the robot
        const double m=70;

        /// stifness and damping
        const double linKe=40000;
        const double angKe=400;
        const double linKv=600;
        const double angKv=10;

        struct contact
        {
        static const unsigned nbMax=4;
        static const unsigned nbModeledMax=2;
        // index for the contacts
        static const unsigned lf = 0;
        static const unsigned rf = 1;
        static const unsigned lh = 2;
        static const unsigned rh = 3;
        };






    }

}

#endif //PYRENECONSTANTS

