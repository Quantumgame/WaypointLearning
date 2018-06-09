#ifndef OR_OMPL_SAMPLER_H_
#define OR_OMPL_SAMPLER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <boost/make_shared.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
/// @cond IGNORE
// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.
class MyValidStateSampler : public ob::ValidStateSampler
{
public:
    MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
    {
        name_ = "my sampler";
    }

    bool sample(ob::State *state) override
    {
        double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values; //val[0],val[1],val[2] contain the x,y,z coordinates of the sample taken
        
        switch(rng_.uniformInt(0,1))
        {
            case 0: // normal sample
                val[0] = rng_.uniformReal(-2.41, 2.41); // joint hor limits here
                val[1] = rng_.uniformReal(-2.41, 2.41); // joint vert limits here
                val[2] = rng_.uniformReal(-180, 180); // joint rot limits here
                 
            case 1: // new sample
                // open file containing green regions, top line is a counter indicating the number of times the file was accessed (i.e. samples used) *OR* file of samples from these regions
                // pick random green pixel from list and get a random sample using the pixel bounds in openrave coordinates 
                std::string line;
                std::ifstream sampleFile ("samples.txt");
                std::ifstream gpFile ("numgreenpixels.txt");
                if(sampleFile.is_open() && gpFile.is_open())
                {
                    std::getline(gpFile, line); // first line contains the number of green pixels
                    int numGreenPixels = atoi(line.c_str());
                    int i = 1;
                    int selection = rng_.uniformInt(i, numGreenPixels); // randomly choose a pixel

                    while(std::getline(sampleFile, line))
                    {
                        if(i == selection)
                        {
                            // parse line by whitespace to obtain pixel x-bounds and y-bounds in OR coordinates
                            std::vector<float> bounds; // minx miny maxx maxy
                            std::istringstream iss(line);

                            for(float s; iss >> s; )
                                bounds.push_back(s);

                            val[0] = rng_.uniformReal(bounds[0], bounds[2]);
                            val[1] = rng_.uniformReal(bounds[1], bounds[3]);
                            val[2] = rng_.uniformReal(-180, 180);

                            break;
                        }

                        i += 1;
                    }

                    sampleFile.close();
                    gpFile.close();
                }
                else
                    std::cout << "\nUnable to open files";
        }

        assert(si_->isValid(state));
        return true;
    }


    // We don't need this in the example below.
    bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override
    {
        throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
        return false;
    }
protected:
    ompl::RNG rng_;
};

ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si)
{
    return boost::make_shared<MyValidStateSampler>(si);
}

#endif // OR_OMPL_SAMPLER_H_