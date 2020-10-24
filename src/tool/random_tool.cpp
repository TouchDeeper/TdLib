//
// Created by wang on 19-6-6.
//
#include "random_tool.h"
namespace td{
    /**
     * get a random value between 0 and 1
     * @param p_success Probability of success
     * @return a random value between 0 and 1
     */
    int BernouliSampling(float p_success)
    {
        std::random_device rd;
        std::default_random_engine rng {rd()};
        std::bernoulli_distribution bernoulli_d {p_success};
        return bernoulli_d(rng);
    }
    /**
     * get a random value between 0 and 1
     * @param p_success Probability of success
     * @return a random value between 0 and 1
     */
    bool BernouliSamplingBool(float p_success)
    {
        std::random_device rd;
        std::default_random_engine rng {rd()};
        std::bernoulli_distribution bernoulli_d {p_success};
        std::cout<<std::boolalpha;
        return bernoulli_d(rng);
    }

    /**
     * get a random value sampling from normal distribution
     * @param expected
     * @param sigma
     * @return a random value sampling from normal distribution
     */
    double NormalSampling(double expected, double sigma) {
        std::normal_distribution<double> norm(expected, sigma);
        std::random_device rd;
        std::default_random_engine rng{rd()};
        return norm(rng);
    }

}//namespace td
