//
// Created by wang on 19-6-6.
//

#ifndef TDLIB_RANDOM_TOOL_H
#define TDLIB_RANDOM_TOOL_H

#include <iostream>
#include <random>
namespace td{
    /**
     * get a random value between 0 and 1
     * @param p_success Probability of success
     * @return a random value between 0 and 1
     */
    int BernouliSampling(float p_success);
    /**
     * get a random value between 0 and 1
     * @param p_success Probability of success
     * @return a random value between 0 and 1
     */
    bool BernouliSamplingBool(float p_success);

    /**
    * get a random value sampling from normal distribution
    * @param expected
    * @param sigma
    * @return a random value sampling from normal distribution
    */
    double NormalSampling(double expected, double sigma);
}//namespace td
#endif //TDLIB_RANDOM_TOOL_H
