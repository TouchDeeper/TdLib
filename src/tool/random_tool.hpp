//
// Created by wang on 19-5-20.
//

#ifndef TDLIB_RANDOM_TOOL_HPP
#define TDLIB_RANDOM_TOOL_HPP

#include <random>
#include <iostream>
namespace td{
    /**
     * get a random value between min and max
     * @tparam T type of data
     * @param min
     * @param max
     * @return a random value between min and max
     */
    template <class T> T UniformSampling(T min, T max)
    {
        std::mt19937 rng;
        rng.seed(std::random_device()());

        std::uniform_real_distribution<T> distribution(min, max);
        return distribution(rng);

    }


} // namespace td

#endif //TDLIB_RANDOM_TOOL_HPP
