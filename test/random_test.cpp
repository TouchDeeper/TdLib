//
// Created by wang on 19-6-5.
//
#include "TdLibrary/random_tool.h"
#include "TdLibrary/random_tool.hpp"
#include <stdlib.h>
int main(){
    for (int i = 0; i < 10; ++i) {
        std::cout<<td::BernouliSampling(0.5)<<" ";
    }
    std::cout<<std::endl;
    for (int i = 0; i < 10; ++i) {
        std::cout<<td::BernouliSamplingBool(0.5)<<" ";
    }
    std::cout<<std::endl;
    for (int j = 0; j < 10; ++j) {
        std::cout<<td::NormalSampling(50,10)<<" ";
    }
    std::cout<<std::endl;
    for (int k = 0; k < 10; ++k) {
//        int min = 0;
//        int max = 5;
        std::cout<<int(td::UniformSampling<float>(0,5+1))<<" ";
    }
//    for (int l = 0; l < 10; ++l) {
//        int min = 0;
//        int max = 5;
//        auto randnum = (rand()%(max-min)) + min;
//        std::cout<<randnum
//    }
}