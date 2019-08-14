//
// Created by wang on 19-6-5.
//
#include "TdLibrary/random_tool.h"
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
}