//
// Created by wang on 19-11-19.
//

#include <list>
#include <iostream>
#include <algorithm>
#include <vector>

int main(int argc, char **argv){

    // create list
    std::list<int> cor;
    for (int i = 0; i < 6; ++i) {
        cor.push_back(i);
    }
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_real_distribution<float> distribution(0, 5);

    // remove items when iterating the list
    int remove_num = lround(distribution(rng));
    std::cout<<"now size = "<<cor.size()<<std::endl;
    for(auto iter = cor.begin();iter != cor.end(); ){
        int index = *iter;
        std::cout<< "index = "<<index<<std::endl;
        auto i = std::find(cor.begin(),cor.end(),remove_num);
        if(i == cor.begin())
        {
            std::cout<<"now remove " <<*i<<std::endl;
            iter++;
            cor.remove(*i);
            std::cout<<"now size = "<<cor.size()<<std::endl;

        }
        else
            if(i != cor.end())
            {
                std::cout<<"now remove " <<remove_num<<std::endl;
                auto back_iter = iter;
                back_iter++;
                cor.remove(remove_num);
                if(i == iter)
                    iter = back_iter;
                else
                    iter++;
                std::cout<<"now size = "<<cor.size()<<std::endl;
            } else
                iter++;
        remove_num = lround(distribution(rng));
    }


    std::list<int> test;
    for (int i = 0; i < 6; ++i) {
        test.push_back(i);
    }

//    auto iter_test = test.begin();
////    for (const auto &item : test) {
////        std::cout<<item<<std::endl;
////    }
//    iter_test ++;
//    *iter_test = -1;
//    std::cout<<*iter_test<<std::endl;
//    for (const auto &item : test) {
//        std::cout<<item<<std::endl;
//    }
//    for (auto it = test.begin(); it != test.end() ; ++it) {
//        *it = -1;
//    }
    std::vector<int> t;
    for (int j = 0; j < 6; ++j) {
        t.push_back(j);
    }
    t.resize(3);
    for (const auto &item : t) {
        std::cout<<item<<std::endl;
    }
    return 0;
}