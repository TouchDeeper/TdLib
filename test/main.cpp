#include <iostream>
#include "TdLibrary/realsense.h"
#include "TdLibrary/threadSafeStructure.h"
#include <thread>
#include <atomic>
using namespace std;
int main() {
    td::hello();
    td::rs::helloz();
    td::threadsafe_queue<int> q1;

    std::thread t2([&](){
        std::cout<<"thread t2 is created"<<std::endl;
        int j = 0;
        int num = 100;
        while(!q1.empty() || num > 0)
        {
            q1.wait_and_pop(j);
            std::cout<<"thread t2 pop value "<<j<<std::endl;
            num--;
        }
    });

    for(int i = 0; i < 100; i++)
        {
            q1.push(i);
            std::cout<<" main thread push value "<<i<<std::endl;
        }
    t2.join();
}