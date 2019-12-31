//
// Created by wang on 19-12-10.
//
#include <iostream>
int main(){
    unsigned char* array;
    array = new unsigned char[3*3];
    array[1] = -10;
    int  i = array[1];
    std::cout<<"array[1] = "<<i<<std::endl;
}