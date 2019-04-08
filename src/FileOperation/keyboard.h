//
// Created by wang on 19-3-30.
//

#ifndef TDLIB_KEYBOARD_H
#define TDLIB_KEYBOARD_H

#include <iostream>
namespace td{
    void keyboardInputRecognition(char key, void (*function)()) {
        bool continueInput = true;
        while (continueInput)
        {
            std::cout<<"enter y to continue"<<std::endl;
            char key;
            std::cin>>key;
            if('y' == key)
            {
                function();// do what you want
                std::cout<<"y is pushed"<<std::endl;
                continueInput = false;
            } else
                std::cout<<"Input error, please input again"<<std::endl;
        }
    }
}

#endif //TDLIB_KEYBOARD_H
