#include <iostream>
#include <thread>
#include "dllexport.h"
#include <Environment/environment.h>
#include <cstdio>
using namespace std;
int main(){
    std::vector<double> temp{0.0,0.0,0.0};
    Environment env(10666);
    env.start_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    env.reset();
    while(true){
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        env.step(temp);
    }
	return 0;
}
