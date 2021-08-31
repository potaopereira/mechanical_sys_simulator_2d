#include "msinterface/param.hpp"

void IMRB2DPARAM::setParam(std::string paraName, float value){
    map_t & mmap = getFloatParameters();
    if(mmap.count(paraName)!=0){
        float * p = std::get<0>(mmap[paraName]);
        *p = value;
    }
}

float IMRB2DPARAM::getParam(std::string paraName) {
    map_t & mmap = getFloatParameters();
    if(mmap.count(paraName)!=0){
        float * p = std::get<0>(mmap[paraName]);
        return *p;
    }
    return 0;
}
