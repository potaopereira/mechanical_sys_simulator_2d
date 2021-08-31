#pragma once

#include <string>
#include <map>
#include <tuple>

/**
 * @brief class to get and set geometric parameters
 * 
 */
class IMRB2DPARAM {
public:

    typedef std::tuple<float*, float, float, std::string, std::string> floatparam_t;
    typedef std::map<std::string, floatparam_t> map_t;

    virtual
    map_t& getFloatParameters() = 0;

    void setParam(std::string paraName, float value);

    float getParam(std::string paraName);

    static float getValue(floatparam_t const & p){
        return *(std::get<0>(p));
    };
    static float getMin(floatparam_t const & p){
        return std::get<1>(p);
    };
    static float getMax(floatparam_t const & p){
        return std::get<2>(p);
    };
    static std::string getName(floatparam_t const & p){
        return std::get<3>(p);
    };
    static std::string getDescription(floatparam_t const & p){
        return std::get<4>(p);
    };
};