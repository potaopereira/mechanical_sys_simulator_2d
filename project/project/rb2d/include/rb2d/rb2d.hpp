#pragma once

// std::array
#include  <array>

class LinearPosition {
public:
    typedef std::array<double, 2> p_t;
    LinearPosition(
        double const p1 = 0
        ,
        double const p2 = 0
    );
    double get0() {return p[0];}
    double get1() {return p[1];}
private:
    double p[2];
};

class AngularPosition {
public:
    typedef std::array<double, 4> p_t;
    AngularPosition(
        double theta = 0
    );
    double get00() {return p[0][0];}
    double get01() {return p[0][1];}
    double get10() {return p[1][0];}
    double get11() {return p[1][1];}
private:
    double p[2][2];
};
class Position {
public:
    Position();
    LinearPosition getLP() const {return lp;}
    AngularPosition getAP() const {return ap;}
private:
    LinearPosition lp;
    AngularPosition ap;
};

class RB2D {
public:
    RB2D();
    LinearPosition getLP() const {return p.getLP();}
    AngularPosition getAP() const {return p.getAP();}
private:
    /**
     * @brief linear position
     * 
     */
    Position p;
};