#pragma once

// ExampleMSSolver
#include "ExampleMS/ExampleMSSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class ExampleMSViewer:
public ExampleMSSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    ExampleMSViewer();
    virtual void reset();
};