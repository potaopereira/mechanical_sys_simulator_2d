#pragma once

// ExampleMSSolver
#include "examplems/examplemssolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class ExampleMSWithViewer:
public ExampleMSSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    ExampleMSWithViewer();
    virtual void reset();
};