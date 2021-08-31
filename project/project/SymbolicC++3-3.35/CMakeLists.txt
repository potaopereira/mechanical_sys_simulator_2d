set(PROJECT_NAME SymbolicCpp)

set(
    SRCS
    SymbolicC++3-3.35-vc/src/array.cpp
    SymbolicC++3-3.35-vc/src/cloning.cpp
    SymbolicC++3-3.35-vc/src/constants.cpp
    SymbolicC++3-3.35-vc/src/derive.cpp
    SymbolicC++3-3.35-vc/src/equation.cpp
    SymbolicC++3-3.35-vc/src/functions.cpp
    SymbolicC++3-3.35-vc/src/identity.cpp
    SymbolicC++3-3.35-vc/src/integrate.cpp
    SymbolicC++3-3.35-vc/src/matnorm.cpp
    SymbolicC++3-3.35-vc/src/matrix.cpp
    SymbolicC++3-3.35-vc/src/multinomial.cpp
    SymbolicC++3-3.35-vc/src/number.cpp
    SymbolicC++3-3.35-vc/src/polynomial.cpp
    SymbolicC++3-3.35-vc/src/product.cpp
    SymbolicC++3-3.35-vc/src/quatern.cpp
    SymbolicC++3-3.35-vc/src/rational.cpp
    SymbolicC++3-3.35-vc/src/solve.cpp
    SymbolicC++3-3.35-vc/src/sum.cpp
    SymbolicC++3-3.35-vc/src/symbol.cpp
    SymbolicC++3-3.35-vc/src/symbolic.cpp
    SymbolicC++3-3.35-vc/src/symbolicc++.cpp
    SymbolicC++3-3.35-vc/src/symerror.cpp
    SymbolicC++3-3.35-vc/src/symmatrix.cpp
    SymbolicC++3-3.35-vc/src/vecnorm.cpp
    SymbolicC++3-3.35-vc/src/vector.cpp
    SymbolicC++3-3.35-vc/src/verylong.cpp
)

add_library(
    ${PROJECT_NAME}
    ${SRCS}
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC SymbolicC++3-3.35-vc/include
)