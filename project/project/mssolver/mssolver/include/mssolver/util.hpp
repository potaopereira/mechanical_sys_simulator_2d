#pragma once

// std::vector
#include <vector>

// std::array
#include  <array>

// for symbolic operations
#include <symbolicc++.h>

namespace sop {

typedef std::array<Symbolic, 2> vec_t;

typedef std::array<Symbolic, 2> lp_t;

typedef std::array<std::array<Symbolic, 2>, 2> ap_t;

/**
 * @brief A pose is composed of a linear position and an angular position
 * 
 */
typedef
struct psym_s {
    lp_t lp;
    ap_t ap;
} p_t;



typedef
Symbolic
(*holonomic_constraint_t)(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters
);

typedef
Symbolic
(*point_constraint_t)(
    std::array<Symbolic, 2> const & point
    ,
    std::vector<Symbolic> const & parameters
);

/**
 * @brief A holonomic constraint is composed of a function (that defines the contraint), plus the rigid bodies involved, plus parameters
 * 
 */
typedef
struct {
    holonomic_constraint_t constraint;
    std::vector<int> rigid_bodies;
    std::vector<Symbolic> parameters;
} tuple_holonomic_constraint_t;

Symbolic get_holonomic_constraint(tuple_holonomic_constraint_t t);

/******************************************************************************/
/* holonomic constraints                                                      */
/******************************************************************************/
/*
pc_*** = point constraints
hc_*** = holonomic contraint
*/

Symbolic
pc_distbetweenpoints(
    lp_t const & lp1
    ,
    lp_t const & lp2
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("dsquared")})
);

Symbolic
hc_distbetweenpoints(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("r0"), Symbolic("r1"), Symbolic("s0"), Symbolic("s1"), Symbolic("dsquared")})
);

Symbolic
pc_dist2point(
    lp_t const & lp1
    ,
    lp_t const & lp2
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("dsquared")})
);

Symbolic
hc_dist2point(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("p0"), Symbolic("p1"), Symbolic("r0"), Symbolic("r1"), Symbolic("dsquared")})
);

Symbolic
pc_slope(
    lp_t const & lp
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("n0"), Symbolic("n1")})
);

Symbolic
hc_slope(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("n0"), Symbolic("n1")})
);

Symbolic
hc_parabola(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("a")})
);

Symbolic
pc_parabola(
    lp_t const & lp
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("a")})
);


Symbolic
hc_ellipse(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("a0"), Symbolic("a1")})
);

Symbolic
pc_ellipse(
    lp_t const & lp
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("a0"), Symbolic("a1")})
);

Symbolic
hc_superellipse(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("a0"), Symbolic("a1"), Symbolic("alpha")})
);

Symbolic
pc_superellipse(
    lp_t const & lp
    ,
    std::vector<Symbolic> const & parameters = std::vector<Symbolic>({Symbolic("x0"), Symbolic("x1"), Symbolic("a0"), Symbolic("a1"), Symbolic("alpha")})
);

/******************************************************************************/

// typedef 
// struct constraint_without_contact_s {
//     Symbolic constraint;
// } constraint_without_contact_t;

/**
 * @brief A contact constraint is composed of a constraint (where the contact point must lie), the boundary of the rigid body, and the normal to the consttraint surface
 * 
 */
typedef
struct constraint_with_contact_s {
    int i;
    Symbolic constraint;
    Symbolic boundary;
    Symbolic normal;
} constraint_with_contact_t;


/**
 * @brief A contact constraint is composed of the constraint function, the boundary function and the rigid body which the constraint applies to
 * 
 */
typedef
struct {
    int rigid_body;
    point_constraint_t constraint;
    std::vector<Symbolic> constraint_parameters;
    point_constraint_t boundary;
    std::vector<Symbolic> boundary_parameters;
    bool sliding;
} tuple_holonomic_constraint_with_contact_t;

constraint_with_contact_t
get_constraint_with_contact(tuple_holonomic_constraint_with_contact_t const & t);

/**
 * @brief A sliding constraint is compose of a non-sliding constraint plus the zero velocity condition at the contact point
 * 
 */
typedef
struct constraint_with_contact_and_no_slide_s {
    constraint_with_contact_t constraint_with_contact;
    Symbolic no_slide_pdot[2+4];
    Symbolic no_slide_v[3];
} constraint_with_contact_and_no_slide_t;

constraint_with_contact_and_no_slide_t
get_constraint_with_contact_and_no_slide(tuple_holonomic_constraint_with_contact_t const & t);

// typedef
// struct constraints_s {
//     std::array<constraint_without_contact_t, M1> wocontact;
//     std::array<constraint_with_contact_t, M2> wicontact;
// } constraints_t;

lp_t getPoint(
    p_t const & p
    ,
    lp_t const & q
);

Symbolic dist(
    lp_t const & p1,
    lp_t const & p2
);


template<int N>
std::array<Symbolic, N>
getdf(
    Symbolic (*function)(
        std::array<Symbolic, N>
    )
    ,
    std::array<Symbolic, N> x
){
    std::array<Symbolic, N> out;
    Symbolic y("#", N);
    std::array<Symbolic, N> y_array;
    for(int i = 0; i < N; ++i){
        y_array[i] = y(i);
    }

    Equations eq;
    for(int i = 0; i < N; ++i){
        eq = (eq, y(i) == x[i]);
    }

    for(int i =0; i < N; ++i){
        Symbolic d = df(function(y_array), y(i));
        out[i] = d[eq];
    }

    return out;
}

template<int N, int M>
std::array<std::array<Symbolic, M>, N>
get_darray(
    std::array<Symbolic, N> const & in
    ,
    std::array<Symbolic, M> const & x
){
    std::array<std::array<Symbolic, M>, N> out;
    for(int i = 0; i < N; ++i){
        for(int j = 0; j < M; ++j){
            out[i][j] = df(in[i], x[j]);
        }
    }
    return out;
}

void
getderivative(
    Symbolic const * in
    ,
    int in_size
    ,
    Symbolic const * x
    ,
    int x_size
    ,
    Symbolic * out
);


template<int N>
std::array<Symbolic, N>
getdf(
    Symbolic (*function)(
        std::array<Symbolic, N> const & x,
        std::vector<Symbolic> const & parameters
    )
    ,
    std::array<Symbolic, N> const & x
    ,
    std::vector<Symbolic> const &  parameters
){
    std::array<Symbolic, N> out;
    Symbolic y("#", N);
    std::array<Symbolic, N> y_array;
    for(int i = 0; i < N; ++i){
        y_array[i] = y(i);
    }

    Equations eq;
    for(int i = 0; i < N; ++i){
        eq = (eq, y(i) == x[i]);
    }

    for(int i =0; i < N; ++i){
        Symbolic d = df(function(y_array, parameters), y(i));
        out[i] = d[eq];
    }

    return out;
}

constraint_with_contact_t getConstraints(
    Symbolic (*constraint)(lp_t)
    ,
    Symbolic (*boundary)(lp_t)
    ,
    int i
);

Symbolic
contactPerp(
    Symbolic (*constraint)(lp_t)
    ,
    Symbolic (*boundary)(lp_t)
    ,
    p_t p
    ,
    lp_t q
);

Symbolic
parabola(
    lp_t p
);

Symbolic
ellipse(
    lp_t p
);

p_t getP(
    int i
);

lp_t getLP(
    int i
);

ap_t getAP(
    int i
);

lp_t getLPContact(
    int i
);

vec_t
rotate(
    ap_t const & ap,
    vec_t const & vin
);

Symbolic
inner(
    vec_t const & v1
    ,
    vec_t const & v2
);

}