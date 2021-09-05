#include "mssolver/util.hpp"

namespace sop {

lp_t getPoint(
    p_t const & p
    ,
    lp_t const & q
){

    lp_t out = {
        p.lp[0] + p.ap[0][0]*q[0] + p.ap[0][1]*q[1]
        ,
        p.lp[1] + p.ap[1][0]*q[0] + p.ap[1][1]*q[1]
    };
    return out;
}

Symbolic dist(
    lp_t const & p1,
    lp_t const & p2
){
    return (p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]);
}

Symbolic get_holonomic_constraint(tuple_holonomic_constraint_t t){
    return t.constraint(t.rigid_bodies, t.parameters);
}


constraint_with_contact_t
get_constraint_with_contact(tuple_holonomic_constraint_with_contact_t t){

    p_t p = getP(t.rigid_body);
    lp_t q = getLPContact(t.rigid_body);

    lp_t contact_point = getPoint(p, q);
    Symbolic c = t.constraint(contact_point, t.constraint_parameters);
    

    vec_t dconstraint = getdf<2>(t.constraint, contact_point, t.constraint_parameters);
    vec_t dconstraint_perp = {
        +dconstraint[1]
        ,
        -dconstraint[0]
    };

    Symbolic b = t.boundary(q, t.boundary_parameters);
    vec_t dboundary = getdf<2>(t.boundary, q, t.boundary_parameters);

    vec_t dboundary_inertial = rotate(p.ap, dboundary);

    // std::cout << p.lp[0] << std::endl;
    // std::cout << q[0] << std::endl;
    // std::cout << dconstraint[0] << std::endl;
    // std::cout << dconstraint[1] << std::endl;
    // std::cout << dboundary[0] << std::endl;
    // std::cout << dboundary[1] << std::endl;
    // std::cout << dboundary_inertial[0] << std::endl;
    // std::cout << dboundary_inertial[1] << std::endl;

    constraint_with_contact_t constraint_with_contact = {
        .i = t.rigid_body
        ,
        .constraint = c
        ,
        .boundary = b
        ,
        .normal = inner(dconstraint_perp, dboundary_inertial)
    };

    return constraint_with_contact;
}

constraint_with_contact_and_no_slide_t
get_constraint_with_contact_and_no_slide(tuple_holonomic_constraint_with_contact_t t){

    constraint_with_contact_t constraint_with_contact = get_constraint_with_contact(t);

    p_t p = getP(t.rigid_body);
    lp_t q = getLPContact(t.rigid_body);

    lp_t contact_point = getPoint(p, q);
    Symbolic c = t.constraint(contact_point, t.constraint_parameters);
    

    vec_t dconstraint = getdf<2>(t.constraint, contact_point, t.constraint_parameters);
    vec_t dconstraint_perp = {+dconstraint[1],-dconstraint[0]};

    vec_t q_perp = {-q[1], +q[0]};
    Symbolic omega_constraint = inner(dconstraint_perp, rotate(p.ap, q_perp));

    constraint_with_contact_and_no_slide_t constraint_with_contact_and_no_slide = {
        .constraint_with_contact = constraint_with_contact
        ,
        .no_slide_pdot = {
            dconstraint_perp[0],
            dconstraint_perp[1],
            +dconstraint[1]*q[0],
            +dconstraint[1]*q[1],
            -dconstraint[0]*q[0],
            -dconstraint[0]*q[1]
        }
        ,
        .no_slide_v = {
            dconstraint_perp[0],
            dconstraint_perp[1],
            omega_constraint
        }
    };

    return constraint_with_contact_and_no_slide;
}



Symbolic
contactPerp(
    Symbolic (*constraint)(lp_t)
    ,
    Symbolic (*boundary)(lp_t)
    ,
    p_t p
    ,
    lp_t q
){
    lp_t contact_point = getPoint(p, q);
    Symbolic c = constraint(contact_point);
    

    vec_t dconstraint = getdf<2>(constraint, contact_point);
    vec_t dconstraint_perp = {
        +dconstraint[1]
        ,
        -dconstraint[0]
    };

    Symbolic b = boundary(q);
    vec_t dboundary = getdf<2>(boundary, q);
    std::cout << dboundary[0] << std::endl;
    std::cout << dboundary[1] << std::endl;

    vec_t dboundary_inertial = rotate(p.ap, dboundary);

    return inner(dconstraint_perp, dboundary_inertial);
}



constraint_with_contact_t
getConstraints(
    Symbolic (*constraint)(lp_t)
    ,
    Symbolic (*boundary)(lp_t)
    ,
    int i
){

    p_t p = getP(i);
    lp_t q = getLPContact(i);

    lp_t contact_point = getPoint(p, q);
    Symbolic c = constraint(contact_point);
    

    vec_t dconstraint = getdf<2>(constraint, contact_point);
    vec_t dconstraint_perp = {
        +dconstraint[1]
        ,
        -dconstraint[0]
    };

    Symbolic b = boundary(q);
    vec_t dboundary = getdf<2>(boundary, q);
    std::cout << dboundary[0] << std::endl;
    std::cout << dboundary[1] << std::endl;

    vec_t dboundary_inertial = rotate(p.ap, dboundary);

    constraint_with_contact_t constraint_with_contact = {
        .i = i
        ,
        .constraint = c
        ,
        .boundary = b
        ,
        .normal = inner(dconstraint_perp, dboundary_inertial)
    };

    return constraint_with_contact;
}


/**
 * f(x,y)  = y - x^2
 * df(x, y) = [-2x y] = [d1f, d2f]
 * df_perp(x,y) = [y 2x] = [d2f, -d1f]
 * 
 */

Symbolic
parabola(
    lp_t p
){
    return p[1] - p[0]*p[0];
}

Symbolic
ellipse(
    lp_t p
){
    return p[0]*p[0] + p[1]*p[1];
}



static
std::string getName(int i, int j, std::string prefix = std::string("p")){
    return prefix + std::to_string(i) + std::to_string(j);
}

static
Symbolic getSymbolicName(int i, int j, std::string prefix = std::string("p")){
    return Symbolic(getName(i, j, prefix));
}

p_t getP(
    int i
){
    
    // p_t p = {
    //     .lp = {
    //         getSymbolicName(i,0)
    //         ,
    //         getSymbolicName(i,1)
    //     }
    //     ,
    //     .ap = {
    //         {
    //             getSymbolicName(i,2)
    //             ,
    //             getSymbolicName(i,3)
    //         }
    //         ,
    //         {
    //             getSymbolicName(i,4)
    //             ,
    //             getSymbolicName(i,5)
    //         }
    //     }
    // };

    p_t p;
    p.lp[0] = getSymbolicName(i, 0);
    p.lp[1] = getSymbolicName(i, 1);

    p.ap[0][0] = getSymbolicName(i, 2);
    p.ap[0][1] = getSymbolicName(i, 3);
    p.ap[1][0] = getSymbolicName(i, 4);
    p.ap[1][1] = getSymbolicName(i, 5);

    return p;
}

lp_t getLP(
    int i
){
    p_t p = getP(i);
    return p.lp;
}

ap_t getAP(
    int i
){
    p_t p = getP(i);
    return p.ap;
}


lp_t getLPContact(
    int i
){
    lp_t lp;
    lp[0] = getSymbolicName(i, 0, std::string("q"));
    lp[1] = getSymbolicName(i, 1, std::string("q"));
    return lp;
}

vec_t rotate(
    ap_t const & ap,
    vec_t const & vin
){
    vec_t vout = {
        ap[0][0]*vin[0] + ap[0][1]*vin[1]
        ,
        ap[1][0]*vin[0] + ap[1][1]*vin[1]
    };
    return vout;
}

Symbolic inner(
    vec_t const & v1
    ,
    vec_t const & v2
){
    Symbolic out = v1[0]*v2[0] + v1[1]*v2[1];
    return out;
}

Symbolic
pc_distbetweenpoints(
    lp_t const & lp1
    ,
    lp_t const & lp2
    ,
    std::vector<Symbolic> const & parameters // = std::vector<Symbolic>({Symbolic("dsquared")})
){
    if(parameters.size() != 1)
        throw std::invalid_argument("distance position requires 1 parameters: a distance squared.");

    lp_t dd;
    dd[0] = lp1[0] - lp2[0];
    dd[1] = lp1[1] - lp2[1];
    return inner(dd, dd) - parameters[0];
}

Symbolic
hc_distbetweenpoints(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters // = std::vector<Symbolic>({Symbolic("r0"), Symbolic("r1"), Symbolic("s0"), Symbolic("s1"), Symbolic("dsquared")})
){
    if(rigid_bodies.size()!=2)
        throw std::invalid_argument("Distance between points applies to two rigid bodies.");

    if(parameters.size()!=5)
        throw std::invalid_argument("Distance between points requires 5 parameters: two relative points, and a distance squared.");

    p_t p0 = getP(rigid_bodies[0]);
    lp_t point_body0;
    point_body0[0] = parameters[0];
    point_body0[1] = parameters[1];
    lp_t point_inertial0 = getPoint(p0, point_body0);

    p_t p1 = getP(rigid_bodies[1]);
    lp_t point_body1;
    point_body1[0] = parameters[2];
    point_body1[1] = parameters[3];
    lp_t point_inertial1 = getPoint(p1, point_body1);

    return pc_distbetweenpoints(point_inertial0, point_inertial1, std::vector<Symbolic>({parameters[4]}));
}

Symbolic
pc_dist2point(
    lp_t const & lp1
    ,
    lp_t const & lp2
    ,
    std::vector<Symbolic> const & parameters
){
    if(parameters.size() != 1)
        throw std::invalid_argument("distance position requires 1 parameters: a distance squared.");

    lp_t dd;
    dd[0] = lp1[0] - lp2[0];
    dd[1] = lp1[1] - lp2[1];
    return inner(dd, dd) - parameters[0];
}

Symbolic
hc_dist2point(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters // = std::vector<Symbolic>({Symbolic("p0"), Symbolic("p1"), Symbolic("r0"), Symbolic("r1"), Symbolic("dsquared")})
){
    if(rigid_bodies.size()!=1)
        throw std::invalid_argument("Distance to point applies to a single rigid body.");

    if(parameters.size()!=5)
        throw std::invalid_argument("Distance to point requires 5 parameters: point, relative point, and distance squared.");

    p_t p = getP(rigid_bodies[0]);
    lp_t point_body;
    point_body[0] = parameters[2];
    point_body[1] = parameters[3];
    lp_t point_inertial = getPoint(p, point_body);

    lp_t point_fixed;
    point_fixed[0] = parameters[0];
    point_fixed[1] = parameters[1];

    return pc_dist2point(point_inertial, point_fixed, std::vector<Symbolic>({parameters[4]}));
}


Symbolic
pc_slope(
    lp_t const & lp
    ,
    std::vector<Symbolic> const & parameters
){
    if(parameters.size()!=4){
        throw std::invalid_argument("Slope requires 4 parameters: a point and a non-zero direction.");
    }
    else{
        Symbolic x1 = parameters[0];
        Symbolic x0 = parameters[1];
        Symbolic n0  = parameters[2];
        Symbolic n1  = parameters[3];

        vec_t v1 = {lp[0], lp[1]};
        vec_t v2 = {n0, n1};
        return inner(v1, v2);
    }
}

Symbolic
hc_slope(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters
){
    if(rigid_bodies.size()!=1){
        throw std::invalid_argument("Slope constraint applies to a single rigid body.");
    }
    else{
        p_t p = getP(rigid_bodies[0]);
        lp_t lp = p.lp;
        return pc_slope(lp, parameters);
    }
}

Symbolic
pc_parabola(
    lp_t const & lp
    ,
    std::vector<Symbolic> const & parameters
){
    if(parameters.size()!=3){
        throw std::invalid_argument("Parabola requires 3 parameters.");
    }
    else{
        Symbolic x1 = parameters[0];
        Symbolic x0 = parameters[1];
        Symbolic a  = parameters[2];
        return lp[1] + x1 + a*(lp[0] - x0)*(lp[0] - x0);
    }
}

Symbolic
hc_parabola(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters
){
    if(rigid_bodies.size()!=1){
        throw std::invalid_argument("Slope constraint applies to a single rigid body.");
    }
    else{
        p_t p = getP(rigid_bodies[0]);
        lp_t lp = p.lp;
        return pc_parabola(lp, parameters);
    }
}

Symbolic
pc_ellipse(
    lp_t const & lp
    ,
    std::vector<Symbolic> const & parameters
){
    if(parameters.size()!=4){
        throw std::invalid_argument("Ellipse requires 4 parameters: a center point and two axis' lengths.");
    }
    else{
        Symbolic x0 = parameters[0];
        Symbolic x1 = parameters[1];
        Symbolic a0 = parameters[2];
        Symbolic a1 = parameters[3];
        return (lp[0] - x0)*(lp[0] - x0)/(a0*a0) + (lp[1] - x1)*(lp[1] - x1)/(a1*a1) - 1;
    }
}

Symbolic
hc_ellipse(
    std::vector<int> const & rigid_bodies
    ,
    std::vector<Symbolic> const & parameters
){
    if(rigid_bodies.size()!=1){
        throw std::invalid_argument("Ellipse constraint applies to a single rigid body.");
    }
    else{
        p_t p = getP(rigid_bodies[0]);
        lp_t lp = p.lp;
        return pc_ellipse(lp, parameters);;
    }
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
){
    /*
        int out[L][M][N]
        out[i][j][k] is equivalent to *(&out[0][0][0] + (i*M+j)*N+k)

        int in[L][M]
        in[i][j] is equivalent to *(&in[0][0] + (i*M+j))

        int out[L][M][N][O]
        out[i][j][k][l] is equivalent to *(&out[0][0][0][0] + ((i*M+j)*N+k)*O + l)

        int in[L][M][N]
        in[i][j][k] is equivalent to *(&in[0][0][0] + (i*M+j)*N+k)
    */


    for(int i = 0; i < in_size; ++i){
        for(int j = 0; j < x_size; ++j){
            out[i*x_size + j] = df(in[i], x[j]);
            // std::cout << in[i] << std::endl;
            // std::cout << x[j] << std::endl;
            // std::cout << out[i*x_size + j] << std::endl;
        }
    }
}

}