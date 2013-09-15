#ifndef PERSPECTIVE_MANIPULATION_USER_INTERFACE_INTERPOLATE_H
#define PERSPECTIVE_MANIPULATION_USER_INTERFACE_INTERPOLATE_H

#include "optimization/vertex.h"
#include "optimization/mesh.h"
#include "opencv2/core/core.hpp"
#include <utility>

namespace user_interface{

class Interpolator{
private:
    void init_coeff(cv::Point const & p);

public:
    Interpolator(const optimization::mesh &m, cv::Point const & p);
//    double cij() const;
//    double cij1() const;
//    double ci1j() const;
//    double ci1j1() const;

    double s() const;
    double t() const;
    optimization::mesh_quad quad() const;
private:
    optimization::mesh_quad quad_;
    double s_;
    double t_;
};

void get_mesh_intersections(optimization::mesh const & mesh, user_interface::line const & line, std::list<user_interface::Interpolator> & res);



}

#endif // INTERPOLATE_H
