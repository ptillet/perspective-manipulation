#ifndef PERSPECTIVE_MANIPULATION_OPTIMIZATION_MESH_H
#define PERSPECTIVE_MANIPULATION_OPTIMIZATION_MESH_H

#include <list>

#include "opencv2/core/core.hpp"
#include "user_interface/original_image_infos.hpp"
#include "user_interface/line.h"
#include "optimization/vertex.h"

#include "Eigen/Sparse"

namespace optimization{

class mesh;

class mesh_quad{
public:
    mesh_quad(mesh const & m, unsigned int i, unsigned int j);

    vertex const & uij() const;
    vertex const & ui1j() const;
    vertex const & uij1() const;
    vertex const & ui1j1() const;

    unsigned int i() const;
    unsigned int j() const;

    void draw(cv::Mat & img, cv::Scalar color) const;

private:
    mesh const &  m_;
    unsigned int i_;
    unsigned int j_;
};

class mesh{
private:
    friend class mesh_quad;

    typedef std::vector<vertex>  vertices_t;
    void init_grid();
    std::pair<unsigned int, unsigned int> get_closest_upleft_ij(cv::Point const & p) const;
public:
    mesh(user_interface::original_image_infos const & original_image_infos, unsigned int n_points);
    mesh(Eigen::VectorXd const & positions, unsigned int size1, unsigned int size2);
    mesh_quad get_enclosing_quad(cv::Point const & p) const;
    vertex const & get_vertex(unsigned int i,unsigned int j) const;
    unsigned int n_vertices() const;
    void draw(cv::Mat & img, cv::Scalar color);
    void clear();
    const unsigned int grid_resolution1() const;
    const unsigned int grid_resolution2() const;
    unsigned int vertices_size1() const;
    unsigned int vertices_size2() const;
    void fill_smoothness_constraint(double ws, Eigen::SparseMatrix<double>& m,  Eigen::VectorXd& v, unsigned int& k) const;
    void fill_cauchy_constraint(double wc, Eigen::SparseMatrix<double>& m,  Eigen::VectorXd& v, unsigned int &k) const;
    bool is_out_of_bond(cv::Point const & p) const;
private:
    vertices_t vertices_;
    unsigned int vertices_size1_;
    unsigned int vertices_size2_;
    const unsigned int grid_resolution1_;
    const unsigned int grid_resolution2_;
};

}

#endif
