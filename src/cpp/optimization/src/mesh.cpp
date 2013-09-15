#include <iostream>

#include "opencv2/core/core.hpp"

#include "optimization/mesh.h"
#include "optimization/vertex.h"
#include "user_interface/window_handler.h"
#include "user_interface/interpolate.h"

namespace optimization{


mesh_quad::mesh_quad(mesh const & m, unsigned int i, unsigned int j) : m_(m), i_(i), j_(j){ }

vertex const & mesh_quad::uij() const { return m_.get_vertex(i_,j_); }
vertex const & mesh_quad::ui1j() const { return m_.get_vertex( std::min(i_+1,m_.vertices_size1_ - 1),j_); }
vertex const & mesh_quad::uij1() const { return m_.get_vertex(i_,std::min(j_+1,m_.vertices_size2_ - 1) ); }
vertex const & mesh_quad::ui1j1() const { return m_.get_vertex( std::min(i_+1, m_.vertices_size1_ - 1), std::min(j_+1,m_.vertices_size2_ - 1)); }

unsigned int mesh_quad::i() const { return i_; }
unsigned int mesh_quad::j() const { return j_; }

void mesh_quad::draw(cv::Mat &img, cv::Scalar color) const{
    cv::Point contour1[] = { uij().to_point(), uij1().to_point(), ui1j1().to_point(), ui1j().to_point() };
    const cv::Point* contours[] = {contour1};
    int n_points=4;
    cv::polylines(img,contours,&n_points,1,true,color);
}

const unsigned int mesh::grid_resolution1() const { return grid_resolution1_; }
const unsigned int mesh::grid_resolution2() const { return grid_resolution1_; }


mesh::mesh(user_interface::original_image_infos const & original_image_infos, unsigned int n_points): grid_resolution1_(abs(original_image_infos.bottom_left()->y - original_image_infos.top_left()->y)/(double)n_points)
                                                                                                        , grid_resolution2_(abs(original_image_infos.top_right()->x - original_image_infos.top_left()->x)/(double)n_points){
    //Initialize Square Grid

    unsigned int x_limit = original_image_infos.bottom_right()->x;
    unsigned int x_start = original_image_infos.top_left()->x;
    unsigned int y_limit = original_image_infos.bottom_left()->y;
    unsigned int y_start = original_image_infos.top_left()->y;
    vertices_size1_ = 1;
    vertices_size2_ = 1;
    for(unsigned int y = y_start ; y < y_limit; y+= grid_resolution1_) ++vertices_size1_;
    for(unsigned int x = x_start ; x < x_limit ; x+= grid_resolution2_) ++vertices_size2_;

    unsigned int id = 0;
    vertices_.reserve(vertices_size1_*vertices_size2_);
    for(unsigned int i = 0; i<vertices_size1_ ; ++i){
        for(unsigned int j = 0; j<vertices_size2_ ; ++j){
            vertices_.push_back(vertex(std::min(x_start+grid_resolution2_*j,x_limit), std::min(y_start+grid_resolution1_*i,y_limit),i,j,id++));
        }
    }
}

mesh::mesh(Eigen::VectorXd const & positions, unsigned int size1, unsigned int size2) : vertices_size1_(size1), vertices_size2_(size2), grid_resolution1_(0), grid_resolution2_(0){
    for(unsigned int k=0; k<positions.rows()-1; k+=2){
        unsigned int i = k/vertices_size2_;
        unsigned int j = k%vertices_size2_;
        vertices_.push_back(vertex(positions(k),positions(k+1),i,j,k));
    }
}

unsigned int mesh::vertices_size1() const{
    return vertices_size1_;
}
unsigned int mesh::vertices_size2() const{
    return vertices_size2_;
}

void mesh::draw(cv::Mat& img, cv::Scalar color){
    for(unsigned int i=0; i<vertices_size1_;++i){
        for(unsigned int j=0 ; j<vertices_size2_;++j){
            cv::line(img,get_vertex(i,j).to_point(),get_vertex(std::min(vertices_size1_-1,i+1),j).to_point(),color);
            cv::line(img,get_vertex(i,j).to_point(),get_vertex(i,std::min(vertices_size2_-1,j+1)).to_point(),color);
        }
    }
//    for(vertices_t::iterator it = vertices_.begin() ; it != vertices_.end() ; ++it){
//        cv::circle(img,it->to_point(),1,cv::Scalar(0,0,0));
//    }
}

vertex const & mesh::get_vertex(unsigned int i, unsigned int j) const{
    return vertices_.at(i*vertices_size2_ +j);
}

mesh_quad mesh::get_enclosing_quad(cv::Point const & p) const{
    std::pair<unsigned int, unsigned int> ij(get_closest_upleft_ij(p));
    unsigned int i=ij.first;
    unsigned int j=ij.second;

    return mesh_quad(*this,i,j);
}

bool mesh::is_out_of_bond(const cv::Point &p) const{
    return p.x < get_vertex(0,0).x()
            || p.y < get_vertex(0,0).y()
            || p.x > get_vertex(0,vertices_size2_-1).x()
            || p.y > get_vertex(vertices_size1_-1,0).y();
}
std::pair<unsigned int, unsigned int> mesh::get_closest_upleft_ij(cv::Point const & p) const{
    int i=0;
    int j=0;
    for( ; i < (int)vertices_size1_ ; ++i){
        if(get_vertex(i,0).y() > p.y) break;
    }
    for( ; j < (int)vertices_size2_ ; ++j){
        if(get_vertex(0,j).x() > p.x) break;
    }
    return std::make_pair(std::max(i-1,0),std::max(j-1,0));
}

void mesh::fill_smoothness_constraint(double ws, Eigen::SparseMatrix<double>& m,  Eigen::VectorXd& v, unsigned int& k) const{
    for(int i = 0 ; i < (int)vertices_size1_ ; ++i){
        for(int j = 0 ; j < (int)vertices_size2_ ; ++j){
            vertex const & pij = get_vertex(i,j);

            vertex const & pijp1 = get_vertex(i, std::min((unsigned int)j+1,vertices_size2_-1));
            vertex const & pijm1 = get_vertex(i, std::max(0,j-1));

            vertex const & pip1j = get_vertex(std::min((unsigned int)i+1,vertices_size1_-1),j);
            vertex const & pim1j = get_vertex(std::max(0,i-1),j);

            vertex const & pip1jp1 = get_vertex(std::min((unsigned int)i+1,vertices_size1_-1), std::min((unsigned int)j+1,vertices_size2_-1));

            if(j>0 && (j<(int)(vertices_size2_-1))){
                m.coeffRef(k,get_id_y(pijp1))+=ws*1;
                m.coeffRef(k,get_id_y(pij))-=ws*2;
                m.coeffRef(k,get_id_y(pijm1))+=ws*1;
                v(k++)=0;
                m.coeffRef(k,get_id_x(pijp1))+=ws*1;
                m.coeffRef(k,get_id_x(pij))-=ws*2;
                m.coeffRef(k,get_id_x(pijm1))+=ws*1;
                v(k++)=0;
            }
            else{
                v(k++)=0;
                v(k++)=0;
            }



            if(i>0 && (i<(int)(vertices_size1_-1))){
                m.coeffRef(k,get_id_y(pip1j))+=ws*1;
                m.coeffRef(k,get_id_y(pij))-=ws*2;
                m.coeffRef(k,get_id_y(pim1j))+=ws*1;
                v(k++)=0;
                m.coeffRef(k,get_id_x(pip1j))+=ws*1;
                m.coeffRef(k,get_id_x(pij))-=ws*2;
                m.coeffRef(k,get_id_x(pim1j))+=ws*1;
                v(k++)=0;
            }
            else{
                v(k++)=0;
                v(k++)=0;
            }

            if(i<(int)(vertices_size1_-1) && j<(int)(vertices_size2_-1)){
                m.coeffRef(k,get_id_x(pip1jp1))+=ws*1;
                m.coeffRef(k,get_id_x(pip1j))-=ws*1;
                m.coeffRef(k,get_id_x(pijp1))-=ws*1;
                m.coeffRef(k,get_id_x(pij))+=ws*1;
                v(k++)=0;
                m.coeffRef(k,get_id_y(pip1jp1))+=ws*1;
                m.coeffRef(k,get_id_y(pip1j))-=ws*1;
                m.coeffRef(k,get_id_y(pijp1))-=ws*1;
                m.coeffRef(k,get_id_y(pij))+=ws*1;
                v(k++)=0;
            }
            else{
                v(k++)=0;
                v(k++)=0;
            }



        }
    }
}

void mesh::fill_cauchy_constraint(double wc, Eigen::SparseMatrix<double>& m,  Eigen::VectorXd& v, unsigned int& k) const{
    for(unsigned int i = 0 ; i < vertices_size1_-1 ; ++i){
        for(unsigned int j = 0 ; j < vertices_size2_-1 ; ++j){
            vertex const & pij = get_vertex(i,j);
            vertex const & pi1j = get_vertex(std::min(i+1,vertices_size1_-1),j);
            vertex const & pij1 = get_vertex(i,std::min(j+1,vertices_size2_-1));
            m.coeffRef(k,get_id_y(pij1))-=wc*1;
            m.coeffRef(k,get_id_y(pij))+=wc*1;
            m.coeffRef(k,get_id_x(pi1j))-=wc*1;
            m.coeffRef(k,get_id_x(pij))+=wc*1;
            v(k++)=0;

            m.coeffRef(k,get_id_x(pij1))-=wc*1;
            m.coeffRef(k,get_id_x(pij))+=wc*1;
            m.coeffRef(k,get_id_y(pi1j))+=wc*1;
            m.coeffRef(k,get_id_y(pij))-=wc*1;
            v(k++)=0;
        }
    }
}

unsigned int mesh::n_vertices() const{
    return vertices_.size();
}

void mesh::clear(){
    vertices_.clear();
}

}

