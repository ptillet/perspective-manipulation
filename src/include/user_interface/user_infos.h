#ifndef SRC_INCLUDE_USER_INTERFACE_USER_INFOS_H
#define SRC_INCLUDE_USER_INTERFACE_USER_INFOS_H

#include <vector>
#include <list>

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>
#include "opencv/cv.h"

#include "optimization/mesh.h"
#include "line.h"
#include "user_interface/original_image_infos.hpp"
#include "user_interface/interpolate.h"

#include "Eigen/Sparse"
#include "Eigen/Core"

namespace user_interface{

class plane{
private:
    bool add_point_nocheck(cv::Point* p);
public:
    static const int N_UNKNOWNS=6;
    plane();
    unsigned int n_constraints(optimization::mesh const & mesh) const;
    bool add_point(cv::Point* p,std::list<plane> const & planes, std::map<line, std::pair<unsigned int,unsigned int> > & common_lines);
    bool is_constructed() const;
    void draw(cv::Mat& img) const;
    std::list<line> get_lines() const;
    bool is_inside(cv::Point const & p) const;
    void fill_constraints(double wh, const optimization::mesh &m, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd & vec, unsigned int &i, unsigned int plan_offset) const;
private:
    std::vector<cv::Point*> points_;
    bool is_constructed_;
};

class border_constraints{
public:
    border_constraints() : is_left(false), x_left(0), is_right(false), x_right(0)
                            ,is_top(false), y_top(0), is_bottom(false), y_bottom(0){

    }

    void draw(cv::Mat & img) const{
        if(is_left) cv::line(img,cv::Point(x_left,0), cv::Point(x_left,img.rows) ,cv::Scalar(0,255,255));
        if(is_top) cv::line(img,cv::Point(0,y_top), cv::Point(img.cols,y_top) ,cv::Scalar(0,255,255));
        if(is_right) cv::line(img,cv::Point(x_right,0), cv::Point(x_right,img.rows) ,cv::Scalar(0,255,255));
        if(is_top) cv::line(img,cv::Point(0,y_bottom), cv::Point(img.cols,y_bottom) ,cv::Scalar(0,255,255));
    }

    void fill_constraints(double wb, const optimization::mesh &m, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd & vec, unsigned int &i) const;
    unsigned int n_constraints(optimization::mesh const & mesh) const;

    bool is_left;
    unsigned int x_left;
    bool is_right;
    unsigned int x_right;
    bool is_top;
    unsigned int y_top;
    bool is_bottom;
    unsigned int y_bottom;
};

class vanishing_point{
private:
    void fill_line_constraint(double wv, line l, line l2, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const;
    unsigned int n_line_points(line l, optimization::mesh const & m) const;
public:
    vanishing_point();
    void draw(cv::Mat& img) const;
    bool add_line(line new_line);
    void refresh();
    cv::Point* get() const;
    void fill_constraints(double wv, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const;
    unsigned int n_constraints(optimization::mesh const & mesh) const;
private:
    boost::shared_ptr<cv::Point> vanishing_point_user;
    boost::shared_ptr<cv::Point> vanishing_point_planes;
    std::pair<line,line> lines_;
    std::pair<cv::Point*, cv::Point*> closests_;
    std::pair<cv::Point*, cv::Point*> furthests_;
};

class fixed_line{
public:
    fixed_line();
    bool add_point(cv::Point * new_point);
    line get() const;
    void fill_constraints(double wl, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const;
    void draw(cv::Mat& img) const;
private:
    line line_;
};

class user_infos{
private:
    line get_closest_line(cv::Point const & p) const;
    void fill_fixed_points_constraints(double wp, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const;
    void fill_fixed_line_constraints(double wl, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const;
    void fill_vanishing_point_constraints(double wv, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const;
    void fill_homography_constraints(double wh, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const;
    void fill_homography_compatibility(double whc, const optimization::mesh &m, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd &vec, unsigned int &i) const;

public:
    user_infos(original_image_infos const & infos);
    void add_plan_point(cv::Point const & p);
    void construct_vanishing_point();
    void construct_fixed_line(const cv::Point &p);
    void add_border_constraint();
    void add_fixed_point_constraint(const cv::Point & p);
    cv::Point * selected_point();
    line selected_line() const;
    void update_selected_line(const cv::Point &mouse_pos);
    void reset_selected_point();
    void update_selected_point(cv::Point const & p);
    void reset();
    void refresh();
    void create_image(cv::Mat const & in, cv::Mat out) const;

    std::list<fixed_line> const & fixed_lines() const;

    void fill_optimization_matrix(optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec) const;

private:
    std::list<boost::shared_ptr<cv::Point> > user_points_;

    std::list<plane> planes_;
    std::list<vanishing_point> vanishing_points_;
    border_constraints border_constraints_;
//    std::list<line> active_borders_;
    std::map<line, std::pair<unsigned int,unsigned int> > shared_planes_line_;
    std::list<fixed_line> fixed_lines_;
    std::list<cv::Point*> fixed_points_;
    original_image_infos const & original_image_infos_;
    line selected_line_;
    cv::Point* selected_point_;

};

}


#endif
