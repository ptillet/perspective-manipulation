#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "user_interface/window_handler.h"
#include "user_interface/user_infos.h"
#include "user_interface/original_image_infos.hpp"
#include "user_interface/interpolate.h"
#include "optimization/mesh.h"

#include <Eigen/Sparse>
#include <Eigen/UmfPackSupport>

namespace user_interface{
    static const int N_GRID_POINTS = 40;

    window_handler::window_handler(const char * filename, const char * window_name) :
        image_origin_(cv::imread(filename, CV_LOAD_IMAGE_COLOR))
      ,original_image_infos_(0.7*image_origin_.rows, 0.7*image_origin_.cols, image_origin_.rows, image_origin_.cols)
      ,mesh_(original_image_infos_,N_GRID_POINTS)
      ,user_infos_(original_image_infos_)
      ,window_name_(window_name)
      ,mode_manager_(*this)
      ,show_mesh_(false)
      ,show_result_(false)
      ,cropping_(false)
      ,cropped_(false)
    {
        if(! image_origin_.data )
        {
            std::cerr << "Couldn't find the file to open" << std::endl;
            throw std::string("Could not open or find the image");
        }

        cv::namedWindow(window_name_, CV_WINDOW_AUTOSIZE | CV_WINDOW_FULLSCREEN );
        cv::copyMakeBorder( image_origin_, image_current_,
                            original_image_infos_.top_left()->y,
                            original_image_infos_.top_left()->y,
                            original_image_infos_.top_left()->x,
                            original_image_infos_.top_left()->x,cv::BORDER_CONSTANT , cv::Scalar(255,255,255) );
        image_current_.copyTo(image_origin_);
        cv::imshow( window_name_, image_current_);
        cv::setMouseCallback(window_name_, mouse_callback, this );

    }

    const char * window_handler::window_name() const{
        return window_name_;
    }

    void window_handler::handle_shortcut(char key_code){
        mode_manager_.handle_shortcut(key_code);
    }

    void window_handler::switch_show_mesh(){
        show_mesh_ = !show_mesh_;
    }

    void window_handler::revert_exit(){
        if(show_result_==false)
            exit(EXIT_SUCCESS);
        cropped_=false;
        show_result_=false;

    }

    static double cross_product(cv::Point const & a, cv::Point const & b){
        return (double)a.x*b.y - (double)a.y*b.x;
    }

    void window_handler::render(){
        image_final_ = cv::Mat::ones(image_current_.rows, image_current_.cols,image_current_.type());
        image_final_.setTo(cv::Scalar(255,255,255));
        optimization::mesh final_mesh(warp_,mesh_.vertices_size1(),mesh_.vertices_size2());

        for(unsigned int i = 0 ; i < final_mesh.vertices_size1() -1 ; ++i){
            for(unsigned int j = 0 ; j < final_mesh.vertices_size2() -1 ; ++j){
                optimization::vertex const & vij = final_mesh.get_vertex(i,j);
                optimization::vertex const & vi1j = final_mesh.get_vertex(i+1,j);
                optimization::vertex const & vij1 = final_mesh.get_vertex(i,j+1);
                optimization::vertex const & vi1j1 = final_mesh.get_vertex(i+1,j+1);

                optimization::vertex const & orig_vij = mesh_.get_vertex(i,j);
                optimization::vertex const & orig_vi1j = mesh_.get_vertex(i+1,j);
                optimization::vertex const & orig_vij1 = mesh_.get_vertex(i,j+1);
                optimization::vertex const & orig_vi1j1 = mesh_.get_vertex(i+1,j+1);

//                std::cout << vij.to_point() << " " << vi1j.to_point() << " " << vij1.to_point() << " " << vi1j1.to_point() << std::endl;
//                std::cout << orig_vij.to_point() << " " << orig_vi1j.to_point() << " " << orig_vij1.to_point() << " " << orig_vi1j1.to_point() << std::endl;

                std::vector<cv::Point> triangle_up(4);
                triangle_up[0] = (vij.to_point());
                triangle_up[1] = (vij1.to_point());
                triangle_up[2] = (vi1j1.to_point());
                triangle_up[3] = (vij.to_point());

                cv::Mat triangle_up_contour(triangle_up);

                std::vector<cv::Point> triangle_down(4);
                triangle_down[0] = (vij.to_point());
                triangle_down[2] = (vi1j1.to_point());
                triangle_down[1] = (vi1j.to_point());
                triangle_down[3] = (vij.to_point());

                cv::Mat triangle_down_contour(triangle_down);

                int min_x = std::min(vij.x(),std::min(vi1j.x(),std::min(vij1.x(), vi1j1.x())));
                int max_x = std::max(vij.x(),std::max(vi1j.x(),std::max(vij1.x(), vi1j1.x())));
                int min_y = std::min(vij.y(),std::min(vi1j.y(),std::min(vij1.y(), vi1j1.y())));
                int max_y = std::max(vij.y(),std::max(vi1j.y(),std::max(vij1.y(), vi1j1.y())));

                for(int y = std::max(min_y, 0); y < std::min(max_y,image_final_.rows) ; ++y){
                    for(int x = std::max(min_x,0) ; x < std::min(max_x,image_final_.cols) ; ++x){
                        cv::Point current(x,y);
                        if(cv::pointPolygonTest(triangle_up_contour,current,false)>=0){
                            double areaAPC = abs(cross_product(current - vij.to_point(),vi1j1.to_point() - vij.to_point()));
                            double areaAPB = abs(cross_product(current - vij.to_point(),vij1.to_point() - vij.to_point()));
                            double areaBPC = abs(cross_product(current - vij1.to_point(),vi1j1.to_point() - vij1.to_point()));
                            double areaTot = areaAPC + areaAPB + areaBPC;

                            if(areaTot==0) continue;
                            double a = areaBPC/areaTot;
                            double b = areaAPC/areaTot;
                            double c = areaAPB/areaTot;

                            double orig_x = a*orig_vij.x() + b*orig_vij1.x() + c*orig_vi1j1.x();
                            double orig_y = a*orig_vij.y() + b*orig_vij1.y() + c*orig_vi1j1.y();

                            unsigned int floor_orig_x = floor(orig_x);
                            unsigned int floor_orig_y = floor(orig_y);

                            double s = 1 - (orig_x - floor_orig_x);
                            double t = 1 - (orig_y - floor_orig_y);

//                            image_final_.at<cv::Vec3b>(y,x) = image_origin_.at<cv::Vec3b>(round(orig_y),round(orig_x));

                            image_final_.at<cv::Vec3b>(y,x) = s*t*image_origin_.at<cv::Vec3b>(floor_orig_y,floor_orig_x)
                                    +(1-s)*t*image_origin_.at<cv::Vec3b>(floor_orig_y,floor_orig_x+1)
                                    +s*(1-t)*image_origin_.at<cv::Vec3b>(floor_orig_y+1,floor_orig_x)
                                    +(1-s)*(1-t)*image_origin_.at<cv::Vec3b>(floor_orig_y+1,floor_orig_x+1);
//                            image_final_.at<cv::Vec3b>(y,x) = image_origin_.at<cv::Vec3b>(orig_y,orig_x);
                        }
                        else if(cv::pointPolygonTest(triangle_down_contour,current,false)>=0){
                            double areaAPC = abs(cross_product(current - vij.to_point(),vi1j1.to_point() - vij.to_point()));
                            double areaAPB = abs(cross_product(current - vij.to_point(),vi1j.to_point() - vij.to_point()));
                            double areaBPC = abs(cross_product(current - vi1j.to_point(),vi1j1.to_point() - vi1j.to_point()));
                            double areaTot = areaAPC + areaAPB + areaBPC;

                            if(areaTot==0) continue;

                            double a = areaBPC/areaTot;
                            double b = areaAPC/areaTot;
                            double c = areaAPB/areaTot;

                            double orig_x = a*orig_vij.x() + b*orig_vi1j.x() + c*orig_vi1j1.x();
                            double orig_y = a*orig_vij.y() + b*orig_vi1j.y() + c*orig_vi1j1.y();

//                            image_final_.at<cv::Vec3b>(y,x) = image_origin_.at<cv::Vec3b>(orig_y,orig_x);
                            unsigned int floor_orig_x = floor(orig_x);
                            unsigned int floor_orig_y = floor(orig_y);

                            double s = 1 - (orig_x - floor_orig_x);
                            double t = 1 - (orig_y - floor_orig_y);

                            image_final_.at<cv::Vec3b>(y,x) = image_origin_.at<cv::Vec3b>(round(orig_y),round(orig_x));

                            image_final_.at<cv::Vec3b>(y,x) = s*t*image_origin_.at<cv::Vec3b>(floor_orig_y,floor_orig_x)
                                    +(1-s)*t*image_origin_.at<cv::Vec3b>(floor_orig_y,floor_orig_x+1)
                                    +s*(1-t)*image_origin_.at<cv::Vec3b>(floor_orig_y+1,floor_orig_x)
                                    +(1-s)*(1-t)*image_origin_.at<cv::Vec3b>(floor_orig_y+1,floor_orig_x+1);
                        }
                    }
                }


            }
        }

//        tmp.copyTo(image_final_);
//        cv::boxFilter(tmp,image_final_,-1,cv::Size(3,3));
//        final_mesh.draw(image_final_,cv::Scalar(255,255,0));

    }

    void window_handler::refresh(cv::Point const & current_location){
        if(show_result_)
             user_infos_.create_image(image_final_,image_current_);
        else
            user_infos_.create_image(image_origin_,image_current_);

        if(show_mesh_){
            optimization::mesh_quad const & quad = mesh_.get_enclosing_quad(current_location);
             if(warp_.rows()){
                 optimization::mesh m(warp_,mesh_.vertices_size1(),mesh_.vertices_size2());
                 m.draw(image_current_,cv::Scalar(255,255,0));
                 optimization::mesh_quad warped_quad(m,quad.i(),quad.j());
                 warped_quad.draw(image_current_,cv::Scalar(0,255,0));
             }
            else{
                mesh_.draw(image_current_,cv::Scalar(0,255,255));
                quad.draw(image_current_,cv::Scalar(0,0,255));
            }
        }
        if(cropping_){
            cv::rectangle(image_current_,cropping_rectangle_,cv::Scalar(255,125,255));
        }
        cv::imshow( window_name_, image_current_);
    }

    bool window_handler::is_out_of_image(cv::Point const & p){
        if(p.x < original_image_infos_.top_left()->x
           || p.x > original_image_infos_.top_right()->x
           || p.y < original_image_infos_.top_left()->y
           || p.y > original_image_infos_.bottom_left()->y){
            return true;
        }
        return false;
    }

    void window_handler::mouse_callback_impl(int event, int x, int y, int flags){
        cv::Point current_location(x,y);
        switch(event){
            case CV_EVENT_MOUSEMOVE:
            if(cropping_){
                cropping_rectangle_.width = x-cropping_rectangle_.x;
                cropping_rectangle_.height = y-cropping_rectangle_.y;
            }
            else{
                user_infos_.update_selected_line(current_location);
                if(user_infos_.selected_point()){
                        user_infos_.selected_point()->x = x;
                        user_infos_.selected_point()->y = y;
                }
            }
            break;


            case CV_EVENT_LBUTTONDOWN:
            if(mode_manager_.mode()==mode_manager::CREATE_PLANS){
                user_infos_.add_plan_point(current_location);
            }
            else if(mode_manager_.mode() == mode_manager::VANISHING_POINT){
                user_infos_.construct_vanishing_point();
            }
            else if(mode_manager_.mode() == mode_manager::BORDER_CONSTRAINT){
                user_infos_.add_border_constraint();
            }
            else if(mode_manager_.mode() == mode_manager::FIXED_LINE){
                user_infos_.construct_fixed_line(current_location);
            }
            else if(mode_manager_.mode() == mode_manager::FIXED_POINT){
                user_infos_.add_fixed_point_constraint(current_location);
            }
            else if(mode_manager_.mode() == mode_manager::CROP){
                cropping_=true;
                cropping_rectangle_ = cv::Rect(x,y,0,0);
            }
            break;

            case CV_EVENT_RBUTTONDOWN:
            user_infos_.update_selected_point(current_location);
            break;

            case CV_EVENT_LBUTTONUP:
            user_infos_.reset_selected_point();
            if(cropping_){
                cropping_=false;


                cropping_rectangle_.x+=1;
                cropping_rectangle_.y+=1;
                cropping_rectangle_.width-=2;
                cropping_rectangle_.height-=2;

                if(show_result_)
                    cropped_image_ = image_final_(cropping_rectangle_);
                else
                    cropped_image_ = image_current_(cropping_rectangle_);

                cropping_rectangle_.width=0;
                cropping_rectangle_.height=0;
                std::string filename;
                std::cout << "Enter filename : " << std::endl;
                std::cin >> filename;
                cv::imwrite(filename,cropped_image_);
            }
            break;

            case CV_EVENT_RBUTTONUP:
            user_infos_.reset_selected_point();
            break;
        }
        Interpolator(mesh_,current_location);
        refresh(current_location);
    }

    void window_handler::mouse_callback(int event, int x, int y, int flags, void* params){
        window_handler* p = static_cast<window_handler*>(params);
        p->mouse_callback_impl(event,x,y,flags);
    }

    void window_handler::reset(){
        user_infos_.reset();
        mode_manager_.reset();
    }

    void window_handler::optimize(){
        //Solve
        Eigen::SparseMatrix<double> A;
        Eigen::VectorXd b;
        user_infos_.fill_optimization_matrix(mesh_,A,b);
        Eigen::SparseMatrix<double> A2 = A.adjoint() * A;
        Eigen::VectorXd b2 = A.adjoint() * b;
        Eigen::UmfPackLU<Eigen::SparseMatrix<double> > lu_of_mat(A2);
        warp_ = lu_of_mat.solve(b2);
        render();
        show_result_=true;
//        std::cout << " Energies : " <<  A*warp_ << std::endl;
    }

}
