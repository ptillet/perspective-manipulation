#include "user_interface/interpolate.h"

namespace user_interface{

void Interpolator::init_coeff(const cv::Point &p){
    s_ = 1 - ((double)p.x - quad_.uij().x())/(quad_.uij1().x() - quad_.uij().x());
    t_ = 1 - ((double)p.y - quad_.uij().y())/(quad_.ui1j().y() - quad_.uij().y());
//    std::cout << "s,t : " << s_ << " " << t_ << std::endl;
}

//double Interpolator::cij() const { return s_*t_; }
//double Interpolator::cij1() const { return (1-s_)*t_;}
//double Interpolator::ci1j() const { return (1-t_)*s_;}
//double Interpolator::ci1j1() const { return (1-t_)*(1-s_);}

Interpolator::Interpolator(optimization::mesh const & m, cv::Point const & p) : quad_(m.get_enclosing_quad(p)){
    init_coeff(p);
//    cv::Point uij = quad_.uij().to_point();
//    cv::Point uij1 = quad_.uij1().to_point();
//    cv::Point ui1j = quad_.ui1j().to_point();
//    cv::Point ui1j1 = quad_.ui1j1().to_point();
//    std::cout << p << " " << t_*(uij*s_ + uij1*(1-s_)) + (1-t_)*(ui1j*s_ + ui1j1*(1-s_)) << std::endl;
}

double Interpolator::s() const { return s_; }
double Interpolator::t() const { return t_; }

optimization::mesh_quad Interpolator::quad() const { return quad_; }

void get_mesh_intersections(optimization::mesh const & mesh, user_interface::line const & line, std::list<user_interface::Interpolator> & result){
    assert(line.is_init() && "Line not initialized");
    cv::Point pa;
    cv::Point pb;
    if(line.px_->x < line.py_->x){
        pa = *line.px_;
        pb = *line.py_;
    }
    else{
        pa = *line.py_;
        pb = *line.px_;
    }

    if(abs(pa.x - pb.x) < (int)mesh.grid_resolution2()){
        float slope = (double)(pb.x - pa.x)/(pb.y - pa.y);
        for(int y = pa.y ; y < pb.y ; y+=mesh.grid_resolution1()){
            cv::Point newp = cv::Point(pa.x+slope*(y - pa.y),y);
            if(mesh.is_out_of_bond(newp)) return;
            result.push_back(user_interface::Interpolator(mesh,newp));
        }
    }
    else{
        float slope = (double)(pb.y - pa.y)/(pb.x - pa.x);
        for(int x = pa.x ; x < pb.x ; x+=mesh.grid_resolution2()){
            cv::Point newp = cv::Point(x,pa.y+slope*(x - pa.x));
            if(mesh.is_out_of_bond(newp)) return;
            result.push_back(user_interface::Interpolator(mesh,newp));
        }
    }



    result.push_back(user_interface::Interpolator(mesh,pb));
}


}
