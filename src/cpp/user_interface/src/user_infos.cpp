#include <iostream>


#include "user_interface/user_infos.h"
#include "user_interface/window_handler.h"
#include "user_interface/interpolate.h"

#include "optimization/vertex.h"

#include <Eigen/Sparse>
#include "list"

namespace user_interface{


struct my_comparator{
    my_comparator(cv::Point ref_point) : ref_point_(ref_point){ }

    bool operator()(boost::shared_ptr<cv::Point> & p1, boost::shared_ptr<cv::Point>  & p2) const{
        return cv::norm(*p1 - ref_point_) < cv::norm(*p2 - ref_point_);
    }

    bool operator()(cv::Point*  p1, cv::Point* p2) const{
        return cv::norm(*p1 - ref_point_) < cv::norm(*p2 - ref_point_);
    }

    bool operator()(vanishing_point&  p1, vanishing_point& p2) const{
        if(p1.get() && p2.get())
            return cv::norm(*p1.get() - ref_point_) < cv::norm(*p2.get() - ref_point_);
        return false;
    }

private:
    cv::Point ref_point_;
};

cv::Point* get_ptr(vanishing_point const & p){ return p.get(); }
cv::Point* get_ptr(boost::shared_ptr<cv::Point> const & p){ return p.get(); }
cv::Point* get_ptr(cv::Point* p){ return p; }

template<class T>
static cv::Point* get_closest_point(T & points_pointer_list, cv::Point const &p){
    if(points_pointer_list.empty()) return NULL;
    typename T::iterator it(std::min_element(points_pointer_list.begin(), points_pointer_list.end(), my_comparator(p)));
    return get_ptr(*it);
}


plane::plane() : is_constructed_(false){ }

bool plane::add_point_nocheck(cv::Point * p){
    assert(!is_constructed_);
    points_.push_back(p);
    return false;
}

unsigned int plane::n_constraints(optimization::mesh const & mesh) const{
    if(is_constructed_==false) return 0;
    unsigned int res=0;
    for(unsigned int i=0; i<mesh.vertices_size1(); ++i){
        for(unsigned int j=0 ; j<mesh.vertices_size2(); ++j){
            optimization::vertex const & v = mesh.get_vertex(i,j);
            if(is_inside(v.to_point())) res+=2;
        }
    }
    return res;
}

void plane::fill_constraints(double wh, const optimization::mesh &mesh, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd &vec, unsigned int &k, unsigned int plan_offset) const{
    unsigned int offset = plan_offset*N_UNKNOWNS;
    for(unsigned int i=0; i<mesh.vertices_size1(); ++i){
        for(unsigned int j=0 ; j<mesh.vertices_size2(); ++j){
            optimization::vertex const & v = mesh.get_vertex(i,j);
            if(is_inside(v.to_point())){
                mat.coeffRef(k,optimization::get_id_x(v)) += wh*1;
                mat.coeffRef(k,2*mesh.n_vertices()+offset+0) -=  wh*v.x();
                mat.coeffRef(k,2*mesh.n_vertices()+offset+1) -=  wh*v.y();
                mat.coeffRef(k,2*mesh.n_vertices()+offset+2) -=  wh*1;
                vec(k++) = 0;

                mat.coeffRef(k,optimization::get_id_y(v)) += wh*1;
                mat.coeffRef(k,2*mesh.n_vertices()+offset+3) -=  wh*v.x();
                mat.coeffRef(k,2*mesh.n_vertices()+offset+4) -=  wh*v.y();
                mat.coeffRef(k,2*mesh.n_vertices()+offset+5) -=  wh*1;
                vec(k++) = 0;
            }
        }
    }
}

bool plane::is_inside(const cv::Point &p) const{
    if(!is_constructed_) return false;
    std::vector<cv::Point> contour(points_.size());
    for(std::vector<cv::Point*>::const_iterator it = points_.begin() ; it!= points_.end() ; ++it){
        contour.push_back(**it);
    }
    contour.push_back(*points_.front());
    return (cv::pointPolygonTest(cv::Mat(contour),p,false)>=0);
}


std::list<line> plane::get_lines() const{
    std::list<line> res;
    if(is_constructed_){
        cv::Point* prev=NULL;
        for(std::vector<cv::Point*>::const_iterator it=points_.begin(); it!=points_.end(); ++it){
            if(prev) res.push_back(line(prev,*it));
            prev=*it;
        }
        res.push_back(line(points_.front(),points_.back()));
    }
    return res;
}

bool plane::is_constructed() const{
    return is_constructed_;
}

static void add_shared_lines(line ltest,std::list<plane> const & planes, std::map<line, std::pair<unsigned int,unsigned int> > & common_lines){
    for(std::list<plane>::const_iterator it = planes.begin() ; it!= planes.end() ; ++it){
        std::list<line> current(it->get_lines());
        if(std::find(current.begin(),current.end(),ltest)!=current.end()){
                std::cout << std::distance(planes.begin(),it) << " " << planes.size()-1 << std::endl;
                common_lines.insert(std::make_pair(ltest,std::make_pair(std::distance(planes.begin(),it),planes.size()-1)));
        }
    }
}

bool plane::add_point(cv::Point * p,std::list<plane> const & planes, std::map<line, std::pair<unsigned int,unsigned int> > & common_lines){
    if(points_.empty()) return add_point_nocheck(p);
    if(cv::norm(*p - *points_.front()) > user_interface::CIRCLE_RADIUS){
        add_shared_lines(line(points_.back(),p),planes,common_lines);
        return add_point_nocheck(p);
    }
    add_shared_lines(line(points_.front(),points_.back()),planes,common_lines);
    is_constructed_ = true;
    return true;
}

void plane::draw(cv::Mat& img) const {
    cv::Scalar color(is_constructed_?cv::Scalar(0,255,0):cv::Scalar(0,0,255));
    std::vector<cv::Point> points;
    for(std::vector<cv::Point*>::const_iterator itpoint=points_.begin(); itpoint!= points_.end(); ++itpoint){
        cv::circle(img,**itpoint,user_interface::CIRCLE_RADIUS,cv::Scalar(0,255,0),-1);
        points.push_back(**itpoint);
    }
    const cv::Point *pts = (const cv::Point*) points.data();
    int npts = points.size();
    cv::polylines(img,&pts,&npts,1,is_constructed_,color);
}

unsigned int border_constraints::n_constraints(optimization::mesh const & mesh) const{
    return (is_left + is_right)*mesh.vertices_size1()
          +(is_top + is_bottom)*mesh.vertices_size2();
}

void border_constraints::fill_constraints(double wb, const optimization::mesh &m, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd & vec, unsigned int &k) const{
    if(is_left){
       for(unsigned int i=0; i<m.vertices_size1(); ++i){
           optimization::vertex const & pij = m.get_vertex(i,0);
           mat.coeffRef(k,get_id_x(pij))+=wb*1;
           vec(k++) = wb*x_left;
       }
    }
    if(is_right){
        for(unsigned int i=0; i<m.vertices_size1(); ++i){
            optimization::vertex const & pij = m.get_vertex(i,m.vertices_size2()-1);
            mat.coeffRef(k,get_id_x(pij))+=wb*1;
            vec(k++) = wb*x_right;
        }
    }
    if(is_top){
        for(unsigned int j=0; j<m.vertices_size2(); ++j){
            optimization::vertex const & pij = m.get_vertex(0,j);
            mat.coeffRef(k,get_id_y(pij))+=wb*1;
            vec(k++) = wb*y_top;
        }
    }

    if(is_bottom){
        for(unsigned int j=0; j<m.vertices_size2(); ++j){
            optimization::vertex const & pij = m.get_vertex(m.vertices_size1()-1,j);
            mat.coeffRef(k,get_id_y(pij))+=wb*1;
            vec(k++) = wb*y_bottom;
        }
    }
}

vanishing_point::vanishing_point() { }

cv::Point* vanishing_point::get() const{
    return vanishing_point_user.get();
}

static bool get_intersection(cv::Point o1, cv::Point p1, cv::Point o2, cv::Point p2, cv::Point &r)
{
    cv::Point x = o2 - o1;
    cv::Point d1 = p1 - o1;
    cv::Point d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

bool vanishing_point::add_line(line new_line){
    if(lines_.first.is_init()==false){
        lines_.first = new_line;
        return false;
    }
    cv::Point r;
    if(get_intersection(*lines_.first.px_,*lines_.first.py_, *new_line.px_, *new_line.py_,r)){
        lines_.second = new_line;
        vanishing_point_user.reset(new cv::Point(r));
        vanishing_point_planes.reset(new cv::Point(r));
        closests_.first = std::min(lines_.first.px_,lines_.first.py_,my_comparator(*vanishing_point_user));
        closests_.second = std::min(lines_.second.px_,lines_.second.py_,my_comparator(*vanishing_point_user)) ;
        furthests_.first = std::min(lines_.first.px_,lines_.first.py_,my_comparator(*vanishing_point_user));
        furthests_.second = std::min(lines_.second.px_,lines_.second.py_,my_comparator(*vanishing_point_user)) ;
        return true;
    }
    std::cerr << "There is no intersection ..." << std::endl;
    return false;
}

void vanishing_point::refresh() {
//    if(vanishing_point_planes.get()){
//        get_intersection(*lines_.first.px_,*lines_.first.py_, *lines_.second.px_, *lines_.second.py_,*vanishing_point_planes);
//    }
}

void vanishing_point::draw(cv::Mat& img) const{
    if(vanishing_point_user.get()){
         cv::circle(img,*vanishing_point_planes,user_interface::CIRCLE_RADIUS,cv::Scalar(255,0,0),-1);
         cv::circle(img,*vanishing_point_user,user_interface::CIRCLE_RADIUS,cv::Scalar(0,255,0),-1);

         cv::line(img,*vanishing_point_user,*closests_.first,cv::Scalar(0,0,255),1,4);
         cv::line(img,*vanishing_point_user,*closests_.second,cv::Scalar(0,0,255),1,4);
    }
}

void vanishing_point::fill_line_constraint(double wv, line l, line l2, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const{
    cv::Point p(*vanishing_point_user);
    float theta = l.theta();
    std::list<Interpolator> interpolators;
//    interpolators.push_back(Interpolator(m,*l2.px_));
//    interpolators.push_back(Interpolator(m,*l2.py_));

    get_mesh_intersections(m,l2,interpolators);

    for(std::list<Interpolator>::iterator it = interpolators.begin() ; it != interpolators.end() ; ++it){
        optimization::mesh_quad quad = it->quad();
        double t = it->t();
        double s = it->s();
        //Distance is sin(theta)*(p.x - p0.x) + cos(theta)*(p.y - p0.y) and it has to be minimized
        mat.coeffRef(i,optimization::get_id_x(quad.uij())) += wv*sin(theta)*s*t;
        mat.coeffRef(i,optimization::get_id_x(quad.uij1())) += wv*sin(theta)*(1-s)*t;
        mat.coeffRef(i,optimization::get_id_x(quad.ui1j())) += wv*sin(theta)*s*(1-t);
        mat.coeffRef(i,optimization::get_id_x(quad.ui1j1())) += wv*sin(theta)*(1-s)*(1-t);

        mat.coeffRef(i,optimization::get_id_y(quad.uij())) += wv*cos(theta)*s*t;
        mat.coeffRef(i,optimization::get_id_y(quad.uij1())) += wv*cos(theta)*(1-s)*t;
        mat.coeffRef(i,optimization::get_id_y(quad.ui1j())) += wv*cos(theta)*s*(1-t);
        mat.coeffRef(i,optimization::get_id_y(quad.ui1j1())) += wv*cos(theta)*(1-s)*(1-t);


        vec(i++) = wv*(sin(theta)*p.x + cos(theta)*p.y);
    }




}

unsigned int vanishing_point::n_line_points(line l, optimization::mesh const & m) const{
    cv::Point p(*vanishing_point_user);
//    cv::Point p0(*std::max(l.px_,l.py_,my_comparator(p)));
//    float theta = - atan2(p.y - p0.y, p.x - p0.x);
    std::list<Interpolator> interpolators;
    get_mesh_intersections(m,l,interpolators);
//    std::list<Interpolator>::iterator it0 = interpolators.begin();
    return interpolators.size();
}

void vanishing_point::fill_constraints(double wv, const optimization::mesh &m, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd &vec, unsigned int &i) const{
    assert(vanishing_point_user.get() && "Vanishing point not initialized !");
    fill_line_constraint(wv, line(vanishing_point_user.get(),closests_.first),lines_.first,m,mat,vec,i);
    fill_line_constraint(wv, line(vanishing_point_user.get(),closests_.second),lines_.second,m,mat,vec,i);
}

unsigned int vanishing_point::n_constraints(const optimization::mesh &mesh) const{
    if(vanishing_point_user.get()) return n_line_points(lines_.first,mesh) + n_line_points(lines_.second,mesh);
    return 0;

}

user_infos::user_infos(original_image_infos const & infos): planes_(1)
  , vanishing_points_(1)
  , fixed_lines_(1)
  , original_image_infos_(infos)
  , selected_point_(NULL)
{ }

void user_infos::refresh(){
    for(std::list<vanishing_point>::iterator it = vanishing_points_.begin() ; it != vanishing_points_.end() ; ++it){
        it->refresh();
    }
}

void user_infos::add_plan_point(cv::Point const & p){
    cv::Point* closest_point = get_closest_point(user_points_,p);
    plane& back_plane=planes_.back();
    if(closest_point){
        if(cv::norm(p - *closest_point) <= user_interface::CIRCLE_RADIUS){
            if(back_plane.add_point(closest_point,planes_,shared_planes_line_))
                planes_.push_back(plane());
            selected_point_=closest_point;
            return;
        }
    }
    user_points_.push_back(boost::shared_ptr<cv::Point>(new cv::Point(p)));
    selected_point_=user_points_.back().get();
    if(back_plane.add_point(selected_point_,planes_,shared_planes_line_))
        planes_.push_back(plane());
//    std::cout << shared_points_line_.size() << std::endl;

}

void user_infos::construct_vanishing_point(){
    if(selected_line_.px_ && selected_line_.py_){
        vanishing_point& back_vanishing_point = vanishing_points_.back();
        if(back_vanishing_point.add_line(selected_line_)){
            vanishing_points_.push_back(vanishing_point());
        }
    }
}

void user_infos::construct_fixed_line(cv::Point const & p){
    cv::Point* closest_point = get_closest_point(user_points_,p);
    fixed_line& back_line=fixed_lines_.back();
    if(closest_point){
        if(cv::norm(p - *closest_point) <= user_interface::CIRCLE_RADIUS){
            if(back_line.add_point(closest_point))
                fixed_lines_.push_back(fixed_line());
            selected_point_=closest_point;
            return;
        }
    }
    user_points_.push_back(boost::shared_ptr<cv::Point>(new cv::Point(p)));
    selected_point_=user_points_.back().get();
    if(back_line.add_point(selected_point_))
        fixed_lines_.push_back(fixed_line());
}


cv::Point * user_infos::selected_point(){
    return selected_point_;
}

void user_infos::reset_selected_point(){
    selected_point_=NULL;
}

void user_infos::update_selected_point(cv::Point const & p){
    std::list<cv::Point*> closests;
    if(cv::Point* closest_plane_point = get_closest_point(user_points_,p)) closests.push_back(closest_plane_point);
    if(cv::Point* closest_vanishing_point = get_closest_point(vanishing_points_, p)) closests.push_back(closest_vanishing_point);
    cv::Point* closest_of_all = get_closest_point(closests,p);
    if(closest_of_all){
        if(cv::norm(p - *closest_of_all) <= user_interface::CIRCLE_RADIUS)
            selected_point_ = closest_of_all;
    }
}

line user_infos::selected_line() const{
    return selected_line_;
}

void user_infos::update_selected_line(const cv::Point &mouse_pos){
    line closest(get_closest_line(mouse_pos));
    if(closest.px_ && closest.py_){
        if(abs(closest.distance_to(mouse_pos))<10)
            selected_line_ = closest;
        else
            selected_line_.reset();
    }
}

std::list<fixed_line> const & user_infos::fixed_lines() const{
    return fixed_lines_;
}
void user_infos::add_fixed_point_constraint(const cv::Point &p) {
    cv::Point* closest_point = get_closest_point(user_points_,p);
    if(closest_point){
        if(cv::norm(p - *closest_point) <= user_interface::CIRCLE_RADIUS){
            fixed_points_.push_back(closest_point);
        }
    }
    user_points_.push_back(boost::shared_ptr<cv::Point>(new cv::Point(p)));
    selected_point_=user_points_.back().get();
    fixed_points_.push_back(selected_point_);
}

void user_infos::create_image(cv::Mat const & in, cv::Mat out) const{
     in.copyTo(out);
     for(std::list<plane>::const_iterator it=planes_.begin(); it!= planes_.end(); ++it){
         it->draw(out);
     }
     for(std::list<vanishing_point>::const_iterator it=vanishing_points_.begin(); it!=vanishing_points_.end(); ++it){
         it->draw(out);
     }
     for(std::list<fixed_line>::const_iterator it=fixed_lines_.begin(); it!=fixed_lines_.end(); ++it){
         it->draw(out);
     }
     for(std::list<cv::Point*>::const_iterator it=fixed_points_.begin(); it!=fixed_points_.end(); ++it){
         cv::circle(out,**it,user_interface::CIRCLE_RADIUS,cv::Scalar(255,255,0),-1);
     }

     border_constraints_.draw(out);
     if(selected_line_.px_ && selected_line_.py_){
         cv::line(out,*selected_line_.px_, *selected_line_.py_,cv::Scalar(255,0,0),2);
     }
 }



 struct my_line_comparator{
     my_line_comparator(cv::Point ref_point) : ref_point_(ref_point){ }
     bool operator()(line & l1, line  & l2) const{
         return abs(l1.distance_to(ref_point_)) < abs(l2.distance_to(ref_point_));
     }

 private:
     cv::Point ref_point_;
 };

 line user_infos::get_closest_line(const cv::Point &p) const {
     std::list<line> lines;
     for(std::list<plane>::const_iterator it = planes_.begin();it!=planes_.end()
         ;++it){
         std::list<line> new_lines(it->get_lines());
         lines.splice(lines.end(),new_lines);
     }
     lines.push_back(line((cv::Point*)original_image_infos_.top_left(),(cv::Point*)original_image_infos_.top_right()));
     lines.push_back(line((cv::Point*)original_image_infos_.top_right(),(cv::Point*)original_image_infos_.bottom_right()));
     lines.push_back(line((cv::Point*)original_image_infos_.bottom_right(),(cv::Point*)original_image_infos_.bottom_left()));
     lines.push_back(line((cv::Point*)original_image_infos_.bottom_left(),(cv::Point*)original_image_infos_.top_left()));
     std::list<line>::iterator it(std::min_element(lines.begin(),lines.end(),my_line_comparator(p)));
     return *it;
 }

 void user_infos::add_border_constraint(){
     if(selected_line_.is_init()){
         int ytop = original_image_infos_.top_left()->y;
         int ybottom = original_image_infos_.bottom_left()->y;
         int xleft = original_image_infos_.top_left()->x;
         int xright = original_image_infos_.top_right()->x;
         if(selected_line_.px_->y == ytop && selected_line_.py_->y == ytop){
                border_constraints_.is_top=true;
                border_constraints_.y_top=ytop;
         }
         else if(selected_line_.px_->y == ybottom && selected_line_.py_->y == ybottom){
             border_constraints_.is_bottom=true;
             border_constraints_.y_bottom=ybottom;
         }
         else if(selected_line_.px_->x == xleft && selected_line_.py_->x == xleft){
             border_constraints_.is_left=true;
             border_constraints_.x_left = xleft;
         }
         else if(selected_line_.px_->x == xright && selected_line_.py_->x == xright){
             border_constraints_.is_right=true;
             border_constraints_.x_right = xright;
         }
     }

 }


void fixed_line::fill_constraints(double wl, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const{

        float theta =  line_.theta();
        std::list<Interpolator> interpolators;
        get_mesh_intersections(m,line_,interpolators);
        std::list<Interpolator>::iterator it0 = interpolators.begin();
        optimization::mesh_quad quad0 = it0->quad();
        double t0 = it0->t();
        double s0 = it0->s();
        for(std::list<Interpolator>::iterator itp = it0 ; itp!= interpolators.end() ; ++itp){
            optimization::mesh_quad quad = itp->quad();
            double t = itp->t();
            double s = itp->s();
            //Distance is sin(theta)*(p.x - p0.x) + cos(theta)*(p.y - p0.y) and it has to be minimized
            mat.coeffRef(i,optimization::get_id_x(quad.uij())) += sin(theta)*wl*s*t;
            mat.coeffRef(i,optimization::get_id_x(quad.uij1())) += sin(theta)*wl*(1-s)*t;
            mat.coeffRef(i,optimization::get_id_x(quad.ui1j())) += sin(theta)*wl*s*(1-t);
            mat.coeffRef(i,optimization::get_id_x(quad.ui1j1())) += sin(theta)*wl*(1-s)*(1-t);

            mat.coeffRef(i,optimization::get_id_x(quad0.uij())) -= sin(theta)*wl*s0*t0;
            mat.coeffRef(i,optimization::get_id_x(quad0.uij1())) -= sin(theta)*wl*(1-s0)*t0;
            mat.coeffRef(i,optimization::get_id_x(quad0.ui1j())) -= sin(theta)*wl*s0*(1-t0);
            mat.coeffRef(i,optimization::get_id_x(quad0.ui1j1())) -= sin(theta)*wl*(1-s0)*(1-t0);

            mat.coeffRef(i,optimization::get_id_y(quad.uij())) += cos(theta)*wl*s*t;
            mat.coeffRef(i,optimization::get_id_y(quad.uij1())) += cos(theta)*wl*(1-s)*t;
            mat.coeffRef(i,optimization::get_id_y(quad.ui1j())) += cos(theta)*wl*s*(1-t);
            mat.coeffRef(i,optimization::get_id_y(quad.ui1j1())) += cos(theta)*wl*(1-s)*(1-t);

            mat.coeffRef(i,optimization::get_id_y(quad0.uij())) -= cos(theta)*wl*s0*t0;
            mat.coeffRef(i,optimization::get_id_y(quad0.uij1())) -= cos(theta)*wl*(1-s0)*t0;
            mat.coeffRef(i,optimization::get_id_y(quad0.ui1j())) -= cos(theta)*wl*s0*(1-t0);
            mat.coeffRef(i,optimization::get_id_y(quad0.ui1j1())) -= cos(theta)*wl*(1-s0)*(1-t0);


            vec(i++) = 0;
        }
}

void user_infos::fill_homography_constraints(double wh, const optimization::mesh &m, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd &vec, unsigned int &i) const{
    unsigned int n=0;
    for(std::list<plane>::const_iterator itp = planes_.begin() ; itp != planes_.end() ; ++itp){
        if(itp->is_constructed()){
            itp->fill_constraints(wh,m,mat,vec,i,n++);
        }
    }
}

static const unsigned int N_COMPATIBILITY=10;

void user_infos::fill_homography_compatibility(double whc, const optimization::mesh &m, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd &vec, unsigned int &i) const{
    for(std::map<line, std::pair<unsigned int,unsigned int> >::const_iterator itl = shared_planes_line_.begin() ; itl != shared_planes_line_.end() ; ++itl){
        line current(itl->first);
        assert(current.is_init());
        unsigned int id1 = itl->second.first;
        unsigned int offset1 = id1*plane::N_UNKNOWNS;
        unsigned int id2 = itl->second.second;
        unsigned int offset2 = id2*plane::N_UNKNOWNS;
        cv::Point x1 = *current.px_;
        cv::Point x2 = *current.py_;
        for(unsigned int k=0 ; i<N_COMPATIBILITY ;k++){
            double alpha = (double)k/N_COMPATIBILITY;
            std::cout << alpha<< std::endl;
            mat.coeffRef(i,2*m.n_vertices()+offset1+0) += whc*(alpha*x1.x + (1-alpha)*x2.x);
            mat.coeffRef(i,2*m.n_vertices()+offset1+1) += whc*(alpha*x1.y + (1-alpha)*x2.y);
            mat.coeffRef(i,2*m.n_vertices()+offset1+2) += whc*(1);
            mat.coeffRef(i,2*m.n_vertices()+offset1+3) += whc*(alpha*x1.x + (1-alpha)*x2.x);
            mat.coeffRef(i,2*m.n_vertices()+offset1+4) += whc*(alpha*x1.y + (1-alpha)*x2.y);
            mat.coeffRef(i,2*m.n_vertices()+offset1+5) += whc*(1);

            mat.coeffRef(i,2*m.n_vertices()+offset2+0) -= whc*(alpha*x1.x + (1-alpha)*x2.x);
            mat.coeffRef(i,2*m.n_vertices()+offset2+1) -= whc*(alpha*x1.y + (1-alpha)*x2.y);
            mat.coeffRef(i,2*m.n_vertices()+offset2+2) -= whc*(1);
            mat.coeffRef(i,2*m.n_vertices()+offset2+3) -= whc*(alpha*x1.x + (1-alpha)*x2.x);
            mat.coeffRef(i,2*m.n_vertices()+offset2+4) -= whc*(alpha*x1.y + (1-alpha)*x2.y);
            mat.coeffRef(i,2*m.n_vertices()+offset2+5) -= whc*(1);

            vec(i++)=0;
        }
    }
}




void user_infos::fill_fixed_line_constraints(double wl, optimization::mesh const & mesh, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const{
    for(std::list<fixed_line>::const_iterator itl = fixed_lines_.begin(); itl != fixed_lines_.end(); ++itl){
        if(itl->get().is_init()) itl->fill_constraints(wl,mesh,mat,vec,i);
    }
}

void user_infos::fill_fixed_points_constraints(double wp, optimization::mesh const & mesh, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & k) const{

    for(std::list<cv::Point*>::const_iterator itp = fixed_points_.begin() ; itp != fixed_points_.end() ; ++itp){
        Interpolator interp(mesh,**itp);
        optimization::mesh_quad quad = interp.quad();
        double t = interp.t();
        double s = interp.s();
        mat.coeffRef(k,optimization::get_id_x(quad.uij())) += wp*s*t;
        mat.coeffRef(k,optimization::get_id_x(quad.uij1())) += wp*(1-s)*t;
        mat.coeffRef(k,optimization::get_id_x(quad.ui1j())) += wp*s*(1-t);
        mat.coeffRef(k,optimization::get_id_x(quad.ui1j1())) += wp*(1-s)*(1-t);
        vec(k++) = wp*(*itp)->x;

        mat.coeffRef(k,optimization::get_id_y(quad.uij())) += wp*s*t;
        mat.coeffRef(k,optimization::get_id_y(quad.uij1())) += wp*(1-s)*t;
        mat.coeffRef(k,optimization::get_id_y(quad.ui1j())) += wp*s*(1-t);
        mat.coeffRef(k,optimization::get_id_y(quad.ui1j1())) += wp*(1-s)*(1-t);
        vec(k++) = wp*(*itp)->y;

    }
}


void user_infos::fill_vanishing_point_constraints(double wp, optimization::mesh const & m, Eigen::SparseMatrix<double> & mat, Eigen::VectorXd & vec, unsigned int & i) const{
    for(std::list<vanishing_point>::const_iterator itv = vanishing_points_.begin(); itv != vanishing_points_.end(); ++itv){
        if(itv->get()) itv->fill_constraints(wp,m,mat,vec,i);
    }
}



 void user_infos::fill_optimization_matrix(optimization::mesh const & mesh, Eigen::SparseMatrix<double> &mat, Eigen::VectorXd & vec) const{
     unsigned int n_line_points=0;
     for(std::list<fixed_line>::const_iterator it = fixed_lines_.begin(); it != fixed_lines_.end(); ++it){

         if(it->get().is_init()) n_line_points+= 1 + (abs(it->get().py_->x - it->get().px_->x))/mesh.grid_resolution2() + 1 ;
     }

     unsigned int size2 = 2*mesh.n_vertices();
     size2 += (planes_.size() - 1)*plane::N_UNKNOWNS;

     unsigned int size1 = 0;
     size1 += 2*mesh.n_vertices(); //Cauchy Energy for Shape Preservation
     size1 += 6*mesh.n_vertices(); //Smoothness Energy Constraints
     size1 += border_constraints_.n_constraints(mesh);
     size1 += fixed_points_.size()*2; //One constraint in x and one in y per fix point;

     for(std::list<plane>::const_iterator it = planes_.begin() ; it != planes_.end() ; ++it){
         size1 += it->n_constraints(mesh);
     }
     size1 += shared_planes_line_.size()*N_COMPATIBILITY;
     for(std::list<vanishing_point>::const_iterator it = vanishing_points_.begin() ; it != vanishing_points_.end() ; ++it){
         size1 += it->n_constraints(mesh);
     }

     size1 += n_line_points;
     size1 += (vanishing_points_.size() - 1)*2; //2 Lines per vanishing point


     mat.resize(size1,size2);
     vec.resize(size1);

     double ws=30;
     double wc=3;
     double wl=100;
     double wv=120;
     double wp=100;
     double wb=100;
     double wh=20;
     double whc=200;

     unsigned int k = 0;
     mesh.fill_cauchy_constraint(wc,mat,vec,k);
     mesh.fill_smoothness_constraint(ws,mat,vec,k);
     border_constraints_.fill_constraints(wb,mesh,mat,vec,k);
     fill_fixed_points_constraints(wp,mesh,mat,vec,k);

     fill_homography_constraints(wh,mesh,mat,vec,k);
     fill_homography_compatibility(whc,mesh,mat,vec,k);
     fill_fixed_line_constraints(wl,mesh,mat,vec,k);
     fill_vanishing_point_constraints(wv,mesh,mat,vec,k);

//     std::cout << mat << std::endl;
//     exit(1);
 }

 void user_infos::reset(){
     planes_.clear();
     planes_.push_back(plane());
     fixed_lines_.clear();
     fixed_lines_.push_back(fixed_line());
     user_points_.clear();
//     active_borders_.clear();
     vanishing_points_.clear();
     vanishing_points_.push_back(vanishing_point());
     fixed_points_.clear();
     selected_point_=NULL;
 }




fixed_line::fixed_line(){ }

bool fixed_line::add_point(cv::Point * new_point){
    if(line_.px_==0){
        line_.px_ = new_point;
        return false;
    }
    else{
        assert(line_.py_==0);
        line_.py_ = new_point;
        return true;
    }
}

line fixed_line::get() const{
    return line_;
}

void fixed_line::draw(cv::Mat& img) const{
    if(line_.px_) cv::circle(img,*line_.px_,user_interface::CIRCLE_RADIUS,cv::Scalar(0,0,255),-1);
    if(line_.py_) cv::circle(img,*line_.py_,user_interface::CIRCLE_RADIUS,cv::Scalar(0,0,255),-1);
    if(line_.is_init()) cv::line(img,*line_.px_,*line_.py_,cv::Scalar(255,255,0),1);
}



}
