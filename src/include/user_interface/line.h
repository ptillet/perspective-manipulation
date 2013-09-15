#ifndef PERSPECTIVE_MANIPULATION_USER_INTERFACE_LINE_H
#define PERSPECTIVE_MANIPULATION_USER_INTERFACE_LINE_H

#include "opencv2/core/core.hpp"

namespace user_interface{

struct line{
    line() : px_(0), py_(0){ }
    line(cv::Point* px,cv::Point* py) : px_(px), py_(py){ }
    double distance_to(cv::Point const & p){
        std::vector<cv::Point> contour(2); contour[0] = *px_; contour[1] = *py_;
        return cv::pointPolygonTest(cv::Mat(contour),p,true);
    }
    void reset(){ px_ = 0 ; py_ = 0; }
    bool is_init() const{ return px_ && py_; }
    float theta() const {
        assert(is_init() && "Line not initialized!");
        return -atan2((float)py_->y - (float)px_->y, (float)py_->x - (float)px_->x);
    }
    bool operator==(line const & other) const{
        return (px_==other.px_ && py_==other.py_)
                || (py_==other.px_ && px_==other.py_);
    }
    bool operator<(line const & other) const{
        return (std::make_pair(px_,py_) < std::make_pair(other.px_, other.py_))
                || (std::make_pair(py_,px_) < std::make_pair(other.px_,other.py_));

    }

    cv::Point* px_;
    cv::Point* py_;
};



}

#endif
