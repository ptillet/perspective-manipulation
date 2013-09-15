#ifndef PERSPECTIVE_MANIPULATION_USER_INTERFACE_ORIGINAL_IMAGE_INFOS_HPP
#define PERSPECTIVE_MANIPULATION_USER_INTERFACE_ORIGINAL_IMAGE_INFOS_HPP

#include "boost/shared_ptr.hpp"
#include "opencv2/opencv.hpp"

namespace user_interface{

struct original_image_infos{
    original_image_infos(unsigned int border_top, unsigned int border_left,unsigned int rows, unsigned int cols) :
        top_left_(new cv::Point(border_left,border_top))
      , top_right_(new cv::Point(border_left+cols,border_top))
      , bottom_right_(new cv::Point(border_left+cols,border_top+rows))
      , bottom_left_(new cv::Point(border_left, border_top + rows))
    {

    }
 
    cv::Point const * top_left() const { return top_left_.get() ; }
    cv::Point const * bottom_left() const { return bottom_left_.get() ; }
    cv::Point const * top_right() const { return top_right_.get() ; }
    cv::Point const * bottom_right() const { return bottom_right_.get() ; }

private:
    boost::shared_ptr<cv::Point> top_left_;
    boost::shared_ptr<cv::Point> top_right_;
    boost::shared_ptr<cv::Point> bottom_right_;
    boost::shared_ptr<cv::Point> bottom_left_;
};

}

#endif // BORDER_INFOS_H
