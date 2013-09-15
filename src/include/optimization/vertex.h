#ifndef PERSPECTIVE_MANIPULATION_OPTIMIZATION_VERTEX_H
#define PERSPECTIVE_MANIPULATION_OPTIMIZATION_VERTEX_H

#include "opencv2/core/core.hpp"

namespace optimization{

class vertex{
public:
    vertex();
    vertex(int x, int y, unsigned int i, unsigned int j, unsigned int id);
    int x() const;
    int y() const ;
    unsigned int i() const;
    unsigned int j() const;
    cv::Point to_point() const;
    unsigned int id() const;
private:
    int x_;
    int y_;
    unsigned int i_;
    unsigned int j_;
    unsigned int id_;
};

unsigned int get_id_x(vertex const & v);
unsigned int get_id_y(vertex const & v);

}
#endif // GRID_POINT_H
