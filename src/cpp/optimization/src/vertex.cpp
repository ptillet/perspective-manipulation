#include "opencv2/core/core.hpp"

#include "optimization/vertex.h"

namespace optimization{

vertex::vertex() : x_(0), y_(0), i_(0), j_(0){ }
vertex::vertex(int x,int y, unsigned int i, unsigned int j, unsigned int id) : x_(x), y_(y), i_(i), j_(j), id_(id){ }

int vertex::x() const { return x_;}
int vertex::y() const { return y_; }
unsigned int vertex::i() const { return i_;}
unsigned int vertex::j() const { return j_; }

unsigned int vertex::id() const { return id_; }

cv::Point vertex::to_point() const { return cv::Point(x_,y_); }

unsigned int get_id_x(vertex const & v) { return 2*v.id(); }
unsigned int get_id_y(vertex const & v) { return 2*v.id() + 1; }

}
