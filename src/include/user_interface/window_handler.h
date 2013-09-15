#ifndef SRC_INCLUDE_USER_INTERFACE_WINDOW_HANDLER_H
#define SRC_INCLUDE_USER_INTERFACE_WINDOW_HANDLER_H

#include <opencv2/core/core.hpp>
#include "user_interface/mode_manager.h"
#include "user_interface/user_infos.h"
#include "user_interface/original_image_infos.hpp"
#include "optimization/mesh.h"

namespace user_interface{

static const int CIRCLE_RADIUS = 5;

class window_handler{
private:
    void mouse_callback_impl(int event, int x, int y, int flags);
    static void mouse_callback(int event, int x, int y, int flags, void* param);
    void refresh(cv::Point const & current_location);
    void render();
public:
    window_handler(const char * filename, const char * window_name);
    const char * window_name() const;
    cv::Mat & image();
    void handle_shortcut(char key_code);
    void switch_show_mesh();
    void reset();
    void bind_mesh();
    void optimize();
    bool is_out_of_image(cv::Point const & p);
    void revert_exit();
    void save(std::string const & filename);

private:
    cv::Mat image_current_;
    cv::Mat image_origin_;
    cv::Mat image_final_;
    cv::Mat cropped_image_;

    cv::Rect cropping_rectangle_;
    user_interface::original_image_infos original_image_infos_;
    optimization::mesh mesh_;
    user_interface::user_infos user_infos_;
    const char * window_name_;
    user_interface::mode_manager mode_manager_;
    Eigen::VectorXd warp_;
    bool show_mesh_;
    bool show_result_;
    bool cropping_;
    bool cropped_;
};


}
#endif // WINDOW_HANDLER_H
