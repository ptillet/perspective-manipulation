#include <iostream>
#include <cstdlib>

#include "user_interface/mode_manager.h"
#include "user_interface/window_handler.h"

namespace user_interface{


mode_manager::mode_manager(window_handler & _window_handler) : mode_(CREATE_PLANS), window_handler_(_window_handler){ }

void mode_manager::reset() {
    mode_ = CREATE_PLANS;
}
void mode_manager::handle_shortcut(char key_code){
        switch(key_code){
        case SHORTCUTS_EXIT:
            window_handler_.revert_exit();
            break;

        case SHORTCUTS_RESET:
            window_handler_.reset();
            break;

        case SHORTCUTS_CREATE_PLAN:
             std::cout << "Mode : create plan" << std::endl;
            mode_ = CREATE_PLANS;
            break;


        case SHORTCUTS_VANISHING_POINTS:
            std::cout << "Mode : vanishing points" << std::endl;
            mode_ = VANISHING_POINT;
            break;

        case SWITCH_SHOW_MESH:
            std::cout << "Switching view on mesh..." << std::endl;
            window_handler_.switch_show_mesh();
            break;


        case SHORTCUT_BORDER_CONSTRAINT:
            std::cout << "Mode : border constraint" << std::endl;
            mode_ = BORDER_CONSTRAINT;
            break;

        case SHORTCUT_FIXED_LINE:
            std::cout << "Mode : fixed lines" << std::endl;
            mode_ = FIXED_LINE;
            break;

        case SHORTCUT_FIXED_POINT:
            std::cout << "Mode : fixed point" << std::endl;
            mode_ = FIXED_POINT;
            break;

        case SHORTCUT_OPTIMIZE:
            std::cout << "Optimizing..." << std::endl;
            window_handler_.optimize();
            break;


        case SHORTCUT_CROP:
            std::cout << "Cropping..." << std::endl;
            mode_ = CROP;
            break;

        }


    }

    mode_manager::mode_t mode_manager::mode() const{
        return mode_;
    }

}
