#define DEBUG

#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "user_interface/mode_manager.h"
#include "user_interface/window_handler.h"

int main(int argc, char* argv[]){
	if( argc != 2)
    {
     std::cerr <<" Usage: display_image ImageToLoadAndDisplay" << std::endl;
     return -1;
    }
    user_interface::window_handler window_handler(argv[1], "Perspective Manipulation");

    std::cout << "Welcome to the perspective manipulation tool." << std::endl;
    std::cout << "Coming back to edit mode after optimization / Exit : Esc" << std::endl;
    std::cout << "Reset : " << user_interface::SHORTCUTS_RESET << std::endl;
    std::cout << "Create plan : " << user_interface::SHORTCUTS_CREATE_PLAN << std::endl;
    std::cout << "Vanishing point : " << user_interface::SHORTCUTS_VANISHING_POINTS << std::endl;
    std::cout << "Show/Unshow mesh : " << user_interface::SWITCH_SHOW_MESH << std::endl;
    std::cout << "Border constraint : " << user_interface::SHORTCUT_BORDER_CONSTRAINT << std::endl;
    std::cout << "Fixed line : " << user_interface::SHORTCUT_FIXED_LINE << std::endl;
    std::cout << "Fixed point : " << user_interface::SHORTCUT_FIXED_POINT << std::endl;
    std::cout << "Optimize : " << user_interface::SHORTCUT_OPTIMIZE << std::endl;
    std::cout << "Crop and save : " << user_interface::SHORTCUT_CROP << std::endl;

    while(1){
        window_handler.handle_shortcut(static_cast<char>(cv::waitKey(0)));// Wait for a keystroke in the window

    }

    return 0;
}
