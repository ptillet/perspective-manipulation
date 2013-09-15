#ifndef SRC_INCLUDE_USER_INTERFACE_MODE_MANAGER_H
#define SRC_INCLUDE_USER_INTERFACE_MODE_MANAGER_H

namespace user_interface{

    class window_handler;

    static const char SHORTCUTS_EXIT = 27;
    static const char SHORTCUTS_CREATE_PLAN = 'h';
    static const char SHORTCUTS_VANISHING_POINTS = 'v';
    static const char SHORTCUTS_RESET = 'r';
    static const char SWITCH_SHOW_MESH = 'g';
    static const char SHORTCUT_BORDER_CONSTRAINT = 'b';
    static const char SHORTCUT_FIXED_LINE = 'l';
    static const char SHORTCUT_FIXED_POINT = 'p';
    static const char SHORTCUT_OPTIMIZE = 'o';
    static const char SHORTCUT_CROP = 'c';

    class mode_manager{
    public:
        enum mode_t{
            CREATE_PLANS,
            CREATE_FIXED_LINES,
            MAKE_INTERSECT,
            VANISHING_POINT,
            BORDER_CONSTRAINT,
            FIXED_LINE,
            FIXED_POINT,
            CROP
        };
        mode_manager(window_handler& _window_handler);
        void handle_shortcut(char key_code);
        mode_t mode() const;
        void reset();
    private:
        mode_t mode_;
        window_handler& window_handler_;
    };
}

#endif
