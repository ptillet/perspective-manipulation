======================================
PERSPECTIVE MANIPULATION V 1.0 BETA
======================================

=====================================
USAGE
=====================================


./build/bin/perspective_manipulation INPUT_IMG

Coming back to edit mode after optimization / Exit: Esc
Reset : r
Create plan : h
Vanishing point : v
Show/Unshow mesh : g
Border constraint : b
Fixed line : l
Fixed point : p
Optimize : o
Crop / Save : c

=====================================
KNOWN ISSUES
=====================================

* There is no security to prevent you from setting constraints out of the
image range (in the border zone). If you do so, the optimization will find no solution
and the resulting image will be all white.

* To prevent conflict between vanishing points constraint linearization, 
the homography constraints have been weakened.

* Only possible to crop in top-left - bottom right diagonal
