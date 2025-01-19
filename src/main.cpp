#include <iostream>
#include <cmath>
#include <vector>
#include <geometry/images.h>
#include <geometry/vector_2d.h>
#include <geometry/polygon_2d.h>

using namespace vfm;

int main() {
   vfm::Vec2D A{ 100, 100 };
   vfm::Vec2D C{ 400, 300 };
   vfm::Vec2D B{ 200, 600 };

   Pol2D clothoid{};
   clothoid = clothoid.bezier(A, C, C, B, 0.01);
   //clothoid = clothoid.normalize();
   vfm::Image img{ 1000, 1000 };
   vfm::Pol2D arrow{};
   vfm::Pol2D arrow2{};
   arrow.createArrow(clothoid, 100);
   arrow2.createArrow(clothoid, 5);

   img.fillPolygon(arrow, vfm::GREY);
   img.drawPolygon(arrow, vfm::YELLOW);
   img.fillPolygon(arrow2, vfm::WHITE);
   img.circle(A.x, A.y, 20, vfm::ORANGE);
   img.circle(C.x, C.y, 20, vfm::RED);
   img.circle(B.x, B.y, 20, vfm::ORANGE);
   img.store("test");

   return 0;
}
