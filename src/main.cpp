#include <iostream>
#include <cmath>
#include <vector>
#include <geometry/images.h>
#include <geometry/vector_2d.h>
#include <geometry/polygon_2d.h>

using namespace vfm;

int main() {
   const int num_lanes{ 2 };
   vfm::Vec2D A{ 100, 100 };
   vfm::Vec2D C{ 800, 400 };
   vfm::Vec2D B{ 200, 900 };

   vfm::Image img{ 1000, 1000 };
   img.fillImg(DARK_GREEN);

   Pol2D clothoid{};
   clothoid = clothoid.bezier(A, C, C, B, 0.001);
   clothoid = clothoid.normalize();
   vfm::Pol2D arrow{};
   vfm::Pol2D arrow2{};
   arrow.createArrow(clothoid, 100);
   img.fillPolygon(arrow, vfm::GREY);

   for (int i = 0; i < num_lanes - 1; i++) {
      auto segments = Pol2D::gestrichelterPfeil(clothoid, 3, 3);
      img.drawPolygons(segments, { { nullptr, std::make_shared<Color>(WHITE)}, {nullptr, nullptr} });
   }

   img.circle(A.x, A.y, 20, vfm::ORANGE);
   img.circle(C.x, C.y, 20, vfm::RED);
   img.circle(B.x, B.y, 20, vfm::ORANGE);
   img.store("test");

   return 0;
}
