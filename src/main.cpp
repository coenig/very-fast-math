#include <iostream>
#include <cmath>
#include <vector>
#include <geometry/images.h>
#include <geometry/vector_2d.h>
#include <geometry/polygon_2d.h>

using namespace vfm;

int main() {
   vfm::Vec2D A{ 100, 100 };
   vfm::Vec2D B{ 200, 800 };
   vfm::Vec2D C{ 450, 400 };

   Pol2D clothoid{};
   clothoid = clothoid.bezier(A, C, C, B, 0.001);
   //clothoid.insertNodesOnExistingEdges(10, false);
   clothoid = clothoid.normalize();
   vfm::Image img{ 1000, 1000 };
   img.fillImg(DARK_GREEN);
   vfm::Pol2D arrow{};
   vfm::Pol2D arrow2{};
   arrow.createArrow(clothoid, 100);

   auto segments = Pol2D::gestrichelterPfeil(clothoid, 3, 3);
   std::cout << "segments.size(): " << segments.size() << std::endl;

   img.fillPolygon(arrow, vfm::GREY);
   img.drawPolygon(arrow, vfm::WHITE);
   img.drawPolygons(segments, { { nullptr, std::make_shared<Color>(WHITE)}, {nullptr, nullptr}});
   img.circle(A.x, A.y, 20, vfm::ORANGE);
   img.circle(C.x, C.y, 20, vfm::RED);
   img.circle(B.x, B.y, 20, vfm::ORANGE);
   img.store("test");

   return 0;
}
