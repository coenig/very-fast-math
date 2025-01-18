#include <iostream>
#include <cmath>
#include <vector>
#include <geometry/images.h>
#include <geometry/vector_2d.h>
#include <geometry/polygon_2d.h>

// Constants
const double PI = 3.141592653589793;

// Function to compute Fresnel integrals using numerical integration 
void computeFresnel(float s, float a, float&C, float&S, int steps = 1000) {
C = 0.0;
S = 0.0;
float ds = s / steps;
for (int i = 0; i <= steps; ++i) {
   float u = i * ds;
   float k = a * u * u;
   C += cos(k) * ds;
   S += sin(k) * ds;
}
}

// Function to generate clothoid coordinates 
vfm::Pol2D generateClothoid(float s_max, float R, int points) {
   vfm::Pol2D curve;
   float a = 1.0 / (2 * R * R); // Curvature constant
   float step = s_max / points;

   for (int i = 0; i <= points; ++i) {
      float s = i * step;
      float C, S;
      computeFresnel(s, a, C, S);
      curve.add({ C, S });
   }

   return curve;
}

double computeAngleAtC(const vfm::Vec2D& A, const vfm::Vec2D& C, const vfm::Vec2D& B) {
   float dx1 = C.x - A.x, dy1 = C.y - A.y;
   float dx2 = B.x - C.x, dy2 = B.y - C.y;
   float dot = dx1 * dx2 + dy1 * dy2;
   float mag1 = sqrt(dx1 * dx1 + dy1 * dy1);
   float mag2 = sqrt(dx2 * dx2 + dy2 * dy2);
   return acos(dot / (mag1 * mag2)); // Angle in radians
}

int main() {
   vfm::Vec2D A{ 100, 100 };
   vfm::Vec2D C{ 300, 300 };
   vfm::Vec2D B{ 300, 400 };

   double dAC = A.distance(C);
   double dCB = C.distance(B);
   double angleAtC = computeAngleAtC(A, C, B);

   // Parameters
   double R = (dAC + dCB) / 2.0 / sin(angleAtC / 2.0); // Radius
   double s_max = R * angleAtC;                       // Arc length
   int points = 10;      // Number of points on the curve

   // Generate the clothoid curve
   auto clothoid = generateClothoid(s_max, R, points);

   float angleAC = A.angle(C);
   float angleCB = C.angle(B);

   // Rotate and translate the curve (example: rotate by 45° and translate by (5, 5))
   clothoid.rotate(angleAC, { 0, 0 });
   clothoid.translate(A);

   vfm::Vec2D lastPoint = clothoid.points_.back();
   vfm::Vec2D translation = B;
   translation.sub(lastPoint);
   clothoid.translate(translation);

   //clothoid.points_.insert(clothoid.points_.begin(), A);
   clothoid.add(B);

   vfm::Image img{ 1000, 1000 };
   vfm::Pol2D arrow{};
   arrow.createArrow(clothoid, 50);

   img.circle(A.x, A.y, 20, vfm::ORANGE);
   img.circle(C.x, C.y, 20, vfm::RED);
   img.circle(B.x, B.y, 20, vfm::ORANGE);
   img.drawPolygon(clothoid, vfm::YELLOW, false, true);
   img.drawPolygon(arrow, vfm::WHITE, true, true);
   img.store("test");

   return 0;
}
