//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "geometry/images.h"
#include "static_helper.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "geometry/stb_image_write.h"
#include "failable.h"
#include <memory>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <random>
#include <cassert>
#include <setjmp.h>

#if __cplusplus >= 201703L // https://stackoverflow.com/a/51536462/7302562 and https://stackoverflow.com/a/60052191/7302562
   #include <filesystem>
#endif

#ifdef _WIN32
   #include <windows.h>
#endif

#define _MAX(a,b) (((a) > (b)) ? (a) : (b))
#define _MIN(a,b) (((a) < (b)) ? (a) : (b))

using namespace vfm;

const std::vector<Image> vfm::Image::MONOSPACE_NEW_CACHED_ASCII_TABLE{ retrieveAsciiTableFromPPMString(StaticHelper::fromSafeString(MONOSPACE_NEW)) };

std::vector<Image> splitImageIntoAsciiChunks(const Image& overall, const int symb_width_pixel, const int offset_left_pixel)
{
   std::vector<Image> vec;

   for (int p = offset_left_pixel; p + symb_width_pixel < overall.getWidth(); p += symb_width_pixel) {
      Image img = overall.copyArea(p, 0, p + symb_width_pixel, overall.getHeight());
      vec.push_back(img);
   }

   return vec;
}

std::vector<Image> vfm::Image::retrieveAsciiTable(const std::string& ppm_file_path, const int symb_width_pixel, const int offset_left_pixel)
{
   Image overall(0, 0);

#if __cplusplus >= 201703L
   if (!std::filesystem::exists(ppm_file_path)) {
      Failable::getSingleton()->addError("File path '" + ppm_file_path + "' not found for retrieving ASCII table.");
   }
#endif

   overall.loadPPM(ppm_file_path);

   return splitImageIntoAsciiChunks(overall, symb_width_pixel, offset_left_pixel);
}

std::vector<Image> vfm::Image::retrieveAsciiTableFromPPMString(const std::string& ppm_string, const int symb_width_pixel, const int offset_left_pixel)
{
   Image overall(0, 0);

   overall.loadPPMFromString(ppm_string);

   return splitImageIntoAsciiChunks(overall, symb_width_pixel, offset_left_pixel);
}

Image vfm::Image::getAsciiImage(const char symbol, const std::vector<Image>& ascii_table)
{
   if (symbol >= ' ' && symbol <= '~' && symbol - 32 < ascii_table.size()) {
      return ascii_table[symbol - 32];
   }

   return ascii_table[ascii_table.size() - 1]; // Tilde as marker that something might have gone wrong.
}

Image::Image(const int width, const int height) : width_(width), height_(height)
{
   resize(width, height);
   fillImg({0, 0, 0, 255});
}

vfm::Image::Image() : Image(0, 0)
{
}

void Image::freeImg()
{
   buf_.clear();

   if (pdf_document_)
   {
      float temp_font_size{ pdf_font_size_ };
      restartPDF(temp_font_size); // Re-start PDF since it was already going.
   }
}

void vfm::Image::freeImg(const int new_width, const int new_height, const Color& color)
{
   resize(new_width, new_height);
   freeImg();
   fillImg(color);
}

void vfm::Image::fillPolygonPDF(const Pol2Df& pol, const Color& fill_col)
{
   fillAndDrawPolygonPDF(pol, 0, fill_col, fill_col, true);
}

void vfm::Image::drawPolygonPDF(const Pol2Df& pol, const float line_width, const Color& line_col)
{
   fillAndDrawPolygonPDF(pol, line_width, BLACK, line_col, false);
}

int counter{0};

void vfm::Image::fillAndDrawPolygonPDF(const Pol2Df& pol, const float line_width, const Color& fill_col, const Color& line_col, const bool fill)
{
   if (pdf_document_) {
      if (pol.points_.empty()) {
         return;
      }
      
      const auto& normalized_fill_color{ fill_col.normalizeToOne() };
      const auto& normalized_line_color{ line_col.normalizeToOne() };

      HPDF_Page_SetLineWidth(pdf_page_, line_width);
      HPDF_Page_SetRGBStroke(pdf_page_, normalized_line_color[0], normalized_line_color[1], normalized_line_color[2]);
      HPDF_Page_SetRGBFill(pdf_page_, normalized_fill_color[0], normalized_fill_color[1], normalized_fill_color[2]);
      HPDF_Page_SetLineJoin(pdf_page_, HPDF_ROUND_JOIN);

      // Set an extended gstate to allow transparent fillings.
      // Note that it is technically not necessary to create
      // such a extgstate every time, when there is no
      // alpha value (!= 255) given.
      // In future it might be more efficient to reset
      // the extgstate, if there are only few objects WITH
      // transparency expected. However, for now, the process
      // is fast enough, and the PDF viewing in typical PDF viewers
      // like SumatraPDF is super-fast.
      HPDF_ExtGState gstate{ HPDF_CreateExtGState(pdf_document_) };
      HPDF_ExtGState_SetAlphaFill(gstate, (float)fill_col.a / 256);
      HPDF_Page_SetExtGState(pdf_page_, gstate);

      HPDF_Page_MoveTo(pdf_page_, pol.points_[0].x - crop_left_, height_ - pol.points_[0].y);

      for (int i = 1; i < pol.points_.size(); i++) {
         HPDF_Page_LineTo(pdf_page_, pol.points_[i].x - crop_left_, height_ - pol.points_[i].y);
      }

      if (fill) {
         HPDF_Page_ClosePathFillStroke(pdf_page_);
      }
      else {
         HPDF_Page_ClosePathStroke(pdf_page_);
      }

      polygons_for_pdf_.push_back({ { { pol, fill ? fill_col : line_col }, fill }, line_width });
   }
}

void Image::fillImg(const Color& col)
{
   int i, n;
   n = width_ * height_;
   freeImg();

   for (i = 0; i < n; ++i) {
      buf_.push_back(col);
   }

   fillPolygonPDF({ { (float)0, (float)0 }, { (float)width_, (float)0 }, { (float)width_, (float)height_ }, { (float)0, (float)height_ } }, col);
}

void vfm::Image::resize(const int new_width, const int new_height)
{
   width_ = new_width;
   height_ = new_height;
   buf_.resize(width_ * height_, BLACK);

   if (pdf_document_) {
      HPDF_Page_SetHeight(pdf_page_, height_);
      HPDF_Page_SetWidth(pdf_page_, width_ - crop_left_ - crop_right_);
   }
}

void Image::putPixelUnsafe(int x, int y, const Color& col)
{
   int ofs;
   ofs = y * width_ + x;

   if (col.a == 255) { // No transparency.
      buf_[ofs].r = col.r;
      buf_[ofs].g = col.g;
      buf_[ofs].b = col.b;
      buf_[ofs].a = col.a;
   }
   else {
      float a2 = (float) col.a / 255.0, a1 = 1 - a2;
      float a = a1 + a2 * (1 - a1);

      float r1 = (float) buf_[ofs].r / 255.0, r2 = (float) col.r / 255.0;
      float g1 = (float) buf_[ofs].g / 255.0, g2 = (float) col.g / 255.0;
      float b1 = (float) buf_[ofs].b / 255.0, b2 = (float) col.b / 255.0;

      if (a == 0) {
         buf_[ofs].r = 0;
         buf_[ofs].g = 0;
         buf_[ofs].b = 0;
      } else {
         buf_[ofs].r = (int) ((r1 * a1 + r2 * a2 * (1 - a1)) * 255.0 / a);
         buf_[ofs].g = (int) ((g1 * a1 + g2 * a2 * (1 - a1)) * 255.0 / a);
         buf_[ofs].b = (int) ((b1 * a1 + b2 * a2 * (1 - a1)) * 255.0 / a);
      }

      buf_[ofs].a = (int) (a * 255.0);
   }
}

void Image::putPixel(int x, int y, const Color& col)
{
   if (x >= 0 && y >= 0) {
      if (x < width_ && y < height_) {
         putPixelUnsafe(x, y, col);
      }
      else {
         if (x >= width_ && expand_dynamically_to_the_right_) {
            auto new_width{ (std::max)(x - width_, expand_dynamically_to_the_right_) + width_ };
            auto copy{ *this };              // TODO: Expensive (better resize by preserving old image)
            resize(new_width, height_);
            insertImage(0, 0, copy, false);  // TODO: Expensive (better resize by preserving old image)
            putPixelUnsafe(x, y, col);
         }
         if (y >= height_ && expand_dynamically_to_the_bottom_) {
            auto new_height{ (std::max)(y - height_, expand_dynamically_to_the_bottom_) + height_ };
            resize(width_, new_height); // Width hasn't changed ==> We can just attach new pixels at the end.
         }
      }
   }
}

Color Image::getPixel(const int x, const int y) const
{
   if (x < 0 || x >= width_ || y < 0 || y >= height_) {
      return BLACK;
   }

   return buf_[y * width_ + x];
}

void Image::store(const std::string& name_raw, const OutputType type, const AddCorrectExtensionToFileName add_correct_extension_to_filename) const
{
   const auto extension_with_dot{ "." + EXTENSIONS_OF_OUTPUT_FORMATS.at(type)};

   const std::string name{ (add_correct_extension_to_filename == AddCorrectExtensionToFileName::Always
        || add_correct_extension_to_filename == AddCorrectExtensionToFileName::IfMissing && !StaticHelper::stringEndsWith(name_raw, extension_with_dot))
      ? name_raw + extension_with_dot
      : name_raw };

   if (type != OutputType::ppm) {
      int channels = 4;
      size_t size = width_ * height_ * channels;
      color_t* buf = new uint8_t[size];
      int i = 0;

      for (Color c : buf_) {
         buf[i++] = c.r;
         if (channels > 2) {
            buf[i++] = c.g;
            buf[i++] = c.b;
         }
         if (channels == 2 || channels == 4) {
            buf[i++] = c.a;
         }
      }

      if (type == OutputType::png) {
         stbi_write_png(name.c_str(), width_, height_, channels, buf, width_ * channels);
      }
      else if (type == OutputType::jpg) {
         stbi_write_jpg(name.c_str(), width_, height_, channels, buf, 100);
      }
      else if (type == OutputType::bmp) {
         stbi_write_bmp(name.c_str(), width_, height_, channels, buf);
      }
      else if (type == OutputType::pdf) {
         if (pdf_document_) {
            HPDF_SaveToFile(pdf_document_, name.c_str());
         }
         else {
            Failable::getSingleton()->addError("Cannot store in PDF format since it has not been tracked. Did you forget to call 'startPDF'?");
         }
      }
      else {
         Failable::getSingleton()->addError("Unknown image type to store.");
      }

      delete[](buf);
   }
   else {
      storePPM(name);
   }
}

void Image::storePPM(const std::string& name) const
{
   int i, j;
   auto *fp = fopen(name.c_str(), "wb");
   (void) fprintf(fp, "P6\n%d %d\n255\n", width_, height_);

   for (j = 0; j < height_; ++j) {
      for (i = 0; i < width_; ++i) {
         color_t color[3];
         auto p = getPixel(i, j);
         color[0] = p.r;
         color[1] = p.g;
         color[2] = p.b;
         (void) fwrite(color, 1, 3, fp);
      }
   }

   (void) fclose(fp);
}

void Image::loadPPMFromString(const std::string& str)
{
   int first_occ = str.find('\n');
   int snd_occ = str.find('\n', first_occ + 1);
   int third_occ = str.find('\n', snd_occ + 1);
   auto split2 = StaticHelper::split(str.substr(first_occ + 1, snd_occ - 3), ' ');
   width_ = std::stoi(split2[0]);
   height_ = std::stoi(split2[1]);
   buf_.clear();

   for (int i = third_occ + 1; i < str.size();) {
      Color c(str[i++], str[i++], str[i++], 255);
      buf_.push_back(c);
   }
}

void Image::loadPPM(const std::string& name)
{
   std::ifstream file(name, std::ios::binary | std::ios::ate);
   std::streamsize size = file.tellg();
   file.seekg(0, std::ios::beg);

   std::vector<char> buffer(size);
   if (file.read(buffer.data(), size))
   {
      std::string str(buffer.begin(), buffer.end());
      loadPPMFromString(str);
   }
}

void vfm::Image::paintDashedSegment(const float xx1, const float yy1, const float xx2, const float yy2, const float thickness, const Color color)
{
   Vec2Df dir{ xx2, yy2 };
   dir.sub(xx1, yy1);
   Vec2D ortho{ dir };
   ortho.ortho();
   ortho.setLength(thickness / 2);

   Vec2Df neworig1{ xx1, yy1 };
   Vec2Df neworig2{ xx1, yy1 };
   Vec2Df newdest1{ xx2, yy2 };
   Vec2Df newdest2{ xx2, yy2 };
   neworig1.add(ortho);
   neworig2.sub(ortho);
   newdest1.add(ortho);
   newdest2.sub(ortho);

   fillPolygon({ Vec3D(neworig1), Vec3D(neworig2), Vec3D(newdest2), Vec3D(newdest1) }, color);
}

void vfm::Image::dashed_line(const float x1, const float y1, const float x2, float y2, const float thickness, const Color& color, const float dash_length)
{
   float length = sqrtf(powf(x2 - x1, 2) + powf(y2 - y1, 2));
   int seg_num = length / dash_length;

   float x_dir = dash_length * (x2 - x1) / length;
   float y_dir = dash_length * (y2 - y1) / length;
   float xx1 = x1, xx2 = xx1 + x_dir, yy1 = y1, yy2 = yy1 + y_dir;
   bool paint = true;

   for (int i = 0; i < seg_num; i++) {
      if (paint) {
         paintDashedSegment(xx1, yy1, xx2, yy2, thickness, color);
      }

      xx1 = xx2;
      xx2 = xx1 + x_dir;
      yy1 = yy2;
      yy2 = yy1 + y_dir;
      paint = !paint;
   }

   if (paint) {
      float remaining = length - seg_num * dash_length;
      x_dir = remaining * (x2 - x1) / length;
      y_dir = remaining * (y2 - y1) / length;

      xx2 = xx1 + x_dir;
      yy2 = yy1 + y_dir;

      paintDashedSegment(xx1, yy1, xx2, yy2, thickness, color);
   }
}

void vfm::Image::dashed_line(const Vec2Df& src, const Vec2Df& dest, const float thickness, const Color & color, const float dash_length)
{
   dashed_line(src.x, src.y, dest.x, dest.y, thickness, color);
}

//DrawLine(x1, x2, y1, y2) {
//   x = x1;
//   y = y1;
//   dx = x2 − x1;
//   dy = y2 − y1;
//   d = 2 * dy − dx; // discriminator
//
//   // Euclidean distance of point (x,y) from line (signed)
//   D = 0;
//
//   // Euclidean distance between points (x1, y1) and (x2, y2)
//   length = sqrt(dx * dx + dy * dy);
//
//   sin = dx / length;
//   cos = dy / length;
//   while (x <= x2) {
//      IntensifyPixels(x, y − 1, D + cos);
//      IntensifyPixels(x, y, D);
//      IntensifyPixels(x, y + 1, D − cos);
//      x = x + 1
//         if (d <= 0) {
//            D = D + sin;
//            d = d + 2 * dy;
//         }
//         else {
//            D = D + sin − cos;
//            d = d + 2 * (dy − dx);
//            y = y + 1;
//         }
//   }
//}

void Image::clipLine(int& x1, int& y1, int& x2, int& y2)
{
   constexpr auto clipX1{ 0 };
   constexpr auto clipY1{ 0 };
   const auto clipX2{ width_ - 1 };
   const auto clipY2{ height_ - 1 };

   // Calculate line slope and y-intercept
   const float m = x1 == x2 ? std::numeric_limits<float>::infinity() : (float)(y2 - y1) / (float)(x2 - x1);
   const float b = y1 - m * x1;
   const int oldx{ x1 }; // Store for infinite slope case.

   // Clip line to left edge of clipping area
   if (x1 < clipX1 && x2 >= clipX1) {
      y1 = m * clipX1 + b;
      x1 = clipX1;
   }
   else if (x2 < clipX1 && x1 >= clipX1) {
      y2 = m * clipX1 + b;
      x2 = clipX1;
   }

   // Clip line to right edge of clipping area
   if (x1 > clipX2 && x2 <= clipX2) {
      y1 = m * clipX2 + b;
      x1 = clipX2;
   }
   else if (x2 > clipX2 && x1 <= clipX2) {
      y2 = m * clipX2 + b;
      x2 = clipX2;
   }

   // Clip line to top edge of clipping area
   if (y1 < clipY1 && y2 >= clipY1) {
      x1 = (clipY1 - b) / m;
      y1 = clipY1;
   }
   else if (y2 < clipY1 && y1 >= clipY1) {
      x2 = (clipY1 - b) / m;
      y2 = clipY1;
   }

   // Clip line to bottom edge of clipping area
   if (y1 > clipY2 && y2 <= clipY2) {
      x1 = (clipY2 - b) / m;
      y1 = clipY2;
   }
   else if (y2 > clipY2 && y1 <= clipY2) {
      x2 = (clipY2 - b) / m;
      y2 = clipY2;
   }

   if (!std::isfinite(m)) {
      x1 = oldx;
      x2 = oldx;
   }
}

void Image::lineUnsafe(const float xx1, const float yy1, const float xx2, const float yy2, const Color& color)
{
   // TODO: Not supported in PDF, yet.
   
   if (xx1 < 0 && xx2 < 0 || (!expand_dynamically_to_the_right_ && xx1 >= width_ && xx2 >= width_) ||
      yy1 < 0 && yy2 < 0 || (!expand_dynamically_to_the_bottom_ && yy1 >= height_ && yy2 > height_)) {
      return;
   }

   int x1{ (int) xx1 };
   int x2{ (int) xx2 };
   int y1{ (int) yy1 };
   int y2{ (int) yy2 };

   clipLine(x1, y1, x2, y2);

   const Pol2D screen_pol{ {-1, -1}, {(float)width_, -1}, {(float)width_, (float)height_}, {-1, (float)height_} };
   const Vec2D p1{ (float)x1, (float)y1 };
   const Vec2D p2{ (float)x2, (float)y2 };
   assert(screen_pol.isPointWithin(p1));
   assert(screen_pol.isPointWithin(p2));

   const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));

   if (steep) {
      std::swap(x1, y1);
      std::swap(x2, y2);
   }

   if (x1 > x2) {
      std::swap(x1, x2);
      std::swap(y1, y2);
   }

   const int dx = x2 - x1;
   const int dy = fabs(y2 - y1);

   int error = dx / 2.0f;
   const int ystep = (y1 < y2) ? 1 : -1;
   int y = (int) y1;

   const int maxX = (int) x2;

   for (int x = (int) x1; x <= maxX; x++) {
      if (steep) {
         putPixel(y, x, color);
      }
      else {
         putPixel(x, y, color);
      }

      error -= dy;
      if (error < 0) {
         y += ystep;
         error += dx;
      }
   }
}

void vfm::Image::lineUnsafe(const Vec2Df& src, const Vec2Df& dest, const Color& color)
{
   lineUnsafe(src.x, src.y, dest.x, dest.y, color);
}

void Image::circle(
   const int x0,
   const int y0,
   const int radius,
   const Color& col)
{
   // TODO: Not supported in PDF, yet.

   int f = 1 - radius;
   int ddF_x = 0;
   int ddF_y = -2 * radius;
   int x = 0;
   int y = radius;

   putPixel(x0, y0 + radius, col);
   putPixel(x0, y0 - radius, col);
   putPixel(x0 + radius, y0, col);
   putPixel(x0 - radius, y0, col);

   while (x < y)
   {
      if (f >= 0)
      {
         y--;
         ddF_y += 2;
         f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x + 1;
      putPixel(x0 + x, y0 + y, col);
      putPixel(x0 - x, y0 + y, col);
      putPixel(x0 + x, y0 - y, col);
      putPixel(x0 - x, y0 - y, col);
      putPixel(x0 + y, y0 + x, col);
      putPixel(x0 - y, y0 + x, col);
      putPixel(x0 + y, y0 - x, col);
      putPixel(x0 - y, y0 - x, col);
   }
}

void vfm::Image::rectangle(const Vec2Df& tl, const Vec2Df& br, const Color& c, const bool center, const float elevation, const bool paint_nodes, const bool print_coordinates)
{
   rectangle(tl.x, tl.y, (br.x - tl.x), (br.y - tl.y), c, center, elevation, paint_nodes, print_coordinates);
}

void vfm::Image::rectangle(const float x0, const float y0, const float width, const float height, const Color& col, const bool center, const float elevation, const bool paint_nodes, const bool print_coordinates)
{
   float ww = center ? width / 2 : 0;
   float hh = center ? height / 2 : 0;

   Vec3Df tl{ x0 - ww, y0 - hh, elevation };
   Vec3Df br{ x0 + width - ww, y0 + height - hh, elevation };

   drawPolygon({ tl, { br.x, tl.y, elevation }, br, { tl.x, br.y, elevation } }, col, true, paint_nodes, print_coordinates);
}

void vfm::Image::fillRectangle(const float x0, const float y0, const float width, const float height, const Color& col, const bool center, const float elevation)
{
   float ww = center ? width / 2 : 0;
   float hh = center ? height / 2 : 0;

   Vec3Df tl{ x0 - ww, y0 - hh, elevation };
   Vec3Df br{ x0 + width - ww, y0 + height - hh, elevation };

   fillPolygon({ tl, { br.x, tl.y, elevation }, br, { tl.x, br.y, elevation } }, col);
   //fillQuad(tl, {br.x, tl.y}, br, {tl.x, br.y}, col);
}

void vfm::Image::fillRectangle(const Vec2Df& tl, const Vec2Df& br, const Color& c, const bool center, const float elevation)
{
   fillRectangle(tl.x, tl.y, br.x - tl.x, br.y - tl.y, c, center, elevation);
}

Image vfm::Image::copyArea(const int x0, const int y0, const int x1, const int y1) const
{
   //assert(x0 <= x1 && x1 <= width_);  // We want to allow "copying" pixels from outside the image.
   //assert(y0 <= y1 && y1 <= height_); // They are returned as BLACK.

   Image img(x1 - x0, y1 - y0);

   for (int x = 0; x < x1 - x0; x++) {
      for (int y = 0; y < y1 - y0; y++) {
         auto pix = getPixel(x + x0, y + y0);
         img.putPixelUnsafe(x, y, pix);
      }
   }

   return img;
}

void Image::fillArea(const int i, const int j, const Color& oldColor, const Color& newColor)
{
   // TODO: Not supported in PDF, yet.

   std::vector<std::pair<int, int>> stack;
   stack.push_back({i, j});

   while(!stack.empty()) {
      int x = stack.back().first;
      int y = stack.back().second;
      stack.pop_back();

      if (0 <= x && x < width_ && 0 <= y && y < height_) {
         if (getPixel(x, y) == oldColor) {
            putPixelUnsafe(x, y, newColor);

            stack.push_back({x, y + 1});
            stack.push_back({x, y - 1});
            stack.push_back({x + 1, y});
            stack.push_back({x - 1, y});
         }
      }
   }
}

void Image::fillArea(const int x, const int y, const Color& newColor)
{
   fillArea(x, y, getPixel(x, y), newColor);
}

void Image::quadBezier(const int x1, const int y1, const int x2, const int y2, const int x3, const int y3, const Color& col)
{
   // TODO: Not supported in PDF, yet. (Will actually be handed to line, but nicer would be to use native PDF curve drawing.)

   const int segments = 20;
   int i;
   double pts[segments + 1][2];

   for (i = 0; i <= segments; ++i) {
        double t = (double) i / (double) segments;
        double a = pow((1.0 - t), 2.0);
        double b = 2.0 * t * (1.0 - t);
        double c = pow(t, 2.0);
        double x = a * x1 + b * x2 + c * x3;
        double y = a * y1 + b * y2 + c * y3;
        pts[i][0] = x;
        pts[i][1] = y;
    }

    for (i = 0; i < segments; ++i) {
        int j = i + 1;
        lineUnsafe(pts[i][0], pts[i][1], pts[j][0], pts[j][1], col);
    }
}

void Image::cubeBezier(const int x1, const int y1, const int x2, const int y2, const int x3, const int y3, const int x4, const int y4, const Color& col)
{
   // TODO: Not supported in PDF, yet. (Will actually be handed to line, but nicer would be to use native PDF curve drawing.)

   const int segments = 20;
   int i;
   double pts[segments + 1][2];

   for (i=0; i <= segments; ++i) {
      double t = (double) i / (double) segments;

      double a = pow((1.0 - t), 3.0);
      double b = 3.0 * t * pow((1.0 - t), 2.0);
      double c = 3.0 * pow(t, 2.0) * (1.0 - t);
      double d = pow(t, 3.0);

      double x = a * x1 + b * x2 + c * x3 + d * x4;
      double y = a * y1 + b * y2 + c * y3 + d * y4;
      pts[i][0] = x;
      pts[i][1] = y;
   }

   for (i=0; i < segments; ++i) {
      int j = i + 1;
      lineUnsafe(pts[i][0], pts[i][1], pts[j][0], pts[j][1], col);
   }
}

void vfm::Image::drawPolygon(const Polygon3D<float>& pol, const Color& col, const bool close_polygon, const bool paint_nodes, const bool print_coordinates)
{
   Pol2D translated_pol{ translator_->translatePolygon(pol) };

   if (translated_pol.points_.empty()) return;

   for (int i = 0; i < translated_pol.points_.size() + close_polygon - 1; i++) {
      const auto& current_point{ translated_pol.points_[i] };
      const auto& next_point{ translated_pol.points_[(i + 1) % translated_pol.points_.size()] };

      lineUnsafe(current_point.x, current_point.y, next_point.x, next_point.y, col);

      if (paint_nodes) {
         circle(current_point.x, current_point.y, 3, i == 0 ? RED : col);
         circle(next_point.x, next_point.y, 3, i == pol.points_.size() - 1 ? RED : col);
      }

      if (print_coordinates) {
         writeAsciiText(current_point.x, current_point.y, current_point.serializeRoundedDown(), CoordTrans::dont_do_it, false, FUNC_IGNORE_BLACK_CONVERT_TO_YELLOW);
         writeAsciiText(next_point.x, next_point.y, next_point.serializeRoundedDown(), CoordTrans::dont_do_it, false, FUNC_IGNORE_BLACK_CONVERT_TO_YELLOW);
      }
   }

   drawPolygonPDF(translated_pol, 0.3, col);
}

void vfm::Image::fillPolygon(const Polygon3D<float>& pol, const Color& col)
{
   Pol2D translated_pol{ translator_->translatePolygon(pol) };

   if (!translated_pol.points_.empty()) {
      auto bb = translated_pol.getBoundingBox();
      float x1 = bb->upper_left_.x;
      float y1 = bb->upper_left_.y;
      float x2 = bb->lower_right_.x;
      float y2 = bb->lower_right_.y;

      for (float i = (std::max)((int)y1, 0); i <= (std::min)((int)y2, (int)height_) + 1; i += 0.999) {
         auto intersections{ translated_pol.intersect(LinSeg2D{ { x1, i }, { x2, i } }) };
         std::sort(intersections.begin(), intersections.end(), [](const Vec2D& v1, const Vec2D& v2) {
            return v1.x < v2.x;
            });

         for (size_t ii = 0; ii + 1 < intersections.size(); ii += 2) {
            lineUnsafe(
               (std::max)((int)intersections[ii].x, 0),
               i,
               (std::min)((int)intersections[ii + 1].x, (int)width_),
               i,
               col);
         }
      }
   }

   fillPolygonPDF(translated_pol, col);
}

void vfm::Image::drawPolygons(const std::vector<Polygon2D<float>>& pols)
{
   return drawPolygons(pols, { { std::make_shared<Color>(WHITE), nullptr } });
}

void vfm::Image::drawPolygons(
   const std::vector<Polygon2D<float>>& pols,
   const std::vector<std::pair<std::shared_ptr<Color>, std::shared_ptr<Color>>>& frame_and_fill)
{
   int i{ 0 };
   const int num{ (int) frame_and_fill.size() };

   for (const auto& pol : pols) {
      auto color_fill = frame_and_fill.at(i % num).second;
      auto color_frame = frame_and_fill.at(i % num).first;
      if (color_fill) fillPolygon(pol, *color_fill);
      if (color_frame) drawPolygon(pol, *color_frame, true);
      i++;
   }
}

Image vfm::Image::scale(const Vec2Df factor)
{
   // TODO: Not supported in PDF, yet.

   Image result(width_ * factor.x, height_ * factor.y); // Deliberately cutting off possible fraction pixel. TODO: Is this always ok?

   for (int x = 0; x < result.width_; x++) {
      for (int y = 0; y < result.height_; y++) {
         result.putPixelUnsafe(x, y, getPixel(x * width_ / result.width_, y * height_ / result.height_));
      }
   }

   return result;
}


/// \brief Standard triangle triangulation algorithm: Split generic triangle into
/// one with a flat bottom and on with a flat top. Both of those then can be rasterized trivially.
void vfm::Image::fillTriangle(Vec2Df v1, Vec2Df v2, Vec2Df v3, const Color& col)
{
   // Sort the vertices by y1 < y2 < y3
   // Unrolled bubblesort
   if (v1.y > v2.y)
      std::swap(v1, v2);
   if (v2.y > v3.y)
      std::swap(v2, v3);
   if (v1.y > v2.y)
      std::swap(v1, v2);


   if (v2.y == v3.y)
   {
      fillBottomFlatTriangle(v1, v2, v3, col);
   }
   // check for trivial case of top-flat triangle
   else if (v1.y == v2.y)
   {
      fillTopFlatTriangle(v1, v2, v3, col);
   }
   else
   {
      // general case - split the triangle in a topflat and bottom-flat one
      Vec2Df v_mid(
         (int)(v1.x + ((float)(v2.y - v1.y) / (float)(v3.y - v1.y)) * (v3.x - v1.x)), v2.y);

      fillBottomFlatTriangle(v1, v2, v_mid, col);
      fillTopFlatTriangle(v2, v_mid, v3, col);
   }
}

void vfm::Image::fillQuad(Vec2Df v1, Vec2Df v2, Vec2Df v3, Vec2Df v4, const Color& col)
{
   fillTriangle(v1, v2, v3, col);
   fillTriangle(v2, v3, v4, col);
   fillPolygonPDF({ v1, v2, v4, v3 }, col);
}

void vfm::Image::fillTriangleStrip(const Polygon2D<float>& pol, const Color& col)
{
   for (int vertex_index = 2; vertex_index < pol.points_.size(); vertex_index++)
   {
      fillTriangle(pol.points_[vertex_index - 2], pol.points_[vertex_index - 1], pol.points_[vertex_index], col);
   }
}

void vfm::Image::fillBottomFlatTriangle(Vec2Df v1, Vec2Df v2, Vec2Df v3, const Color& col)
{
   float invslope1 = (v2.x - v1.x) / (v2.y - v1.y);
   float invslope2 = (v3.x - v1.x) / (v3.y - v1.y);

   float curx1 = v1.x;
   float curx2 = v1.x;

   for (int scanlineY = v1.y; scanlineY <= v2.y; scanlineY++)
   {
      int max_x = _MAX(curx1, curx2);
      for (int x = (int) _MIN(curx1, curx2); x <= max_x; x++)
      {
         putPixel(x, scanlineY, col);
      }
      curx1 += invslope1;
      curx2 += invslope2;
   }
}

void vfm::Image::fillTopFlatTriangle(Vec2Df v1, Vec2Df v2, Vec2Df v3, const Color& col)
{
   float invslope1 = (v3.x - v1.x) / (v3.y - v1.y);
   float invslope2 = (v3.x - v2.x) / (v3.y - v2.y);

   float curx1 = v3.x;
   float curx2 = v3.x;

   for (int scanlineY = v3.y; scanlineY > v1.y; scanlineY--)
   {
      int max_x = _MAX(curx1, curx2);
      for (int x = (int) _MIN(curx1, curx2); x <= max_x; x++)
      {
         putPixel(x, scanlineY, col);
      }
      //drawLine((int)curx1, scanlineY, (int)curx2, scanlineY);
      curx1 -= invslope1;
      curx2 -= invslope2;
   }
}

std::shared_ptr<VisTranslator> vfm::Image::getTranslator() const
{
   return translator_;
}

int vfm::Image::getWidth() const
{
   return width_;
}

int vfm::Image::getHeight() const
{
   return height_;
}

void vfm::Image::deactivateAutoExpandToTheRight()
{
   expand_dynamically_to_the_right_ = 0;
}

void vfm::Image::autoExpandToTheRight(const unsigned int val)
{
   expand_dynamically_to_the_right_ = val;
}

unsigned int vfm::Image::getAutoExpandToTheRight() const
{
   return expand_dynamically_to_the_right_;
}

void vfm::Image::deactivateAutoExpandToTheBottom()
{
   expand_dynamically_to_the_bottom_ = 0;
}

void vfm::Image::autoExpandToTheBottom(const unsigned int val)
{
   expand_dynamically_to_the_bottom_ = val;
}

unsigned int vfm::Image::getAutoExpandToTheBottom() const
{
   return expand_dynamically_to_the_bottom_;
}

std::vector<Color> vfm::Image::getRawImage()
{
   return buf_;
}

void vfm::Image::insertImage(const int x, const int y, const Image& image, const bool center, const std::function<Color(const Color& oldPix, const Color& newPix)>& f)
{
   // TODO: Not supported in PDF, yet.

   int xx = 0, yy = 0;

   if (center) {
      xx -= image.width_ / 2;
      yy -= image.height_ / 2;
   }

   for (int i = 0; i < image.width_; i++) {
      for (int j = 0; j < image.height_; j++) {
         int corx = x + i + xx;
         int cory = y + j + yy;
         if (expand_dynamically_to_the_bottom_ || expand_dynamically_to_the_right_ || (corx < width_ && cory < height_)) {
            putPixel(corx, cory, f(getPixel(corx, cory), image.getPixel(i, j)));
         }
      }
   }

   for (const auto& pol : image.polygons_for_pdf_) {
      float hh{ (float) image.getHeight() };
      float ww{ (float) image.getWidth() };
      float xf{ (float) x };
      float yf{ (float) y };
      Pol2D other{ { xf + ww, yf }, { xf + ww, yf + hh }, { xf, yf + hh }, { xf, yf }};
      Pol2Df copy{ pol.first.first.first };
      copy.translate({ (float)x, (float)y });

      if (other.getBoundingBox()->intersect(*copy.getBoundingBox())) {
         auto pol_clipped{ copy.clipPolygonsOneConvex(other) };
         if (pol.first.second) {
            fillPolygonPDF(pol_clipped, pol.first.first.second);
         }
         else {
            drawPolygonPDF(pol_clipped, pol.second, pol.first.first.second);
         }
      }
   }

   //using Text = std::string;
   //using TextRectangle = std::pair<Vec2Df, Vec2Df>;
   //using TextWithCoordinates = std::pair<Text, TextRectangle>;
   //using TextWithCoordinatesAndColor = std::pair<TextWithCoordinates, Color>;

   for (const auto& text_comp : image.text_for_pdf_) {
      Color color{ text_comp.second };
      TextRectangle rect{ text_comp.first.second };
      std::string text{ text_comp.first.first };

      drawTextPDF(text, rect, color);
   }
}

void vfm::Image::writeAsciiText(
   const float x,
   const float y,
   const std::string text,
   const CoordTrans translate_position,
   const bool center,
   const std::function<Color(const Color& oldPix, const Color& newPix)>& f,
   const std::vector<Image>& ascii_table)
{
   // TODO: Not supported in PDF, yet.

   Vec2D point{ translate_position == CoordTrans::do_it ? getTranslator()->translate({x, y}) : Vec2D{x, y} };
   int xx = point.x;
   int yy = point.y;
   int xxx = 0;
   int yyy = 0;

   if (center) {
      for (int i = 0; i < text.size(); i++) {
         xxx -= getAsciiImage(text[i], ascii_table).width_;
         yyy = -((std::max)(yyy, (int) getAsciiImage(text[i], ascii_table).height_));
      }

      yy += yyy / 2;
      xx += xxx / 2;
   }

   xx = (std::max)(0, xx);
   yy = (std::max)(0, yy);

   if (!expand_dynamically_to_the_bottom_) {
      yy = (std::min)(yy, getHeight() - 20);
   }

   if (!expand_dynamically_to_the_right_) {
      xx = (std::min)(xx, getWidth() - 20);
   }

   for (int i = 0; i < text.size(); i++) {
      if (text[i] == '\n') {
         xx = x + xxx;
         yy += getAsciiImage(text[0], ascii_table).height_ + 1;
      }
      else {
         Image img = getAsciiImage(text[i], ascii_table);
         insertImage(xx, yy, img, false, f);
         xx += img.width_;
      }
   }

   drawTextPDF(text, xx, yy, f(BLUE, BLUE));
}

void vfm::Image::drawTextPDF(const std::string& text, const float xx, const float yy, const Color& color)
{
   TextRectangle text_rectangle{ {(float)(xx - 75 - crop_left_), (float)(height_ - yy + 1)}, {(float)(xx - crop_left_ + 25), (float)(height_ - yy + 1)} };
   drawTextPDF(text, text_rectangle, color);
}

void vfm::Image::drawTextPDF(const std::string& text, const TextRectangle& text_rectangle, const Color& color)
{
   if (pdf_document_) {
      auto color_norm{ color.normalizeToOne() };
      HPDF_Font font = HPDF_GetFont(pdf_document_, "Helvetica", NULL);
      HPDF_Page_SetFontAndSize(pdf_page_, font, pdf_font_size_);
      HPDF_Page_SetRGBFill(pdf_page_, color_norm[0], color_norm[1], color_norm[2]);
      HPDF_Page_BeginText(pdf_page_);
      HPDF_Page_TextRect(pdf_page_, text_rectangle.first.x, text_rectangle.first.y, text_rectangle.second.x, text_rectangle.second.y, text.c_str(), HPDF_TextAlignment::HPDF_TALIGN_CENTER, NULL);
      HPDF_Page_EndText(pdf_page_);

      text_for_pdf_.push_back({ {text, text_rectangle}, color });
   }
}

void vfm::Image::setTranslator(const std::shared_ptr<VisTranslator> function)
{
   translator_ = function;
}

// PDF stuff
jmp_buf env;

void error_handler(HPDF_STATUS   error_no,
   HPDF_STATUS   detail_no,
   void* user_data)
{
   printf("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
      (HPDF_UINT)detail_no);
   longjmp(env, 1);
}

int vfm::Image::startOrKeepUpPDF(const float font_size)
{
   return pdf_document_ ? 0 : restartPDF(font_size);
}

int vfm::Image::restartPDF(const float font_size)
{
   flushAndStopPDF();

   pdf_font_size_ = font_size;
   pdf_document_ = HPDF_New(error_handler, NULL);

   if (!pdf_document_) {
      printf("error: cannot create PdfDoc object\n");
      return 1;
   }

   if (setjmp(env)) {
      flushAndStopPDF();
      return 1;
   }

   pdf_page_ = HPDF_AddPage(pdf_document_);
   HPDF_Page_SetHeight(pdf_page_, height_);
   HPDF_Page_SetWidth(pdf_page_, width_ - crop_left_ - crop_right_);
   HPDF_Font font = HPDF_GetFont(pdf_document_, "Helvetica", NULL);
   HPDF_Page_SetFontAndSize(pdf_page_, font, pdf_font_size_);

   return 0;
}

void vfm::Image::flushAndStopPDF()
{
   if (pdf_document_) {
      HPDF_Free(pdf_document_);
      pdf_document_ = nullptr;
      pdf_page_ = nullptr;
      polygons_for_pdf_.clear();
      text_for_pdf_.clear();
   }
}

void vfm::Image::setCropLeftRightPDF(const float left, const float right)
{
   crop_left_ = left;
   crop_right_ = right;
}

HPDF_Doc vfm::Image::getPdfDocument() const
{
   return pdf_document_;
}

void vfm::Image::setPdfDocument(const HPDF_Doc doc)
{
   pdf_document_ = doc;
}
// EO PDF stuff

#ifdef _WIN32
std::string vfm::Image::ExePath()
{
   char buffer[MAX_PATH];
   GetModuleFileName(NULL, buffer, MAX_PATH);
   std::string::size_type pos = std::string(buffer).find_last_of("\\/");
   return std::string(buffer).substr(0, pos);
}
#endif
