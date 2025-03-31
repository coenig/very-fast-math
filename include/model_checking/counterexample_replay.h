//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "geometry/images.h"
#include "fsm.h"
#include <thread>


namespace vfm {

inline void removeButton(Image& img, Image& background, const PainterButtonDescription desc) {
   int offset_x = desc.second.first[2] / 2;
   int offset_y = desc.second.first[3] / 2;
   Image sw_chunk = background.copyArea(
      desc.second.first[0] - offset_x - 2, 
      desc.second.first[1] - offset_y - 2, 
      desc.second.first[0] + desc.second.first[2] - offset_x + 2,
      desc.second.first[1] + desc.second.first[3] - offset_y + 2);
   img.insertImage(desc.second.first[0], desc.second.first[1], sw_chunk, true, [](const Color& oldPix, const Color& newPix) -> Color {
      return Color(newPix.r, newPix.g, newPix.b, newPix.a);
   });
}

inline void paintButton(Image& img, const PainterButtonDescription desc) {
   img.fillRectangle(desc.second.first[0], desc.second.first[1], desc.second.first[2], desc.second.first[3], desc.second.second.first);
   img.rectangle(desc.second.first[0], desc.second.first[1], desc.second.first[2], desc.second.first[3], desc.second.second.second);
   img.writeAsciiText(desc.second.first[0], desc.second.first[1], desc.first.second, CoordTrans::do_it, true, FUNC_IGNORE_BLACK_CONVERT_TO_BLACK);
}

enum class HorizontalAlignment {
   centered,
   left,
   right
};

inline void defaultPaintFunction(
   const DataPack& data, 
   Image& img,
   std::vector<PainterVariableDescription>& variables_to_be_painted,
   std::vector<PainterButtonDescription>& button_descriptions,
   const std::string& background_image_path,
   bool& first,
   Image& background_image,
   Image& sw_chunk,
   int& sw_chunk_x,
   int& sw_chunk_y,
   int& i) {

   const HorizontalAlignment align{ HorizontalAlignment::left };
   const int line_height = 20;
   const int char_width = 12;
   const int var_box_top = 5;
   const int var_box_pad_left = 10;
   const int var_box_pad_right = 10;
   const int var_box_pad_top = 10;
   const int var_box_pad_bottom = 10;
   const int shadow_offset = 8;
   const Color shadow_color = Color(100, 100, 150, 100);
   const Color shadow_frame_color = Color(100, 100, 150, 200);
   const Color rectangle_background_color = EGGSHELL_TRANS;
   const Color rectangle_frame_color = BLACK;

   std::vector<std::pair<std::string, std::function<Color(const Color& oldPix, const Color& newPix)>>> var_strs;
   int max_length = 0;
   bool has_any_changed = false;
   i++;

   for (auto& var : variables_to_be_painted) {
      float var_val = StaticHelper::stringStartsWith(var.first, ".") ? data.getValFromArray(var.first, 0) : data.getSingleVal(var.first);
      bool has_changed = var_val != var.second.second;
      std::string res_str{};
      std::function<Color(const Color& oldPix, const Color& newPix)> fct = FUNC_IGNORE_BLACK_CONVERT_TO_BLACK;

      if (data.isDeclared(var.first)) {
         if (var.second.first == 0) {
            if (var_val) {
               res_str = "TRUE";
               fct = FUNC_IGNORE_BLACK_CONVERT_TO_DARK_GREEN;
            }
            else {
               res_str = "FALSE";
               fct = FUNC_IGNORE_BLACK_CONVERT_TO_RED;
            }
         }
         else if (var.second.first == 1) {
            res_str = std::to_string(var_val);
         }
         else if (var.second.first == 2) {
            res_str = data.getSingleValAsEnumName(var.first) + " (" + std::to_string((int)var_val) + ")";
         }
         else if (var.second.first == 3) {
            res_str = std::to_string((int)var_val);
         }
      }
      else {
         res_str = "-";
      }

      std::string complete_str = var.first + " = " + res_str;

      if (has_changed) {
         fct = FUNC_RETURN_TARGET_PIXEL;
         var.second.second = var_val;
         has_any_changed = true;
      }

      max_length = std::max(max_length, (int)complete_str.size());
      var_strs.push_back({ complete_str , fct });

      for (const auto& button : button_descriptions) {
         if (!first && has_changed && button.first.first == var.first) {
            if (var_val) {
               paintButton(img, button);
            }
            else {
               removeButton(img, background_image, button);
            }
         }
      }
   }

   if (background_image_path.empty()) {
      background_image.freeImg(img.getWidth(), img.getHeight(), BLACK);
      background_image.insertImage(0, 0, img, false);
   }
   else {
      if (first) {
         first = false;

         background_image.loadPPM(background_image_path);
         img.freeImg(background_image.getWidth(), background_image.getHeight(), BLACK);
         img.insertImage(0, 0, background_image, false);
      }
   }

   int var_box_width = var_box_pad_left + var_box_pad_right + char_width * max_length;
   int var_box_height = var_box_pad_top + var_box_pad_bottom + line_height * variables_to_be_painted.size();
   int var_box_left = align == HorizontalAlignment::centered
      ? (img.getWidth() - var_box_width) / 2
      : (align == HorizontalAlignment::left
         ? 10
         : img.getWidth() - var_box_width - 10);

   int overall_width = var_box_width + shadow_offset;
   int overall_height = var_box_height + shadow_offset;

   if (sw_chunk.getHeight() == 0 || sw_chunk.getWidth() == 0) {
      sw_chunk_x = var_box_left;
      sw_chunk_y = var_box_top;
   }

   img.insertImage(sw_chunk_x, sw_chunk_y, sw_chunk, false);
   img.fillRectangle(var_box_left + shadow_offset, var_box_top + shadow_offset, var_box_width, var_box_height, shadow_color, false);
   img.rectangle(var_box_left + shadow_offset, var_box_top + shadow_offset, var_box_width, var_box_height, shadow_frame_color, false);
   img.fillRectangle(var_box_left, var_box_top, var_box_width, var_box_height, rectangle_background_color, false);
   img.rectangle(var_box_left, var_box_top, var_box_width, var_box_height, rectangle_frame_color, false);

   if (sw_chunk.getHeight() != overall_height || sw_chunk.getWidth() != overall_width) {
      sw_chunk = background_image.copyArea(var_box_left, var_box_top, var_box_left + overall_width + 1, var_box_top + overall_height + 1);
      sw_chunk_x = var_box_left;
      sw_chunk_y = var_box_top;
   }

   for (int i = 0; i < var_strs.size(); i++) {
      auto var = var_strs[i].first;
      auto col_fct = var_strs[i].second;

      img.writeAsciiText(
         var_box_left + var_box_pad_left,
         var_box_top + var_box_pad_top + i * line_height,
         var,
         CoordTrans::do_it,
         false,
         col_fct);
   }
};

inline void replayCounterExample(
   const std::string& path_to_counterexample_file_input,
   const std::string& path_to_FSM_INIT_file_input,
   const std::string& path_to_FSM_file_input,
   const std::string& path_to_background_image_input,
   const std::string& path_to_custom_vis_png_output,
   const std::string& path_to_FSM_vis_output,
   const std::string& path_to_FSM_DATA_vis_output,
   const std::function<void(const std::shared_ptr<FSMs>)>& association_function,
   const PainterFunction& painter,
   std::vector<PainterVariableDescription>& variable_descriptions,
   std::vector<PainterButtonDescription>& button_descriptions,
   const std::string& fsm_output_type = "pdf",
   const bool autostart_external_visualization = false) {

   bool first = true;
   Image background_image;
   Image sw_chunk;
   int sw_chunk_x;
   int sw_chunk_y;
   int i = 0;

   if (autostart_external_visualization) {
      if (std::system((std::string("start ") + path_to_custom_vis_png_output).c_str()) == 0) {
         std::cout << "Started image view of counterexample visualization." << std::endl;
      }

      if (std::system((std::string("start ") + path_to_FSM_vis_output + "." + fsm_output_type).c_str()) == 0) {
         std::cout << "Started PDF view of state machine." << std::endl;
      }
   }

   auto ce = StaticHelper::extractMCTracesFromNusmvFile(path_to_counterexample_file_input).at(0);
   auto resolver = std::make_shared<FSMResolverDefaultMaxTransWeight>(2.5); // Resolver is overwritten when loading FSM from file.
   std::shared_ptr<FSMs> m = std::make_shared<FSMs>(resolver, NonDeterminismHandling::note_error_on_non_determinism);

   association_function(m);

   m->loadFromFile(path_to_FSM_INIT_file_input);
   m->printAndThrowErrorsIfAny(true, false);
   m->loadFromFile(path_to_FSM_file_input, false, true);
   m->printAndThrowErrorsIfAny(true, false);

   m->createGraficOfCurrentGraph(path_to_FSM_vis_output, true, fsm_output_type, false, vfm::fsm::GraphvizOutputSelector::graph_only);
   m->createGraficOfCurrentGraph(path_to_FSM_DATA_vis_output, true, fsm_output_type, false, vfm::fsm::GraphvizOutputSelector::data_with_functions);

   m->setStatePainter(painter);
   int loop = -1;

   for (int i = 0; i < ce.size();) {
      m->createGraficOfCurrentGraph(path_to_FSM_vis_output, true, fsm_output_type, false, vfm::fsm::GraphvizOutputSelector::graph_only);
      m->createGraficOfCurrentGraph(path_to_FSM_DATA_vis_output, true, fsm_output_type, false, vfm::fsm::GraphvizOutputSelector::data_with_functions);
      m->createStateVisualizationOfCurrentState(
         path_to_custom_vis_png_output, 
         variable_descriptions, 
         button_descriptions, 
         path_to_background_image_input,
         first,
         background_image,
         sw_chunk,
         sw_chunk_x,
         sw_chunk_y,
         i,
         OutputType::png);
      std::this_thread::sleep_for(std::chrono::milliseconds(100 /*(int)m->evaluateFormula("expectedFctCycleTime")*/));
      m->replayCounterExampleForOneStep(ce, i, loop, true);
   }
}

} // vfm
