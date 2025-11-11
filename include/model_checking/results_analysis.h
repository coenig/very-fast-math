//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "failable.h"
#include "static_helper.h"
#include <map>

namespace vfm::mc {

using vfm::StaticHelper;

class Analyzor : public Failable {
public:
   inline Analyzor() : Failable("Analyzor") {}

   void createTikzPlots(const std::string& csv_path, const std::string& output_path, const bool legend)
   {

      std::map<std::string, std::string> markers{ { "2", "triangle*" }, { "3", "square*" }, { "4", "diamond*" } };
      std::map<std::string, std::string> line_styles{ { "500", "solid" }, { "1000", "dashed" }, { "2000", "dotted" } };
      std::map<std::string, std::string> colors = { { "500", "red" }, { "1000", "blue" }, { "2000", "orange"}};

      std::string tikz_code{ R"(
\begin{tikzpicture}
\begin{loglogaxis}[
    xlabel={$\bar{x}$ $\left(\si{m^{-1}}\right)$},
    ylabel={runtime $\left(\si{s}\right)$},
    legend style={at={(1.05,1)}, anchor=north west},
    xtick={0.125, 0.250, 0.500, 1.000, 2.000, 4.000, 8.000},
    xticklabels={0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0},
]
)" };

      std::map<std::string, std::vector<std::string>> df{ StaticHelper::readCSV(csv_path, ";") };

      if (df.empty()) {
         addError("No data found in '" + csv_path + "'.");
         return;
      }

      const std::set<std::string> necessary_ones{ "t", "d", "nonegos" };

      for (const auto& necessary_one : necessary_ones) {
         if (!df.count(necessary_one)) {
            df.insert({ necessary_one, {} });
            for (int i = 0; i < df.begin()->second.size(); i++) {
               df.at(necessary_one).push_back("1");
            }
         }
      }

      std::set<std::string> nonegos_unique(df.at("nonegos").begin(), df.at("nonegos").end());
      std::set<std::string> t_unique(df.at("t").begin(), df.at("t").end());
      std::set<int> d_unique{};

      for (const auto& d : df.at("d")) {
         if (StaticHelper::isParsableAsInt(d)) { // Avoid empty cells.
            d_unique.insert(std::stoi(d));
         }
      }

      for (const auto& nonegos_value : nonegos_unique) {
         for (const auto& t_value : t_unique) {
            bool valid_runtime{ false };
            std::string tmp{ "\\addplot[mark=" + markers[nonegos_value] + ", color=" + colors[t_value] + ", " + line_styles[t_value] + ", thick] coordinates {" };

            for (auto& d : d_unique) { // Sort by d.
               for (int i = 0; i < df.at("nonegos").size(); i++) {
                  if (df.at("d")[i] == std::to_string(d) && df.at("nonegos")[i] == nonegos_value && df.at("t")[i] == t_value) { // Look at rows with correct nonegos and t only.
                        float scaled_d = std::stof(df.at("d")[i]) * 0.001;
                        float scaled_runtime = std::stof(df.at("runtime_ms")[i]) * 0.001;

                        if (scaled_runtime > 0) {
                           valid_runtime = true;
                           tmp += "(" + round(scaled_d, 3) + "," + round(scaled_runtime, 3) + ") ";
                        }
                  }
               }
            }

            tmp += "};\n";
            if (valid_runtime) {
               tikz_code += tmp;
               if (legend) {
                  tikz_code += "(\\addlegendentry{{t=" + t_value + ", nonegos={" + nonegos_value + "}}}\n)";
               }
            }
         }
      }

      tikz_code += R"(
\end{loglogaxis}
\end{tikzpicture}
)";

      StaticHelper::writeTextToFile(tikz_code, output_path);
   }

private:
   std::string round(const float result, const int decimals)
   {
      const double multiplier = std::pow(10.0, decimals);
      auto rounded = std::round(result * multiplier) / multiplier;
      auto rounded_str = std::to_string(rounded);

      if (rounded_str.find('.') != std::string::npos)
      {
         rounded_str = rounded_str.substr(0, rounded_str.find_last_not_of('0') + 1);
         if (rounded_str.find('.') == rounded_str.size() - 1)
         {
            rounded_str = rounded_str.substr(0, rounded_str.size() - 1);
         }
      }

      return rounded_str;
   }
};
} // vfm::mc