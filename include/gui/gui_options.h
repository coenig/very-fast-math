//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "enum_wrapper.h"
#include "static_helper.h"

#include <map>
#include <string>
#include <filesystem>

namespace vfm {
   enum class SecOptionGlobalItemEnum {
   envmodel_variables_general,
   envmodel_variables_specific,
   planner_variables,
   json_visible,
   no_show_ltl_note,
   invalid
};

enum class SecOptionLocalItemEnum {
   selected_job,
   invalid
};

const std::map<SecOptionGlobalItemEnum, std::string> sec_option_global_enum_map{
   { SecOptionGlobalItemEnum::envmodel_variables_general, "envmodel_variables_general" },
   { SecOptionGlobalItemEnum::envmodel_variables_specific, "envmodel_variables_specific" },
   { SecOptionGlobalItemEnum::planner_variables, "planner_variables" },
   { SecOptionGlobalItemEnum::json_visible, "json_visible" },
   { SecOptionGlobalItemEnum::no_show_ltl_note, "no_show_ltl_note" },
   { SecOptionGlobalItemEnum::invalid, "invalid" },
};

const std::map<SecOptionLocalItemEnum, std::string> sec_option_local_enum_map{
   { SecOptionLocalItemEnum::selected_job, "selected_job" },
   { SecOptionLocalItemEnum::invalid, "invalid" },
};

class SecOptionGlobalItem : public fsm::EnumWrapper<SecOptionGlobalItemEnum>
{
public:
   explicit SecOptionGlobalItem(const SecOptionGlobalItemEnum& enum_val) : EnumWrapper("SecOptionGlobalItem", sec_option_global_enum_map, enum_val) {}
   explicit SecOptionGlobalItem(const int enum_val_as_num) : EnumWrapper("SecOptionGlobalItem", sec_option_global_enum_map, enum_val_as_num) {}
   explicit SecOptionGlobalItem(const std::string& enum_val_as_string) : EnumWrapper("SecOptionGlobalItem", sec_option_global_enum_map, enum_val_as_string) {}
   explicit SecOptionGlobalItem() : EnumWrapper("SecOptionGlobalItem", sec_option_global_enum_map, SecOptionGlobalItemEnum::invalid) {}
};

class SecOptionLocalItem : public fsm::EnumWrapper<SecOptionLocalItemEnum>
{
public:
   explicit SecOptionLocalItem(const SecOptionLocalItemEnum& enum_val) : EnumWrapper("SecOptionLocalItem", sec_option_local_enum_map, enum_val) {}
   explicit SecOptionLocalItem(const int enum_val_as_num) : EnumWrapper("SecOptionLocalItem", sec_option_local_enum_map, enum_val_as_num) {}
   explicit SecOptionLocalItem(const std::string& enum_val_as_string) : EnumWrapper("SecOptionLocalItem", sec_option_local_enum_map, enum_val_as_string) {}
   explicit SecOptionLocalItem() : EnumWrapper("SecOptionLocalItem", sec_option_local_enum_map, SecOptionLocalItemEnum::invalid) {}
};

const std::map<SecOptionGlobalItemEnum, std::string> sec_option_global_default_values{
   { SecOptionGlobalItemEnum::envmodel_variables_general, "" },
   { SecOptionGlobalItemEnum::envmodel_variables_specific, "" },
   { SecOptionGlobalItemEnum::planner_variables, "" },
   { SecOptionGlobalItemEnum::json_visible, "false" },
   { SecOptionGlobalItemEnum::no_show_ltl_note, "#INVALID" },
   { SecOptionGlobalItemEnum::invalid, "#INVALID" },
};

const std::map<SecOptionLocalItemEnum, std::string> sec_option_local_default_values{
   { SecOptionLocalItemEnum::selected_job, "true" },
   { SecOptionLocalItemEnum::invalid, "#INVALID" },
};

class OptionsGlobal // TODO: Double code with "local"
{
public:
   explicit inline OptionsGlobal(const std::string& my_path) : my_path_{ my_path }
   {
      if (!my_path.empty()) {
         if (std::filesystem::exists(my_path_)) {
            loadOptions();
         }

         saveOptions();
      }
      else {
         for (const auto& item : sec_option_global_enum_map) { // Add for missing options a default value.
            options_[item.first] = "#ERROR-OPTION-NOT-AVAILABLE";
         }
      }
   }

   OptionsGlobal(const OptionsGlobal& other) {
      my_path_ = other.my_path_;
      options_ = other.options_;
   }

   OptionsGlobal(OptionsGlobal&& other) noexcept {
      my_path_ = other.my_path_;
      options_ = other.options_;
   }

   inline bool hasOptionChanged(const SecOptionGlobalItemEnum option)
   {
      std::string new_value{ getOptionValue(option) };
      bool res{ !(old_values_.count(option) && new_value == old_values_.at(option)) };
      old_values_[option] = new_value;
      return res;
   }

   inline void loadOptions()
   {
      std::lock_guard<std::mutex> lock{ mutex_ };
      options_.clear();

      for (const auto& item : sec_option_global_enum_map) { // Add for missing options a default value.
         if (sec_option_global_default_values.count(item.first)) {
            options_[item.first] = sec_option_global_default_values.at(item.first);
         }
         else {
            options_[item.first] = "#MISSING-DEFAULT";
         }
      }

      if (std::filesystem::exists(my_path_)) {
         const std::string content{ StaticHelper::readFile(my_path_) };
         const auto options_str{ StaticHelper::split(content, ";") };
         for (const auto& option_str_raw : options_str) {
            const std::string option_str{ StaticHelper::removeWhiteSpace(option_str_raw) };
            auto pair{ StaticHelper::split(option_str, "=") };

            if (pair.size() == 2) {
               options_[SecOptionGlobalItem(pair.at(0)).getEnum()] = pair.at(1);
            }
         }
      }
   }

   inline void saveOptions() {
      std::lock_guard<std::mutex> lock{ mutex_ };

      std::string options_str{};

      for (const auto& option : options_) {
         options_str += SecOptionGlobalItem(option.first).getEnumAsString() + "=" + option.second + ";\n";
      }

      StaticHelper::writeTextToFile(options_str, my_path_);
   }

   inline std::string getOptionValue(const SecOptionGlobalItemEnum option)
   {
      return options_.at(option);
   }

   inline void setOptionValue(const SecOptionGlobalItemEnum option, const std::string& value)
   {
      options_[option] = value;
   }

   inline std::set<std::string> getSetOptionValue(const SecOptionGlobalItemEnum option)
   {
      std::string option_val{ getOptionValue(option) };
      std::set<std::string> set{};

      for (const auto& el : StaticHelper::split(option_val, ",")) {
         set.insert(el);
      }

      return set;
   }

   inline bool containsSetOption(const SecOptionGlobalItemEnum option, const std::string& value)
   {
      return getSetOptionValue(option).count(value);
   }

   inline void addToSetOption(const SecOptionGlobalItemEnum option, const std::string& value)
   {
      auto set = getSetOptionValue(option);
      std::string new_option_val{};

      set.insert(value);

      for (const auto& el : set) {
         new_option_val += el + ",";
      }

      setOptionValue(option, new_option_val);
   }

private:
   std::string my_path_{};
   std::map<SecOptionGlobalItemEnum, std::string> options_{};
   std::mutex mutex_{};
   std::map<SecOptionGlobalItemEnum, std::string> old_values_{};
};

class OptionsLocal // TODO: Double code with "global"
{
public:
   explicit inline OptionsLocal(const std::string& my_path) : my_path_{ my_path }
   {
      if (!my_path.empty()) {
         if (std::filesystem::exists(my_path_)) {
            loadOptions();
         }

         saveOptions();
      }
      else {
         for (const auto& item : sec_option_local_enum_map) { // Add for missing options a default value.
            options_[item.first] = "#ERROR-OPTION-NOT-AVAILABLE";
         }
      }
   }

   OptionsLocal(const OptionsLocal& other) {
      my_path_ = other.my_path_;
      options_ = other.options_;
   }

   OptionsLocal(OptionsLocal&& other) noexcept {
      my_path_ = other.my_path_;
      options_ = other.options_;
   }

   inline bool hasOptionChanged(const SecOptionLocalItemEnum option)
   {
      std::string new_value{ getOptionValue(option) };
      bool res{ !(old_values_.count(option) && new_value == old_values_.at(option)) };
      old_values_[option] = new_value;
      return res;
   }

   inline void loadOptions()
   {
      std::lock_guard<std::mutex> lock{ mutex_ };
      options_.clear();

      for (const auto& item : sec_option_local_enum_map) { // Add for missing options a default value.
         if (sec_option_local_default_values.count(item.first)) {
            options_[item.first] = sec_option_local_default_values.at(item.first);
         }
         else {
            options_[item.first] = "#MISSING-DEFAULT";
         }
      }

      if (std::filesystem::exists(my_path_)) {
         const std::string content{ StaticHelper::readFile(my_path_) };
         const auto options_str{ StaticHelper::split(content, ";") };
         for (const auto& option_str_raw : options_str) {
            const std::string option_str{ StaticHelper::removeWhiteSpace(option_str_raw) };
            auto pair{ StaticHelper::split(option_str, "=") };

            if (pair.size() == 2) {
               options_[SecOptionLocalItem(pair.at(0)).getEnum()] = pair.at(1);
            }
         }
      }
   }

   inline void saveOptions() {
      std::lock_guard<std::mutex> lock{ mutex_ };

      std::string options_str{};

      for (const auto& option : options_) {
         options_str += SecOptionLocalItem(option.first).getEnumAsString() + "=" + option.second + ";\n";
      }

      StaticHelper::writeTextToFile(options_str, my_path_);
   }

   inline std::string getOptionValue(const SecOptionLocalItemEnum option)
   {
      return options_.at(option);
   }

   inline void setOptionValue(const SecOptionLocalItemEnum option, const std::string& value)
   {
      options_[option] = value;
   }

private:
   std::string my_path_{};
   std::map<SecOptionLocalItemEnum, std::string> options_{};
   std::mutex mutex_{};
   std::map<SecOptionLocalItemEnum, std::string> old_values_{};
};
} // vfm
