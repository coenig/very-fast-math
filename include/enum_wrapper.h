//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2019 Robert Bosch GmbH.
//
// The reproduction, distribution and utilization of this file as
// well as the communication of its contents to others without express
// authorization is prohibited. Offenders will be held liable for the
// payment of damages. All rights reserved in the event of the grant
// of a patent, utility model or design.
//============================================================================================================

#pragma once

#ifndef ENUM_WRAPPER_H
#define ENUM_WRAPPER_H

#include "failable.h"
#include <memory>
#include <map>
#include <iostream>
#include <cassert>

namespace vfm {
namespace fsm {

/// Encapsulates enum logic, particularly defines mapping between enum and a numerical value. 
/// Example usage with HmiCommandEnum from PJHP FCT:
/// - Convert from num to enum: HmiCommand(1).getEnum(); // ==> HmiCommandEnum::activate
/// - Convert from enum to num: HmiCommand(HmiCommandEnum::activate).getEnumAsInt(); // ==> 1
///
/// Note: For usage as an external variable to the vfm::fsm framework, the class provides a char 
/// pointer to the numerical value. For technical reasons it's currently not an int 
/// (vfm can handle bool, float and char only, will be extended to int soon). Therefore, only
/// 255 different values per enum are allowed for now (but no other restrictions;
/// 1 value is needed as special case denoter).
template<class E>
class EnumWrapper
{
public:

   static std::string getEnumValueName(const std::string& enumName, const std::string& value) 
   {
      return enumName + "::" + value;
   }

   explicit EnumWrapper(const std::string& enum_name, const std::map<E, std::string>& mapping, const E& enum_val) 
      : EnumWrapper(enum_name, mapping)
   {
       createInternalMappings();
       setValue(enum_val);   
   }

   explicit EnumWrapper(const std::string& enum_name, const std::map<E, std::string>& mapping, const int enum_val_as_num)
      : EnumWrapper(enum_name, mapping)
   {
      assert(main_mapping_.count(enum_val_as_num));

      createInternalMappings();
      setValue(enum_val_as_num);
   }

   explicit EnumWrapper(const std::string& enum_name, const std::map<E, std::string>& mapping, const std::string enum_val_as_string)
      : EnumWrapper(enum_name, mapping)
   {
      bool found = false;

      for (const auto& el : main_mapping_) { // TODO: Watch for efficiency issues in release build.
         if (el.second.second == enum_val_as_string) {
            found = true;
            break;
         }
      }

      if (!found) {
         std::string allowed{};

         for (const auto& el : main_mapping_) {
            allowed += " '" + el.second.second + "'";
         }

         Failable::getSingleton()->addFatalError("Invalid value '" + enum_val_as_string + "' for enum wrapper '" + enum_name + "'. Only these are allowed: " + allowed + ".");
      }

      createInternalMappings();
      setValue(enum_val_as_string);
   }

   explicit EnumWrapper(const EnumWrapper<E>& other) : 
      my_enum_(std::make_shared<E>(*other.my_enum_)),
      my_enum_as_char_(other.my_enum_as_char_),
      my_enum_as_char_ptr_(&my_enum_as_char_),
      enum_name_(other.enum_name_),
      main_mapping_(other.main_mapping_),
      mapping_to_enums_(other.mapping_to_enums_),
      mapping_to_names_(other.mapping_to_names_)
   {
   }

   ~EnumWrapper() = default;

   EnumWrapper<E>& operator=(const EnumWrapper<E>& other)
   {
      if (this != &other) {
         mapping_to_enums_ = other.mapping_to_enums_;
         mapping_to_names_ = other.mapping_to_names_;
         my_enum_ = std::make_shared<E>(*other.my_enum_);
         my_enum_as_char_ = other.my_enum_as_char_;
         my_enum_as_char_ptr_ = &my_enum_as_char_;
      }

      return *this;
   }

   E getEnum() const
   {
       return *my_enum_;
   }

   int getEnumAsInt() const
   {
      return my_enum_as_char_;
   }

   std::string getEnumAsString() const
   {
      return getNameMapping().at(getEnumAsInt());
   }

   char* getPtrToNumValue() const
   {
      return my_enum_as_char_ptr_;
   }

   void setValue(const E& enum_val)
   {
       my_enum_ = std::make_shared<E>(enum_val);
       my_enum_as_char_ = static_cast<char>(convertEnumToInt(*my_enum_));
   }

   void setValue(const int enum_val_as_num)
   {
       setValue(convertIntToEnum(enum_val_as_num));
   }

   void setValue(const EnumWrapper<E>& enum_wrap)
   {
       setValue(enum_wrap.getEnum());
   }

   void setValue(const std::string& enum_val_as_string)
   {
      for (const auto& pair : getNameMapping()) {
         if (pair.second == enum_val_as_string) {
            setValue(pair.first);
            return;
         }
      }

      std::cerr << std::string("Enum '" + getEnumName() + "' has no value named '" + enum_val_as_string + "'. No value set.") << std::endl;
   }

   std::string getEnumName() const
   {
       return enum_name_;
   }

   std::map<int, E> getEnumMapping() const
   {
       return mapping_to_enums_;
   }
    
   std::map<int, std::string> getNameMapping() const
   {
      return mapping_to_names_;
   }

   std::string getFSMNameOfEnumValue(const E& enum_val)
   {
      std::string val_name = mapping_to_names_.at(convertEnumToInt(enum_val));
      return getEnumValueName(enum_name_, val_name);
   }

   int getEnumSize() const
   {
       return main_mapping_.size();
   }

private:
   EnumWrapper(const std::string& enum_name, const std::map<E, std::string>& mapping) 
      : enum_name_(enum_name) {
      for (const auto& el : mapping) {
         main_mapping_[static_cast<int>(el.first)] = { el.first, el.second };
      }
   }

   std::shared_ptr<E> my_enum_ = nullptr;
   char my_enum_as_char_ = -1;
   char* my_enum_as_char_ptr_ = &my_enum_as_char_;
   const std::string enum_name_;
   std::map<int, std::pair<E, std::string>> main_mapping_;
   std::map<int, E> mapping_to_enums_ = {};
   std::map<int, std::string> mapping_to_names_ = {};

   void createInternalMappings()
   {
       for (const auto& pair : main_mapping_) {
           mapping_to_enums_[pair.first] = pair.second.first;
           mapping_to_names_[pair.first] = pair.second.second;
       }
   }

   E convertIntToEnum(const int enum_val_as_num)
   {
       return mapping_to_enums_.at(enum_val_as_num);
   }

   int convertEnumToInt(const E& enum_val)
   {
       for (const auto& entry : mapping_to_enums_)
       {
           if (entry.second == enum_val) {
               return entry.first;
           }
       }

       return -1; // This cannot happen.
   }
};

template<class T>
bool operator==(const EnumWrapper<T>& lhs, const int rhs)
{
   return lhs.getEnumAsInt() == rhs;
}

template<class T>
bool operator==(const int lhs, const EnumWrapper<T>& rhs)
{
   return lhs == rhs.getEnumAsInt();
}

template<class T>
bool operator==(const EnumWrapper<T>& lhs, const T& rhs)
{
   return lhs.getEnum() == rhs;
}

template<class T>
bool operator==(const T& lhs, const EnumWrapper<T>& rhs)
{
   return lhs == rhs.getEnum();
}

template<class T>
bool operator!=(const EnumWrapper<T>& lhs, const int rhs)
{
   return lhs.getEnumAsInt() != rhs;
}

template<class T>
bool operator!=(const int lhs, const EnumWrapper<T>& rhs)
{
   return lhs != rhs.getEnumAsInt();
}

template<class T>
bool operator!=(const EnumWrapper<T>& lhs, const T& rhs)
{
   return lhs.getEnum() != rhs;
}

template<class T>
bool operator!=(const T& lhs, const EnumWrapper<T>& rhs)
{
   return lhs != rhs.getEnum();
}

} // fsm
} // vfm

#endif
