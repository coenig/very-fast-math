//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "data_pack.h"
#include "math_struct.h"
#include "term.h"
#include "term_compound.h"
#include "static_helper.h"
#include "dat_src_arr.h"
#include "dat_src_arr_as_readonly_string.h"
#include "dat_src_arr_as_random_access_file.h"
#include "dat_src_arr_as_readonly_random_access_file.h"
#include "dat_src_arr_as_string.h"
#include "dat_src_arr_as_float_vector.h"
#include "vfmacro/script.h"
#include <sstream>
#include <algorithm>
#include <cassert>
#include <sys/stat.h>
#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <cctype>
#include <cmath>

using namespace vfm;

DataPack::DataPack() : Parsable("DataPack")
{
   reset();
}

void DataPack::setSingleVal(const int var_reference, float val, const bool hidden)
{
   auto it = addresses_to_names_.find(var_reference);

   if (it != addresses_to_names_.end() && !isArrayDeclared(it->second)) {
      addOrSetSingleVal(it->second, val, hidden);
   }
   else if (it == addresses_to_names_.end()) {
      setHeapLocation(var_reference, val);
   } else {
      // TODO (AI)
   }
}

bool vfm::DataPack::isStaticArray(const int arr_reference) const
{
   auto it = addresses_to_names_.find(arr_reference);
   return it != addresses_to_names_.end() && isArrayDeclared(it->second);
}

float DataPack::getArrayVal(const std::string& arr_name, const int index) const
{
   return getArrayVal(vfm_memory_.at(names_to_addresses_.at(arr_name)), index);
}

float DataPack::getArrayVal(const int arr_reference, const int index) const
{
   auto it = addresses_to_names_.find(arr_reference);

   if (it != addresses_to_names_.end() && isArrayDeclared(it->second)) { // Old-school static array.
      return getStaticArray(arr_reference).get(index);
   }
   else { // New C-style dynamic array.
      const int address = arr_reference + index;

      if (checkHeapAddress(address)) {
         //pedanticCheckHeapAddress(address);
         return getHeapLocation(address);
      }
      else {
         addFatalError("Tried to access element " + std::to_string(index) + " of array '" + it->second + "' which is out of memory.");
         return 0;
      }
   }
}

void DataPack::setArrayVal(const std::string& arr_name, const int index, const float val)
{
   setArrayVal(vfm_memory_.at(names_to_addresses_.at(arr_name)), index, val);
}

void DataPack::setArrayVal(const int arr_reference, const int index, const float val)
{
   auto it = addresses_to_names_.find(arr_reference);

   if (it != addresses_to_names_.end() && isArrayDeclared(it->second)) { // Old-school static array.
      addArrayAndOrSetArrayVal(it->second, index, val);
   }
   else { // New C-style dynamic array.
      int address = arr_reference + index;

      if (checkHeapAddress(address)) {
         pedanticCheckHeapAddress(address);
         setHeapLocation(address, val);
      }
      else {
         addFatalError("Tried to access element " + std::to_string(index) + " of array '" + it->second + "' which is out of memory.");
      }
   }
}

bool DataPack::pedanticCheckHeapAddress(const int address) const
{
   if (!checkHeapAddress(address)) {
      return false;
   }

   bool ok = true;

   if (address >= getUsedHeapSize()) {
      addWarning("Accessed un-initialized heap location " + std::to_string(address) + " > " + std::to_string(getUsedHeapSize() - 1) + ".");
      ok = false;
   }

   if (addresses_to_names_.find(address) != addresses_to_names_.end()) {
      addWarning("Direct access to heap location that is already reserved by variable '" + addresses_to_names_.at(address) + "'.");
      ok = false;
   }

   return ok;
}

bool DataPack::checkHeapAddress(const int address) const
{
   if (address >= VFM_HEAP_SIZE) {
      addFatalError(std::string("vfm heap size exceeded (max set to ") + std::to_string(VFM_HEAP_SIZE) + "). Access tried at " + std::to_string(address) + ".");
      return false;
   }

   return true;
}

void vfm::DataPack::addConstVariable(const std::string& name, float val, const bool hidden)
{
   addOrSetSingleVal(name, val, hidden, true);
}

void DataPack::addOrSetSingleVal(const std::string& name_orig, float val, const bool hidden, const bool is_const)
{
   std::string name = StaticHelper::privateVarNameForRecursiveFunctions(name_orig, getVarRecursiveID(name_orig));

   // First handle external values to allow reassociating non-external values to external ones.
   bool* ext_bool = getAddressOfExternalBool(name);
   char* ext_char = getAddressOfExternalChar(name);
   float* ext_float = getAddressOfExternalFloat(name);
   if (ext_bool) {
      *ext_bool = val;
      return;
   }
   if (ext_char) {
      *ext_char = val;
      return;
   }
   if (ext_float) {
      *ext_float = val;
      return;
   }

   internal_variables_.insert(name);
   auto address_it = names_to_addresses_.find(name);
   int address = first_free_heap_index_;

   if (address_it != names_to_addresses_.end()) { // Already there.
      address = address_it->second;
   }

   if (!checkHeapAddress(address)) {
      addFatalError("Could not store variable '" + name + "' in memory.");
   }

   vfm_memory_.at(address) = val;
   registerAddress(name);

   if (hidden) {
      hidden_vars_.insert(name);
   }

   if (is_const) {
      const_variables_.insert(name);
   }
}

void DataPack::addOrSetSingleVals(const std::vector<std::string>& names, float val, const bool hidden, const bool is_const)
{
   for (const auto& name : names) {
      addOrSetSingleVal(name, val, hidden, is_const);
   }
}

void vfm::DataPack::declareVariable(const std::string& var_name, const bool is_const, const bool is_hidden)
{
   if (!isDeclared(var_name)) {
      addOrSetSingleVal(var_name, 0, is_hidden, is_const);
   }
}

float DataPack::getSingleVal(const std::string& name_orig) const
{
   std::string var_name = StaticHelper::privateVarNameForRecursiveFunctions(name_orig, getVarRecursiveID(name_orig));
   
   // First handle external values to allow reassociating non-external values to external ones.
   bool* ext_bool = getAddressOfExternalBool(var_name);
   char* ext_char = getAddressOfExternalChar(var_name);
   float* ext_float = getAddressOfExternalFloat(var_name);
   if (ext_bool) {
      return *ext_bool;
   }
   if (ext_char) {
      return *ext_char;
   }
   if (ext_float) {
      return *ext_float;
   }

   if (!names_to_addresses_.count(var_name)) {
      // TODO: Reference and dereference should become operators in future. For lack of time we do it in this static way for now.
      if (StaticHelper::stringStartsWith(var_name, SYMB_REF)) {
         std::string raw_name = var_name.substr(SYMB_REF.size());
         
         if (!isDeclared(raw_name)) {
            if (raw_name.size() > 0 && raw_name[0] == '.') {
               (const_cast<DataPack*>(this))->addArrayAndOrSetArrayVal(raw_name, -1, 0);
            } else {
               (const_cast<DataPack*>(this))->addOrSetSingleVal(raw_name, 0);
            }
         }
         
         return names_to_addresses_.at(raw_name);
      }

      return 0;
   }

   return vfm_memory_.at(names_to_addresses_.at(var_name));
}

void vfm::DataPack::addSingleValIfUndeclared(const std::string& name_orig, const bool hidden, const float init_val)
{
   std::string name = StaticHelper::privateVarNameForRecursiveFunctions(name_orig, getVarRecursiveID(name_orig));

   if (!isVarDeclared(name)) {
      if (isArrayDeclared(name)) {
         std::cerr << "Requested new variable name '" << name << "' already declared as array. Request rejected." << std::endl;
      }
      else {
         addOrSetSingleVal(name, init_val, hidden);
      }
   }
}

void vfm::DataPack::addSingleValsIfUndeclared(const std::vector<std::string>& names, const bool hidden, const float init_val)
{
   for (const auto& name : names) {
      addSingleValIfUndeclared(name, hidden, init_val);
   }
}

bool vfm::DataPack::isInternalValue(const std::string& name_orig) const
{
   std::string name = StaticHelper::privateVarNameForRecursiveFunctions(name_orig, getVarRecursiveID(name_orig));
   return std::find(internal_variables_.begin(), internal_variables_.end(), name) != internal_variables_.end();
}

bool DataPack::isExternalAssociation(const int heap_address) const
{
#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   for (const auto& pair : association_functions_) {
      for (const auto& f : pair.second) {
         auto address = f(heap_address, this->vfm_memory_, "", names_to_addresses_, addresses_to_names_, static_external_bool_addresses_, static_external_char_addresses_, static_external_float_addresses_);
         if (address) {
            return true;
         }
      }
   }
#endif

   return false;
}

bool vfm::DataPack::isExternalValue(const std::string& name) const
{
   return isExternalBool(name)
         || isExternalChar(name)
         || isExternalFloat(name);
}

bool DataPack::isExternalBool(const std::string& name) const
{
   return getAddressOfExternalBool(name);
}

bool DataPack::isExternalChar(const std::string& name) const
{
   return getAddressOfExternalChar(name);
}

bool DataPack::isExternalFloat(const std::string& name) const
{
   return getAddressOfExternalFloat(name);
}

bool vfm::DataPack::isVarDeclared(const std::string& name) const
{
   return isInternalValue(name) || isExternalValue(name);
}

bool vfm::DataPack::isArrayDeclared(const std::string& name) const
{
   return arrays_.count(name);
}

bool vfm::DataPack::isDeclared(const std::string& name) const
{
   return isVarDeclared(name) || isArrayDeclared(name);
}

void vfm::DataPack::setHeapLocation(const int reference, const float val)
{
   checkHeapAddress(reference);

#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   for (const auto& pair : association_functions_) {
      for (const auto& f : pair.second) {
         auto address = f(reference, this->vfm_memory_, "", names_to_addresses_, addresses_to_names_, static_external_bool_addresses_, static_external_char_addresses_, static_external_float_addresses_);

         if (address) {
            switch (pair.first)
            {
            case AssociationType::Bool:
               *(bool*) address = val;
               break;
            case AssociationType::Char:
               *(char*) address = val;
               break;
            default:
               *(float*) address = val;
               break;
            }
         }
      }
   }
#endif

   vfm_memory_.at(reference) = val;

   highest_defined_heap_index_above_free_ = (std::max)(highest_defined_heap_index_above_free_, reference);
}

float vfm::DataPack::getHeapLocation(const int reference) const
{
   checkHeapAddress(reference);

#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   for (const auto& pair : association_functions_) {
      for (const auto& f : pair.second) {
         auto address = f(reference, this->vfm_memory_, "", names_to_addresses_, addresses_to_names_, static_external_bool_addresses_, static_external_char_addresses_, static_external_float_addresses_);

         if (address) {
            switch (pair.first)
            {
            case AssociationType::Bool:
               return *(bool*) address;
               break;
            case AssociationType::Char:
               return *(char*) address;
               break;
            default:
               return *(float*) address;
               break;
            }
         }
      }
   }
#endif

   // TODO: Check if in "used" heap range?
   return vfm_memory_.at(reference);
}

void DataPack::addArrayAndOrSetArrayVal(const std::string& name, const int& index, const float& val, const ArrayMode mode_if_new)
{
   if (arrays_.find(name) == arrays_.end()) {
      if (mode_if_new == ArrayMode::chars_random_access_file 
          || mode_if_new == ArrayMode::chars_read_only_file
          || mode_if_new == ArrayMode::implicit && name[name.length() - 1] == FILE_ARRAY_DENOTER_SUFFIX) {
         std::string file_name = /*starter::pars.in_path*/ + "./" + name + "." + "arr";

         if (mode_if_new == ArrayMode::chars_random_access_file
             || mode_if_new == ArrayMode::implicit && name.length() > 1 && name[name.length() - 2] == RANDOM_ACCESS_FILE_ARRAY_DENOTER_SUFFIX) {
            setArrayViaRawSource(name, std::make_shared<DataSrcArrayAsRandomAccessFile>(file_name, false));
         }
         else {
            struct stat buffer;
            if (stat((file_name).c_str(), &buffer) == 0) {
               setArrayViaRawSource(name, std::make_shared<DataSrcArrayAsReadonlyRandomAccessFile>(file_name));
            } else {
               std::string array_val;
               std::cout << "Please enter an input string for array '" << name << "' (or provide a file named '" << file_name << "'): ";
               std::getline(std::cin, array_val);
               setArrayViaRawSource(name, std::make_shared<DataSrcArrayAsReadonlyString>(array_val));
            }
         }
      }
      else if (mode_if_new == ArrayMode::floats
         || mode_if_new == ArrayMode::implicit && StaticHelper::stringEndsWith(name, FLOAT_ARRAY_DENOTER_SUFFIX)) {
            addStringAsFloatVector(name, (std::max)(index, INITIAL_ARRAY_CAPACITY));
        }
      else {
         addStringAsArray(name, (std::max)(index, INITIAL_ARRAY_CAPACITY));
      }
   }

   if (index >= 0) {
      setArrayAt(name, index, val);
   }
}

void DataPack::registerAddress(const std::string& name, const bool var)
{
   if (!names_to_addresses_.count(name)) {
      int& count = var ? first_free_heap_index_ : first_free_array_index_;
      names_to_addresses_.insert({ name, count });
      addresses_to_names_.insert({ count, name });
      ++count;
   }
}

void DataPack::registerArray(const std::string& name, const std::shared_ptr<DataSrcArray>& dt) 
{
   if (name.empty() || name[0] != '.') {
      std::cerr << "Array '" << name << "' does not start with a dot as required for classic arrays." << std::endl;
   }

   registerAddress(name, false);
   arrays_[name] = dt;
}

void DataPack::addReadonlyStringAsArray(const std::string& name, const std::string& val)
{
   registerArray(name, std::make_shared<DataSrcArrayAsReadonlyString>(val));
}

void DataPack::addStringAsFloatVector(const std::string& name, const int& capacity)
{
   registerArray(name, std::make_shared<DataSrcArrayAsFloatVector>((std::max)(capacity, 0)));
}

void DataPack::addStringAsFloatVector(const std::string& name, const std::vector<float>& raw_data)
{
   registerArray(name, std::make_shared<DataSrcArrayAsFloatVector>(raw_data));
}

void DataPack::addStringAsArray(const std::string& name, const int& capacity)
{
   registerArray(name, std::make_shared<DataSrcArrayAsString>((std::max)(capacity, 0)));
}

void DataPack::addStringAsArray(const std::string& name, const std::string& val)
{
   registerArray(name, std::make_shared<DataSrcArrayAsString>(val));
}

void DataPack::setArrayAt(const std::string & arr_name, const int & index, const float& val) const
{
   if (index >= 0) this->arrays_.at(arr_name)->set(index, val);
}

float DataPack::getValFromArray(const std::string& arr_name, const int& index) const
{
   if (!arrays_.count(arr_name)) {
      (const_cast<DataPack*>(this))->addArrayAndOrSetArrayVal(arr_name, -1, 0); // TODO: AVOID THIS!!
   }

   if (index < 0 /*|| !arrays_.count(arr_name)*/ || arrays_.at(arr_name)->size() <= index) {
      return 0;
   }

   return this->arrays_.at(arr_name)->get(index);
}

int vfm::DataPack::getArraySize(const std::string& arr_name) const
{
   return arrays_.at(arr_name)->size();
}

float* DataPack::getAddressOfSingleVal(const std::string& name) const
{
   std::string var_name;

   if (StaticHelper::stringStartsWith(name, SYMB_REF)) {
      var_name = name.substr(SYMB_REF.length());
   }
   else {
      var_name = name;
   }

   if (getAddressOfExternalBool(var_name)) return nullptr; // nullptr for external non-float variables.
   if (getAddressOfExternalChar(var_name)) return nullptr; // nullptr for external non-float variables.
   float* float_address = getAddressOfExternalFloat(var_name);
   if (float_address) return float_address; // Return direct address for external float variables.

   std::string new_name = StaticHelper::privateVarNameForRecursiveFunctions(var_name, getVarRecursiveID(var_name));

   auto it = names_to_addresses_.find(new_name);

   if (it == names_to_addresses_.end()) {
      const_cast<DataPack*>(this)->addOrSetSingleVal(var_name, 0);
   }

   return (float*) &vfm_memory_.at(names_to_addresses_.at(new_name));
}

#define DEFAULT_EXTERNAL_ADDRESS_METHOD_BODY(TYPE, ASSOCIATION_TYPE) \
auto it = names_to_addresses_.find(name);\
if (it != names_to_addresses_.end()) {\
   for (const auto& f : association_functions_.at(AssociationType::ASSOCIATION_TYPE)) {\
      void* address = f(it->second, this->vfm_memory_, name, names_to_addresses_, addresses_to_names_, static_external_bool_addresses_, static_external_char_addresses_, static_external_float_addresses_); \
      if (address) return (TYPE*) address;\
   }\
} /// This is just the body of the address retrieval methods.

bool* DataPack::getAddressOfExternalBool(const std::string name) const
{
#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   DEFAULT_EXTERNAL_ADDRESS_METHOD_BODY(bool, Bool);
#endif

   return nullptr;  // nullptr for everything BUT external bool variables.
}

char* DataPack::getAddressOfExternalChar(const std::string name) const
{
#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   DEFAULT_EXTERNAL_ADDRESS_METHOD_BODY(char, Char);
#endif

   return nullptr;  // nullptr for everything BUT external char variables.
}

float* DataPack::getAddressOfExternalFloat(const std::string name) const
{
#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   DEFAULT_EXTERNAL_ADDRESS_METHOD_BODY(float, Float);
#endif

   return nullptr;  // nullptr for everything BUT external float variables.
}

const float* DataPack::getAddressOfFloatArray(const std::string & arr_name) const
{
   return arrays_.at(arr_name)->getAddressOfFloatArray(); /// Can be nullptr for char arrays_.
}

void vfm::DataPack::setVarsRecursiveIDs(const std::vector<std::string>& var_names, const int recursive_level)
{
   for (const auto& var_name : var_names) {
      private_vars_recursive_levels_[var_name] = recursive_level;
   }
}

int vfm::DataPack::getVarRecursiveID(const std::string& var_name) const
{
   int idx = StaticHelper::stringStartsWith(var_name, SYMB_REF) ? SYMB_REF.size() : 0; // TODO: Remove once reference operator is implemented.

   auto it = private_vars_recursive_levels_.find(var_name.substr(idx));
   if (it == private_vars_recursive_levels_.end()) {
      return 0;
   }
   else {
      return it->second;
   }
}

void vfm::DataPack::setArrayValuesByCopyingRawSource(
   const DataSrcArray& raw_src, const std::string& to_arr_name, const int begin_pos)
{
   for (int i = 0; i < raw_src.size(); i++) {
      addArrayAndOrSetArrayVal(to_arr_name, i + begin_pos, raw_src.get(i));
   }
}

void vfm::DataPack::copyArrayValues(
   const std::string& from_arr_name, 
   const int range_begin_pos_inc, 
   const int range_end_pos_exc, 
   const std::string& to_arr_name, 
   const int to_begin_pos)
{
   int j = to_begin_pos;
   for (int i = range_begin_pos_inc; i < range_end_pos_exc; ++i) {
      addArrayAndOrSetArrayVal(to_arr_name, j, getValFromArray(from_arr_name, i));
      j++;
   }
}

const char* DataPack::getAddressOfArray(const std::string& arr_name) const
{
   return arrays_.at(arr_name)->getAddressOfArray(); /// Can be nullptr for float arrays_.
}

auto dummy_arr = DataSrcArrayAsString(0); // TODO (AI): Probably not the best way to do this.

DataSrcArray& DataPack::getStaticArray(const int arr_num) const
{
   auto res = addresses_to_names_.find(arr_num);

   if (res != addresses_to_names_.end()) {
      auto res2 = arrays_.find(res->second);

      if (res2 != arrays_.end()) {
         return *res2->second;
      }
   }

   return dummy_arr;
}

DataSrcArray& DataPack::getStaticArray(const std::string& arr_name) const
{
   auto res = arrays_.find(arr_name);

   if (res == arrays_.end()) {
      return dummy_arr;
   }
   else {
      return *res->second;
   }
}

std::set<std::string> DataPack::getAllSingleValVarNames(const bool include_hidden) const
{
   std::set<std::string> vec{};

   for (const auto& var : internal_variables_) {
      if (include_hidden || !isHidden(var)) vec.insert(var);
   }

   for (const auto& pair : static_external_bool_addresses_) {
      if (include_hidden || !isHidden(pair.first)) vec.insert(pair.first);
   }

   for (const auto& pair : static_external_char_addresses_) {
      if (include_hidden || !isHidden(pair.first)) vec.insert(pair.first);
   }

   for (const auto& pair : static_external_float_addresses_) {
      if (include_hidden || !isHidden(pair.first)) vec.insert(pair.first);
   }

   return vec;
}

bool vfm::DataPack::parseProgram(const std::string& data_commands)
{
   const auto commands = StaticHelper::split(StaticHelper::removeSingleLineComments(data_commands), PROGRAM_COMMAND_SEPARATOR);
   bool error_in_this_call = false;

   for (const auto& command : commands) {
      std::string cmd = command;
      StaticHelper::trim(cmd);

      if (!StaticHelper::isEmptyExceptWhiteSpaces(command)) {
         if (StaticHelper::stringStartsWith(cmd, PROGRAM_ENUM_DENOTER + " ")) {
            std::vector<std::string> remaining = StaticHelper::split(cmd.substr(PROGRAM_ENUM_DENOTER.size()), PROGRAM_DEFINITION_SEPARATOR);
            std::string name = remaining[0];
            std::string value = remaining[1];
            StaticHelper::trim(name);
            StaticHelper::trim(value);

            if (value[0] == PROGRAM_OPENING_ENUM_BRACKET) { // enum Test := {v = 1, w = 2}
               std::map<int, std::string> mapping;

               for (const auto& val_num : StaticHelper::split(value.substr(1, value.size() - 2), FSM_PROGRAM_COMMAND_SEPARATOR)) {
                  if (!StaticHelper::isEmptyExceptWhiteSpaces(val_num)) {
                     std::vector<std::string> pair = StaticHelper::split(val_num, PROGRAM_ARGUMENT_ASSIGNMENT);
                     std::string val = pair[0];
                     std::string num = pair[1];
                     StaticHelper::trim(val);
                     StaticHelper::trim(num);

                     mapping.insert({ std::stoi(num), val });
                  }
               }

               addEnumMapping(name, mapping);
            }
            else { // enum test := Test
               associateVarToEnum(name, value);
            }
         } else {
            std::shared_ptr<DataPack> d = std::make_shared<DataPack>();
            d->initializeValuesBy(*this);
            const auto pair = StaticHelper::split(command, PROGRAM_DEFINITION_SEPARATOR);

            if (pair.size() != 2) {
               std::string copy = command;
               StaticHelper::trim(copy);

               addError("Malformed command: " + copy);
               return true;
            }

            std::shared_ptr<Term> formula = MathStruct::parseMathStruct(pair[1], true, false, parser_for_program_parsing_)->toTermIfApplicable();

            error_in_this_call = error_in_this_call || formula->serialize() == PARSING_ERROR_STRING;

            float value = formula->eval(d);
            std::string complete_name = StaticHelper::removeWhiteSpace(pair[0]);
            int index_of_array_op_bracket = complete_name.find(PROGRAM_OPENING_ARRAY_BRACKET);

            if (index_of_array_op_bracket < complete_name.size()) {
               int index_of_array_cl_bracket = complete_name.find(PROGRAM_CLOSING_ARRAY_BRACKET);
               std::string array_name = complete_name.substr(0, index_of_array_op_bracket);
               std::string ind_formula_str = complete_name.substr(index_of_array_op_bracket + 1, index_of_array_cl_bracket - index_of_array_op_bracket - 1);
               std::shared_ptr<Term> index_formula = MathStruct::parseMathStruct(ind_formula_str, true, false, parser_for_program_parsing_)->toTermIfApplicable();

               error_in_this_call = error_in_this_call || index_formula->serialize() == PARSING_ERROR_STRING;

               float index = index_formula->eval(d);

               addArrayAndOrSetArrayVal(array_name, index, value);
            }
            else {
               addOrSetSingleVal(complete_name, value);
            }
         }
      }
   }

   return !error_in_this_call;
}

void vfm::DataPack::printMetaStack()
{
   std::cout << std::endl << "STACK" << std::endl;
   for (const auto& terms : stack_terms_for_meta_) {
      for (const auto& term : *terms) {
         std::cout << *term << " {" << term->visited_stack_item_ << "}; ";
      }
      std::cout << std::endl;
   }

   std::cout << std::endl;
}

std::shared_ptr<std::vector<std::shared_ptr<Term>>> DataPack::getValuesForMeta() const
{
   return stack_terms_for_meta_.back();
}

void DataPack::popValuesForMeta() const
{
   stack_terms_for_meta_.pop_back();
}

void vfm::DataPack::pushValuesForMeta(const std::shared_ptr<std::vector<std::shared_ptr<Term>>>& values) const
{
   stack_terms_for_meta_.push_back(values);
}

void DataPack::setArrayViaRawSource(const std::string& arr_name, const std::shared_ptr<DataSrcArray>& data_source)
{
   registerArray(arr_name, data_source);
   arrays_[arr_name] = data_source;
}

void DataPack::resetPrivateStuff()
{
   private_vars_recursive_levels_.clear();
   std::set<std::string> to_remove{};

   for (const auto& var : internal_variables_) {
      if (StaticHelper::isRenamedPrivateVar(var)) {
         to_remove.insert(var);
      }
   }

   for (const auto& var : to_remove) {
      internal_variables_.erase(var);
      hidden_vars_.erase(var);
   }

   stack_terms_for_meta_.clear();
}

void DataPack::reset()
{
   const_variables_.clear();
   private_vars_recursive_levels_.clear();
   highest_defined_heap_index_above_free_ = -1;
   first_free_heap_index_ = 0;
   first_free_array_index_ = VFM_HEAP_SIZE;
   arrays_.clear();
   names_to_addresses_.clear();
   addresses_to_names_.clear();
   internal_variables_.clear();
   static_external_bool_addresses_.clear();
   static_external_char_addresses_.clear();
   static_external_float_addresses_.clear();
   stack_terms_for_meta_.clear();
   enums_.clear();
   enum_variables_.clear();
   hidden_vars_.clear();

   if (vfm_memory_.size() != VFM_HEAP_SIZE) {
      precalculated_external_addresses_.resize(VFM_HEAP_SIZE);
      vfm_memory_.resize(VFM_HEAP_SIZE);

      for (int i = 0; i < VFM_HEAP_SIZE; i++) {
         vfm_memory_.at(i) = VFM_HEAP_EMPTY_VALUE;
      }
   }

   declareVariable("true", true, false);
   declareVariable("false", true, false);
   declareVariable("strin__", false, true);
   declareVariable("ncInt__", false, true);

#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
#define DEFAULT_ASSOCIATION_FUNCTION(EXTERNAL_ADDRESSES) \
[](\
      const int heap_address, \
      const std::vector<float>& f,\
      const std::string& optional_var_name,\
      const std::map<const std::string, int>& names_to_addresses, \
      const std::map<const int, std::string>& addresses_to_names,\
      const std::map<const std::string, bool*>& external_bool_addresses,\
      const std::map<const std::string, char*>& external_char_addresses,\
      const std::map<const std::string, float*>& external_float_addresses) -> void* {\
   std::string var_name; \
   if (optional_var_name.empty()) { \
      auto it = addresses_to_names.find(heap_address); \
      if (it != addresses_to_names.end()) var_name = it->second; \
      else return nullptr; \
   } else var_name = optional_var_name; \
   const auto ext_address_it = EXTERNAL_ADDRESSES.find(var_name); \
   if (ext_address_it != EXTERNAL_ADDRESSES.end()) return ext_address_it->second; \
   else return nullptr;\
} /// This function returns the internally stored external variables.

   association_functions_.clear();
   association_functions_.insert({ AssociationType::Bool, {} });
   association_functions_.insert({ AssociationType::Float, {} });
   association_functions_.insert({ AssociationType::Char, {} });

   addAssociationFunctionToExternalValue(DEFAULT_ASSOCIATION_FUNCTION(external_float_addresses), AssociationType::Float);
   addAssociationFunctionToExternalValue(DEFAULT_ASSOCIATION_FUNCTION(external_char_addresses), AssociationType::Char);
   addAssociationFunctionToExternalValue(DEFAULT_ASSOCIATION_FUNCTION(external_bool_addresses), AssociationType::Bool);
#endif

   script_data_.reset();

   addOrSetSingleVal("true", 1);
}

void DataPack::initializeValuesBy(const DataPack& other)
{
   reset();

   current_recursion_var_depth_id_ = other.current_recursion_var_depth_id_;
   highest_defined_heap_index_above_free_ = other.highest_defined_heap_index_above_free_;
   first_free_heap_index_ = other.first_free_heap_index_;
   first_free_array_index_ = other.first_free_array_index_;

   stack_terms_for_meta_.clear();
   for (const auto& stm : other.stack_terms_for_meta_) {
      stack_terms_for_meta_.push_back(stm);
   }

   for (int i = 0; i < other.getUsedHeapSize(); i++) {
      vfm_memory_.at(i) = other.vfm_memory_.at(i);
   }

   for (const auto& v : other.internal_variables_) {
      internal_variables_.insert(v);
   }

   for (const auto& n : other.names_to_addresses_) {
      names_to_addresses_[n.first] = n.second;
   }

   for (const auto& n : other.addresses_to_names_) {
      addresses_to_names_[n.first] = n.second;
   }

   for (const auto& h : other.hidden_vars_) {
      hidden_vars_.insert(h);
   }

   for (const auto& c : other.const_variables_) {
      const_variables_.insert(c);
   }

   for (const auto& arr : other.arrays_) {
      for (int i = 0; i < arr.second->size(); ++i) {
         addArrayAndOrSetArrayVal(arr.first, i, arr.second->get(i));
      }
   }

   for (const auto& pair : other.enums_) {
      addEnumMapping(pair.first, pair.second);
   }

   for (const auto& pair : other.enum_variables_) {
      associateVarToEnum(pair.first, pair.second);
   }

   for (const auto& v_pair : other.private_vars_recursive_levels_) {
      setVarsRecursiveIDs({ v_pair.first }, v_pair.second);
   }

#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   for (const auto& ext : other.static_external_bool_addresses_) {
      associateSingleValWithExternalBool(ext.first, ext.second);
   }

   for (const auto& ext : other.static_external_float_addresses_) {
      associateSingleValWithExternalFloat(ext.first, ext.second);
   }

   for (const auto& ext : other.static_external_char_addresses_) {
      associateSingleValWithExternalChar(ext.first, ext.second);
   }

   for (const auto& pair : other.association_functions_) {
      for (const auto& f : pair.second) {
         association_functions_.at(pair.first).push_back(f);
      }
   }
#endif

   script_data_ = other.script_data_;
}

void vfm::DataPack::initializeValuesBy(const std::shared_ptr<DataPack> other)
{
   if (other) initializeValuesBy(*other);
}

void DataPack::addEnumMapping(const std::string& enum_name, const std::map<int, std::string>& mapping)
{
    enums_[enum_name] = mapping;

    for (const auto& enum_pair : mapping)
    {
       std::string var_name = fsm::EnumWrapper<DataPack>::getEnumValueName(enum_name, enum_pair.second);
       addConstVariable(var_name, enum_pair.first);
       hidden_vars_.insert(var_name);
    }
}

std::map<int, std::string> DataPack::getEnumMapping(const std::string& enum_variable_name) const
{
   return enums_.at(enum_variables_.at(enum_variable_name));
}

inline bool isNan(const float num)
{
   return num != num;
}

bool vfm::DataPack::hasEqualData(
   const DataPack& other, 
   const std::function<bool(const std::string& var_name, const DataPack& data_this, const DataPack& data_other)>& is_included) const
{
   std::map<std::string, float> arr_names;
   std::vector<std::string> var_names;
   
   // Collect all names from this...
   for (const auto& arr : this->arrays_) {
      arr_names.insert({ arr.first, arr.second->size() });
   }

   for (const auto& var : this->getAllSingleValVarNames()) {
      var_names.push_back(var);
   }

   // ...and other. If sizes differ, use max, since the DataPacks are still
   // considered equal if a value doesn't exist in one (implicit zero), and is zero in the other.
   // (TODO: This procedure is open for discussion, but consistent with how it's done with get and set.)
   for (const auto& arr : other.arrays_) {
      if (arr_names.count(arr.first)) {
         arr_names.at(arr.first) = (std::max)(arr.second->size(), arr_names.at(arr.first));
      }
      else {
         arr_names.insert({ arr.first, arr.second->size() });
      }
   }

   for (const auto& var : other.getAllSingleValVarNames()) {
      var_names.push_back(var);
   }
   
   for (const auto& arr_name : arr_names) {
      for (int i = 0; i < arr_name.second; ++i) {
         std::string name = arr_name.first;
         int val1 = getValFromArray(name, i);
         int val2 = other.getValFromArray(arr_name.first, i);

         if (is_included(name, *this, other)
            && val1 != val2 && (!isNan(val1) || !isNan(val2))) {
            addNote("Data comparison failed. Array '" + arr_name.first + "' differs between this and other at position " + std::to_string(i));
            addNote("Data from this:\r\n" + this->toString());
            addNote("Data from other:\r\n" + other.toString());
            return false;
         }
      }
   }

   for (const auto& var_name : var_names) {
      int val1 = this->getSingleVal(var_name);
      int val2 = other.getSingleVal(var_name);

      if (is_included(var_name, *this, other)
          && val1 != val2 && (!isNan(val1) || !isNan(val2))) {
         addNote("Data comparison failed. Variable '" + var_name + "' differs between this and other.");
         addNote("Data from this:\r\n" + this->toString());
         addNote("Data from other:\r\n" + other.toString());
         return false;
      }
   }

   return true;
}

bool vfm::DataPack::isHidden(const std::string & var_name) const
{
   return hidden_vars_.count(var_name);
}

std::string DataPack::getSingleValAsEnumName(const std::string& var_name, const bool simple, const bool discard_decimals_if_non_enum, const float special_value) const
{
   float val = std::isinf(special_value) ? getSingleVal(var_name) : special_value;
   std::string name = discard_decimals_if_non_enum ? std::to_string((int) val) : std::to_string(val);

   auto map_to_enum = enum_variables_.find(var_name);

   if (map_to_enum != enum_variables_.end())
   {
       name = "out_of_range";
       auto map_to_enum_map = enums_.find(map_to_enum->second);

       if (map_to_enum_map != enums_.end()) {
          auto map_to_num = map_to_enum_map->second.find(val);

          if (map_to_num != map_to_enum_map->second.end())
          {
             return simple ? map_to_num->second : fsm::EnumWrapper<DataPack>::getEnumValueName(map_to_enum_map->first, map_to_num->second);
          }
       }
   }

   return name;
}

void DataPack::associateVarToEnum(const std::string& var_name, const std::string enum_name)
{
    enum_variables_.insert(std::pair<std::string, std::string>(var_name, enum_name));
}

bool DataPack::isEnum(const std::string var_name) const
{
    return enum_variables_.find(var_name) != enum_variables_.end();
}

void vfm::DataPack::setValueAsEnum(const std::string var_name, const std::string enum_value)
{
   if (!isEnum(var_name)) {
      addError("Variable '" + var_name + "' is not an enum variable.");
      return;
   }

   auto en = getEnumMapping(var_name);
   for (const auto& el : en) {
      if (el.second == enum_value) {
         addOrSetSingleVal(var_name, el.first);
         return;
      }
   }

   addError("Enum variable '" + var_name + "' cannot be set to value '" + enum_value + "'");
}

std::string vfm::DataPack::serializeEnumTypes() const
{
   std::string s;

   for (const auto& el : enums_) {
      s += PROGRAM_ENUM_DENOTER + " ";
      s += el.first + " " + PROGRAM_DEFINITION_SEPARATOR + " " + PROGRAM_OPENING_ENUM_BRACKET;

      for (const auto& el2 : el.second) {
         s += el2.second + " " + PROGRAM_ARGUMENT_ASSIGNMENT + " " + std::to_string(el2.first) + FSM_PROGRAM_COMMAND_SEPARATOR + " ";
      }

      s += std::string(1, PROGRAM_CLOSING_ENUM_BRACKET) + PROGRAM_COMMAND_SEPARATOR + "\n";
   }

   return s;
}

std::string vfm::DataPack::serializeEnumVariables() const
{
   std::string s;

   for (const auto& el : enum_variables_) {
      s += PROGRAM_ENUM_DENOTER + " " + el.first + " " + PROGRAM_DEFINITION_SEPARATOR + " " + el.second + PROGRAM_COMMAND_SEPARATOR + "\n";
   }

   return s;
}

std::string vfm::DataPack::serializeEnumsFull() const
{
   return serializeEnumTypes() + "\n" + serializeEnumVariables();
}

std::string vfm::DataPack::serializeVariableValues() const
{
   std::string s;

   for (const auto& var : getAllSingleValVarNames()) {
      s += var + " " + PROGRAM_DEFINITION_SEPARATOR + " " + std::to_string(getSingleVal(var)) + PROGRAM_COMMAND_SEPARATOR + "\n";
   }

   return s;
}

int DataPack::getRecursionVarDepthID()
{
   return current_recursion_var_depth_id_;
}

void DataPack::incrementRecursionVarDepthID()
{
   current_recursion_var_depth_id_++;
}

void DataPack::decrementRecursionVarDepthID()
{
   current_recursion_var_depth_id_--;
}

std::string DataPack::toString() const
{
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

std::string vfm::DataPack::toStringHeap() const
{
   std::string s;
   int first_free_heap_index = (std::max)(first_free_heap_index_, highest_defined_heap_index_above_free_ + 1);

   s += "address\tvalue\t\tlabel\r\n";
   for (int i = 0; i < first_free_heap_index; i++) {
      auto var_name = addresses_to_names_.find(i);
      std::string positionOfFirstHeapCell = i == first_free_heap_index_ ? "*" : "";
      s += std::to_string(i) + positionOfFirstHeapCell + "\t" + std::to_string(getHeapLocation(i)) + "\t";
      
      if (var_name != addresses_to_names_.end()) {
         std::string is_const = isConst(var_name->second) ? " (const)" : "";
         s += var_name->second + is_const;
      }

      if (isExternalAssociation(i)) {
         s += "\t<EXT>";
      }

      s += "\r\n";
   }

   s += "--- end of used part of heap ---\r\n";

   if (first_free_heap_index < VFM_HEAP_SIZE) {
      if (first_free_heap_index < VFM_HEAP_SIZE - 1) {
         s += std::to_string(first_free_heap_index) + "\tundefined\r\n";
         if (first_free_heap_index < VFM_HEAP_SIZE - 2) {
            s += "\t.\r\n";
            s += "\t.\r\n";
            s += "\t.\r\n";
         }
      }
      s += std::to_string(VFM_HEAP_SIZE - 1) + "\tundefined\r\n";
   }

   s += "--- end of heap ---\r\n";

   return s;
}

std::string DataPack::toValString(const std::string& var_name)
{
   std::ostringstream strs;

   if (internal_variables_.count(var_name)) {
      strs << getSingleVal(var_name);
   }
   else if (static_external_bool_addresses_.find(var_name) != static_external_bool_addresses_.end()) {
      strs << *static_external_bool_addresses_.at(var_name);
   }
   else if (static_external_char_addresses_.find(var_name) != static_external_char_addresses_.end()) {
      strs << +*static_external_char_addresses_.at(var_name);
   }
   else if (static_external_float_addresses_.find(var_name) != static_external_float_addresses_.end()) {
      strs << *static_external_float_addresses_.at(var_name);
   }
   else if (arrays_.find(var_name) != arrays_.end()) {
      for (int i = 0; i < arrays_[var_name]->size(); ++i) {
         strs << arrays_[var_name]->get(i);
      }
   }
   else {
      strs << "No value stored for '" << var_name << "'.";
      throw std::runtime_error(strs.str());
   }

   return strs.str();
}

std::ostream& operator<<(std::ostream &os, const DataPack& m) {
   for (auto v : m.static_external_bool_addresses_) {
      std::string internal_address = std::string("*") + std::to_string(m.names_to_addresses_.at(v.first));
      std::string add_label = "";
        if (m.isEnum(v.first))
        {
            add_label = m.getSingleValAsEnumName(v.first) + " ";
        }
      os << v.first << "=" << add_label << *v.second << " -- " << internal_address << " ext. bool" << std::endl;
   }
   for (auto v : m.static_external_char_addresses_) {
      std::string internal_address = std::string("*") + std::to_string(m.names_to_addresses_.at(v.first));
      std::string add_label = "";
        if (m.isEnum(v.first))
        {
            add_label = m.getSingleValAsEnumName(v.first) + " ";
        }
      os << v.first << "=" << add_label << +*v.second << " -- " << internal_address << " ext. char" << std::endl;
   }
   for (auto v : m.static_external_float_addresses_) {
      std::string internal_address = std::string("*") + std::to_string(m.names_to_addresses_.at(v.first));
      std::string add_label = "";
        if (m.isEnum(v.first))
        {
            add_label = m.getSingleValAsEnumName(v.first) + " ";
        }
      os << v.first << "=" << add_label << *v.second << " -- " << internal_address << " ext. float" << std::endl;
   }
   for (auto v : m.internal_variables_) {
        std::string internal_address = std::string("*") + std::to_string(m.names_to_addresses_.at(v));
        std::string add_label = "";
        if (m.isEnum(v))
        {
            add_label = m.getSingleValAsEnumName(v) + " ";
        }

#if !defined(COMPOUND_DEBUG)
#if defined(FSM_DEBUG)
      if (!StaticHelper::stringStartsWith(v.first, "_"))
#else
       if (!m.hidden_vars_.count(v))
#endif
#endif
         os << (m.hidden_vars_.count(v) ? "(H) " : "") << v << "=" << add_label << m.getSingleVal(v) << " -- " << internal_address << std::endl;
   }
   for (auto v : m.arrays_) {
      std::stringstream ss;
      ss << *v.second;
      os << v.first << "[" << v.second->size() << "]=" << StaticHelper::replaceAll(StaticHelper::wrap(ss.str(), 80), "\n", " -- \n=") << std::endl;
   }

   return os;
}

int DataPack::getAddressOf(const std::string& var_or_arr_name) const
{
   return names_to_addresses_.at(var_or_arr_name);
}

std::string DataPack::getNameOf(const int var_or_arr_address) const
{
   auto name_ptr = addresses_to_names_.find(var_or_arr_address);

   if (name_ptr == addresses_to_names_.end()) {
      return CODE_FOR_ERROR_VAR_NAME;
   }
   
   return name_ptr->second;
}

std::vector<std::string> DataPack::getAllArrayNames() const
{
   std::vector<std::string> vec;

   for (const auto& pair : arrays_) {
      vec.push_back(pair.first);
   }

   return vec;
}

int vfm::DataPack::reserveHeap(const int cells)
{
   int first_cell = first_free_heap_index_;
   first_free_heap_index_ += cells;
   return first_cell;
}

int vfm::DataPack::deleteHeapCells(const int cells)
{
   int reserved = getUsedHeapSize();
   int to_delete = (std::min)(reserved, cells);
   first_free_heap_index_ -= to_delete;
   return to_delete;
}

int vfm::DataPack::getUsedHeapSize() const
{
   return first_free_heap_index_;
}

#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
void vfm::DataPack::precalculateExternalAddresses(int max_address_exclusive, int min_address_inclusive)
{
   if (min_address_inclusive < 0) {
      min_address_inclusive = external_addresses_need_recalculation_from_;
   }

   if (max_address_exclusive < 0) {
      max_address_exclusive = (std::max)(first_free_heap_index_, highest_defined_heap_index_above_free_ + 1);
   }

   assert(min_address_inclusive <= external_addresses_need_recalculation_from_); // Avoid gaps in recalculated array.
   assert(min_address_inclusive <= max_address_exclusive);
   assert(max_address_exclusive <= VFM_HEAP_SIZE);

   for (int reference = min_address_inclusive; reference < max_address_exclusive; reference++) {
      precalculated_external_addresses_.at(reference) = nullptr;

      for (const auto& pair : association_functions_) {
         for (const auto& f : pair.second) {
            auto void_ptr = f(reference, this->vfm_memory_, "", names_to_addresses_, addresses_to_names_, static_external_bool_addresses_, static_external_char_addresses_, static_external_float_addresses_);

            if (void_ptr) {
               precalculated_external_addresses_.at(reference) = void_ptr;
            }
         }
      }

      if (!precalculated_external_addresses_.at(reference)) {
         precalculated_external_addresses_.at(reference) = &vfm_memory_.at(reference);
      }
   }

   external_addresses_need_recalculation_from_ = max_address_exclusive;
}

std::vector<void*> vfm::DataPack::getPrecalculatedExternalAddresses()
{
   return precalculated_external_addresses_;
}

void vfm::DataPack::setParserForProgramParsing(const std::shared_ptr<FormulaParser> parser)
{
   parser_for_program_parsing_ = parser;
}

void vfm::DataPack::addAssociationFunctionToExternalValue(const AssociationFunction& f, const AssociationType type)
{
   association_functions_.at(type).push_back(f);
   external_addresses_need_recalculation_from_ = 0;
}

void vfm::DataPack::addAssociationFunctionToExternalValue(const AssociationFunctionSimple &f, const AssociationType type)
{
   addAssociationFunctionToExternalValue([f](
      const int heap_address,
      const std::vector<float>& vfm_memory,
      const std::string& optional_var_name,
      const std::map<const std::string, int>& names_to_addresses, 
      const std::map<const int, std::string>& addresses_to_names,
      const std::map<const std::string, bool*>& external_bool_addresses,
      const std::map<const std::string, char*>& external_char_addresses,
      const std::map<const std::string, float*>& external_float_addresses) {
      return f(heap_address, vfm_memory, names_to_addresses, addresses_to_names);
   }, type);
}

void vfm::DataPack::addAssociationFunctionToExternalValue(const AssociationFunctionVerySimple& f, const AssociationType type)
{
   addAssociationFunctionToExternalValue([f]( // Call core method rather than indirect call.
      const int heap_address,
      const std::vector<float>& vfm_memory,
      const std::string& optional_var_name,
      const std::map<const std::string, int>& names_to_addresses, 
      const std::map<const int, std::string>& addresses_to_names,
      const std::map<const std::string, bool*>& external_bool_addresses,
      const std::map<const std::string, char*>& external_char_addresses,
      const std::map<const std::string, float*>& external_float_addresses) {
      return f(heap_address, vfm_memory, names_to_addresses);
   }, type);
}

void vfm::DataPack::addAssociationFunctionToExternalValue(const AssociationFunctionVeryVerySimple& f, const AssociationType type)
{
   addAssociationFunctionToExternalValue([f]( // Call core method rather than indirect call.
      const int heap_address,
      const std::vector<float>& vfm_memory,
      const std::string& optional_var_name,
      const std::map<const std::string, int>& names_to_addresses, 
      const std::map<const int, std::string>& addresses_to_names,
      const std::map<const std::string, bool*>& external_bool_addresses,
      const std::map<const std::string, char*>& external_char_addresses,
      const std::map<const std::string, float*>& external_float_addresses) {
      return f(heap_address);
   }, type);
}

void DataPack::associateExternalFloatArray(const std::string& var_name, const float* outside_array_ref, const int length, const bool array_exists_already)
{
   int vfm_ref;

   if (array_exists_already) {
      vfm_ref = getSingleVal(var_name);
   }
   else {
      vfm_ref = reserveHeap(length);
      addOrSetSingleVal(var_name, vfm_ref);
   }

   addAssociationFunctionToExternalValue([vfm_ref, outside_array_ref, length](const int heap_address) -> void* {
      if (heap_address >= vfm_ref && heap_address < vfm_ref + length) {
         return (void*) (outside_array_ref + heap_address - vfm_ref);
      }

      return nullptr;
   }, AssociationType::Float);
}

void DataPack::associateSingleValWithExternalBool(const std::string& name, bool* val)
{
   static_external_bool_addresses_[name] = val;
   registerAddress(name);
}

void DataPack::associateSingleValWithExternalChar(const std::string& name, char* val)
{
   static_external_char_addresses_[name] = val;
   registerAddress(name);
}

void DataPack::associateSingleValWithExternalFloat(const std::string& name, float* val)
{
   static_external_float_addresses_[name] = val;
   registerAddress(name);
}
#endif

bool vfm::DataPack::isConst(const std::string& var_name) const
{
   return const_variables_.count(var_name);
}

std::set<std::string> vfm::DataPack::getAllVariableNamesMatchingRegex(const std::string& regex_str, const bool include_hidden) const
{
   std::set<std::string> set{};
   std::regex regex{ };

   try {
      regex = std::regex(regex_str);
   }
   catch (const std::regex_error&) {
      return { "MALFORMED_REGEX" };
   }

   for (const auto& var : getAllSingleValVarNames(include_hidden)) {
      if (std::regex_match(var, regex)) {
         set.insert(var);
      }
   }

   return set;
}

std::set<std::string> vfm::DataPack::getAllArrayNamesMatchingRegex(const std::string& regex_str, const bool include_hidden) const
{
   std::set<std::string> set{};
   std::regex regex{ };

   try {
      regex = std::regex(regex_str);
   }
   catch (const std::regex_error&) {
      return {"MALFORMED_REGEX"};
   }

   for (const auto& var : getAllArrayNames()) {
      if (std::regex_match(var, regex)) {
         set.insert(var);
      }
   }

   return set;
}

std::vector<float> vfm::DataPack::getVfmMemory() const
{
   return vfm_memory_;
}

void vfm::DataPack::addStringToDataPack(const std::string& string_raw, const std::string& var_name)
{
   std::string string{ string_raw };
   int address = -1;

   if (isDeclared(var_name)) {
      bool same_length_as_old;
      bool same_as_old;
      isStringAtAdressWhichComparesTo(var_name, string, same_length_as_old, same_as_old);

      if (same_as_old) return;

      if (same_length_as_old) { // TODO: This is still dangerous, might override variables until, by accident, a \0 is found. Should be sufficiently improbable, for now.
         address = getSingleVal(var_name);
      }
   }

   string.push_back('\0');
   address = address < 0 ? reserveHeap(string.size()) : address;

   for (int i = 0; i < string.size(); i++) {
      setHeapLocation(address + i, (float)string[i]);
   }

   addOrSetSingleVal(var_name, address);
}

std::string vfm::DataPack::printHeap(const std::string& varname, const std::string& default_on_undeclared) const
{
   if (!isDeclared(varname)) { return default_on_undeclared; } // Ignore calls on non-existing variables.

   std::string str{};

   for (int i = getSingleVal(varname); getVfmMemory()[i] != 0; i++) {
      str += (char)getVfmMemory()[i];
   }

   return StaticHelper::replaceAll(str, "\\n", "\n");
}

void vfm::DataPack::isStringAtAdressWhichComparesTo(const std::string& address_var, const std::string& comp_string, bool& same_length, bool& same)
{
   if (!isDeclared(address_var) || getSingleVal(address_var) < 0 || getSingleVal(address_var) >= VFM_HEAP_SIZE) {
      same_length = false;
      same = false;
      return;
   }

   same_length = true;
   same = true;

   std::string str{};
   int i = getSingleVal(address_var);

   for (; getVfmMemory()[i] != 0; i++) {
      str += (char)getVfmMemory()[i];
      if (str.size() > comp_string.size()) {
         same_length = false;
         same = false;
         return;
      }
   }

   if (str.size() != comp_string.size()) {
      same_length = false;
      same = false;
      return; // Fail fast.
   }

   if (str != comp_string) {
      same = false;
   }
}

void vfm::DataPack::resetPrivateVarsRecursiveLevels()
{
   private_vars_recursive_levels_.clear();
}

ScriptData& vfm::DataPack::getScriptData()
{
   return script_data_;
}

std::shared_ptr<DataPack> DataPack::instance_;

std::shared_ptr<DataPack> vfm::DataPack::getSingleton()
{
   if (!instance_) {
      instance_ = std::make_shared<DataPack>();
   }

   return instance_;
}
