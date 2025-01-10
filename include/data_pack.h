//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "parsable.h"
#include <functional>
#include <set>
#include <map>
#include <memory>
#include <limits>


namespace vfm { class DataPack; }
std::ostream& operator<<(std::ostream &os, const vfm::DataPack & m);
constexpr auto VFM_HEAP_SIZE = 1000000;
constexpr auto VFM_HEAP_EMPTY_VALUE = 0;

namespace vfm {

class MathStruct;
class Term;
class DataSrcArray;
class FormulaParser;

enum class ArrayMode 
{
   implicit,
   chars,
   floats,
   chars_read_only_file,
   chars_random_access_file
};

constexpr int INITIAL_ARRAY_CAPACITY = 1000;
const std::string CODE_FOR_ERROR_VAR_NAME = "";

/// \brief The DataPack handles all data operations of the vfm framework, including creating, 
/// writing to and reading from internal and external variables and arrays.
///
/// Note on thread-safety: The DataPack is not thread-safe, and it would probably make no sense to make it.
/// Apart from applying the usual locking mechanisms to members, the handling of private 
/// variables in recursive calls would have to be reimplemented which would be a lot of effort.
/// Also, this would make the data structure inefficient for non-multi-thread usage, and
/// hardly significantly more efficient with several threads than just by making sure from outside
/// that no two formulae are processed simultaneously.
class DataPack : public Parsable
{
public:
   DataPack();

   friend std::ostream& ::operator<<(std::ostream& os, const DataPack& m);

   std::shared_ptr<std::vector<std::shared_ptr<Term>>> getValuesForMeta() const;
   void popValuesForMeta() const;
   void pushValuesForMeta(const std::shared_ptr<std::vector<std::shared_ptr<Term>>>& value_terms) const;

   /// \brief Uses values from "other" to initialize the corresponding values in this.
   void initializeValuesBy(const DataPack& other);
   void initializeValuesBy(const std::shared_ptr<DataPack> other);

   std::string toString() const;
   std::string toStringHeap() const;
   std::string toValString(const std::string& var_name);
   void setSingleVal(const int var_reference, float val, const bool hidden = false);

   void addConstVariable(const std::string& name, float val, const bool hidden = false);
   void addOrSetSingleVal(const std::string& name, float val, const bool hidden = false, const bool is_const = false);
   void addOrSetSingleVals(const std::vector<std::string>& names, float val, const bool hidden = false, const bool is_const = false);

   /// Declares a single-value variable of the given name and sets it to a given value 
   /// (default = 0) iff none with this name existed before. Otherwise does nothing.
   void addSingleValIfUndeclared(const std::string& name, const bool hidden = false, const float init_val = 0);
   void addSingleValsIfUndeclared(const std::vector<std::string>& names, const bool hidden = false, const float init_val = 0);

   /// Returns true if a variable of the given name already exists in the data pack.
   bool isVarDeclared(const std::string& name) const;

   /// Returns true if an array or a variable of the given name already exists in the data pack.
   bool isDeclared(const std::string& name) const;

   bool isInternalValue(const std::string& name) const;

   bool isExternalAssociation(const int heap_address) const;
   bool isExternalValue(const std::string& name) const;
   bool isExternalBool(const std::string& name) const;
   bool isExternalChar(const std::string& name) const;
   bool isExternalFloat(const std::string& name) const;

   void setHeapLocation(const int reference, const float val);
   float getHeapLocation(const int reference) const;
   void setArrayVal(const int arr_reference, const int index, const float val);
   void setArrayVal(const std::string& arr_name, const int index, const float val);
   float getArrayVal(const int arr_reference, const int index) const;
   float getArrayVal(const std::string& arr_name, const int index) const;

   ///// Old-school static arrays /////

   // These arrays are somewhat deprecated, but they still offer some functionality that
   // cannot be emulated by the new dynamic heap data structure (such as storing on HDD).
   // In long-term future they will go away, though.

   void addArrayAndOrSetArrayVal(const std::string& name, const int& index, const float& val, const ArrayMode mode_if_new = ArrayMode::implicit);
   void addReadonlyStringAsArray(const std::string& name, const std::string& val);
   void addStringAsArray(const std::string& name, const int& capacity);
   void addStringAsArray(const std::string& name, const std::string& val);
   void addStringAsFloatVector(const std::string& name, const int& capacity);
   void addStringAsFloatVector(const std::string& name, const std::vector<float>& raw_data);
   float getValFromArray(const std::string& arr_name, const int& index) const;
   int getArraySize(const std::string& arr_name) const;
   void setArrayViaRawSource(const std::string& arr_name, const std::shared_ptr<DataSrcArray>& data_source);
   DataSrcArray& getStaticArray(const int arr_num) const;
   DataSrcArray& getStaticArray(const std::string& arr_name) const;

   void setArrayValuesByCopyingRawSource(
      const DataSrcArray& raw_src, const std::string& to_arr_name, const int begin_pos);

   void copyArrayValues(const std::string& from_arr_name, 
      const int range_begin_pos_inc,
      const int range_end_pos_exc,
      const std::string& to_arr_name, 
      const int to_begin_pos
   );

   /// Returns true if an array of the given name already exists in the data pack.
   bool isArrayDeclared(const std::string& name) const;

   bool isStaticArray(const int arr_reference) const;
   ///// EO Old-school static arrays /////

   void declareVariable(const std::string& var_name, const bool is_const = false, const bool is_hidden = false);

   float getSingleVal(const std::string& var_name) const;
   std::set<std::string> getAllSingleValVarNames(const bool include_hidden = true) const;
   int getAddressOf(const std::string& var_or_arr_name) const;
   std::string getNameOf(const int var_or_arr_address) const;

   /// \brief Parses lines from a data pack program. Note that this method assumes
   /// that the input is syntactically correct.
   bool parseProgram(const std::string& data_commands) override;

   void addEnumMapping(const std::string& enum_name, const std::map<int, std::string>& mapping);
   void associateVarToEnum(const std::string& var_name, const std::string enum_name);
   std::string getSingleValAsEnumName(const std::string& var_name, const bool simple = true, const bool discard_decimals_if_non_enum = false, const float special_value = std::numeric_limits<float>::infinity()) const;
   bool isEnum(const std::string var_name) const;
   void setValueAsEnum(const std::string var_name, const std::string enum_value);
   std::string serializeEnumTypes() const;
   std::string serializeEnumVariables() const;
   std::string serializeEnumsFull() const;
   std::string serializeVariableValues() const;

   void printMetaStack();

   /// \brief Sets all members to initial values. External values are not changed, but the associations
   /// are cleared.
   void reset();
   void resetPrivateStuff();

   /// \brief Retrieves the address of the given variable IFF it is an internal variable OR
   /// an external float variable. Otherwise (particularly for external bools/chars) returns nullptr.
   float* getAddressOfSingleVal(const std::string& var_name) const;

   /// \brief Checks if the name denotes an external bool, returns its
   /// address if true, nullptr otherwise.
   bool* getAddressOfExternalBool(const std::string name) const;

   /// \brief Checks if the name denotes an external char, returns its
   /// address if true, nullptr otherwise.
   char* getAddressOfExternalChar(const std::string name) const;

   /// \brief Checks if the name denotes an external float, returns its
   /// address if true, nullptr otherwise. To get the address of both external and
   /// internal floats, use getAddressOfSingleVal;
   float* getAddressOfExternalFloat(const std::string name) const;

   /// \brief The address of the beginning of a char array (nullptr if the array
   /// is a float array, use the getAddressOfFloatArray method in this case.)
   const char* getAddressOfArray(const std::string& arr_name) const;

   /// \brief The address of the beginning of a float array (nullptr if the array
   /// is a char array, use the getAddressOfArray method in this case.)
   const float* getAddressOfFloatArray(const std::string & arr_name) const;

   /// The recursive level is added to a variable name if it is a private variable in a recursive call,
   /// to differentiate between private variables on different levels of recursion.
   /// Note that from outside only the original name of the variable (i.e. _var_name#*privNum*)
   /// has to be provided. The additional name postamble if handled internally.
   void setVarsRecursiveIDs(const std::vector<std::string>& var_names, const int recursive_level);

   int getVarRecursiveID(const std::string& var_name) const;

   int getRecursionVarDepthID();
   void incrementRecursionVarDepthID();
   void decrementRecursionVarDepthID();

   std::map<int, std::string> getEnumMapping(const std::string& enum_variable_name) const;

   bool hasEqualData(
      const DataPack& other, 
      const std::function<bool(const std::string& var_name, const DataPack& data_this, const DataPack& data_other)>& is_included
         = [](const std::string& var_name, const DataPack& data_this, const DataPack& data_other) { return true; }) const;

   bool isHidden(const std::string& var_name) const;

   std::vector<std::string> getAllArrayNames() const;

   int reserveHeap(const int cells);
   int deleteHeapCells(const int cells);
   int getUsedHeapSize() const;

   std::vector<float> getVfmMemory() const;
   void addStringToDataPack(const std::string& string_raw, const std::string& var_name);
   std::string printHeap(const std::string& varname, const std::string& default_on_undeclared = "") const;

   void resetPrivateVarsRecursiveLevels();


#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   // The mechanism for associating heap cells to external variables can be relatively expensive.
   // Considering the crucial role of memory management, it can be turned off completely by the above flag.
   
   /// Uses the provided association functions once to calculate the external addresses for a faster access during execution.
   /// This assumes that the association functions return the same value at each call. If that is not the case, do not use this function.
   /// Adding a new association will reset the precalculated addresses, i.e., you'll need to re-run this function.
   ///
   /// If there is no external association, the respecitve cell entry is the address in the internal heap.
   ///
   /// Contract: After calling this method and until calling one of the "association" methods...
   /// - the array returned by getPrecalculatedExternalAddresses
   ///   will contain for each of the entries between min_address_inclusive and max_address_exclusive - 1 the address of the value of the
   ///   respective vfm heap reference (be it on the internal heap or somewhere externally). This assumes constant return values by the
   ///   association functions; and
   /// - subsequent calls of this method WITHOUT ARGUMENTS will never result in some cell being calculated more than once.
   /// 
   ///
   /// CAUTION: Note that there is no automatic precalculation, or storage of addresses that happen to have been already calculated,
   /// because it can be intended to have different addresses at different stages of the program. Assembly cannot be generated
   /// for getter functions in this case - which cannot be automatically detected, though!
   ///
   /// max_address_exclusive < 0: Set max address to max defined heap address.
   /// min_address_inclusive < 0: Set min address to first not yet precalculated address.
   ///
   /// TODO: Use precalculated values throughout DataPack, if available.
   /// TODO: Handle bool and char; for now this works only with float externals (unless you know from somewhere else which type your entry is).
   void precalculateExternalAddresses(int max_address_exclusive = -1, int min_address_inclusive = -1);

   std::vector<void*> getPrecalculatedExternalAddresses();

   void setParserForProgramParsing(const std::shared_ptr<FormulaParser> parser);

   enum class AssociationType
   {
      Float,
      Char,
      Bool,
      None,
   };

   using AssociationFunction = std::function<void*(
      const int heap_address,
      const std::vector<float>& vfm_memory,
      const std::string& optional_var_name, // If the address is labeled, there MAY be the name stored here to allow for faster processing.
      const std::map<const std::string, int>& names_to_addresses, 
      const std::map<const int, std::string>& addresses_to_names,
      const std::map<const std::string, bool*>& external_bool_addresses,
      const std::map<const std::string, char*>& external_char_addresses,
      const std::map<const std::string, float*>& external_float_addresses)>;

   using AssociationFunctionSimple = std::function<void*(
      const int heap_address, 
      const std::vector<float>& vfm_memory,
      const std::map<const std::string, int>& names_to_addresses, 
      const std::map<const int, std::string>& addresses_to_names)>;

   using AssociationFunctionVerySimple = std::function<void*(
      const int heap_address, 
      const std::vector<float>& vfm_memory,
      const std::map<const std::string, int>& names_to_addresses)>;

   using AssociationFunctionVeryVerySimple = std::function<void*(
      const int heap_address)>;

   /// Associate arbitrary heap chunks with arbitrary external values of a 
   /// specific type (bool, char, float, ...).
   ///
   /// Most advanced version of the method (finally called). It allows access to
   /// the internally stored static connections between labeled variables and 
   /// external variables.
   void addAssociationFunctionToExternalValue(const AssociationFunction& f, const AssociationType type = AssociationType::Float);

   /// Associate arbitrary heap chunks with arbitrary external values of a 
   /// specific type (bool, char, float, ...).
   void addAssociationFunctionToExternalValue(const AssociationFunctionSimple& f, const AssociationType type = AssociationType::Float);

   /// Associate arbitrary heap chunks with arbitrary external values of a 
   /// specific type (bool, char, float, ...).
   void addAssociationFunctionToExternalValue(const AssociationFunctionVerySimple& f, const AssociationType type = AssociationType::Float);

   /// Associate arbitrary heap chunks with arbitrary external values of a 
   /// specific type (bool, char, float, ...).
   void addAssociationFunctionToExternalValue(const AssociationFunctionVeryVerySimple& f, const AssociationType type = AssociationType::Float);

   void associateExternalFloatArray(const std::string& var_name, const float* outsideArray, const int length, const bool array_exists_already = false);
   void associateSingleValWithExternalBool(const std::string& name, bool* val);
   void associateSingleValWithExternalChar(const std::string& name, char* val);
   void associateSingleValWithExternalFloat(const std::string& name, float* val);

   bool isConst(const std::string& var_name) const;

   std::set<std::string> getAllVariableNamesMatchingRegex(const std::string& regex_str, const bool include_hidden = false) const;
   std::set<std::string> getAllArrayNamesMatchingRegex(const std::string& regex_str, const bool include_hidden = false) const;

private:
   std::map<AssociationType, std::vector<AssociationFunction>> association_functions_{};
public:
#endif // DISALLOW_EXTERNAL_ASSOCIATIONS

   static std::shared_ptr<DataPack> getSingleton();

private:
   static std::shared_ptr<DataPack> instance_;

   std::shared_ptr<FormulaParser> parser_for_program_parsing_{};
   int external_addresses_need_recalculation_from_{ 0 };
   std::vector<void*> precalculated_external_addresses_{};

   int current_recursion_var_depth_id_{ 0 };
   int first_free_heap_index_{ 0 };
   int highest_defined_heap_index_above_free_{ -1 }; /// Only defined if a reference above first_free_heap_index_ has been accessed.
   int first_free_array_index_{ VFM_HEAP_SIZE };
   std::map<std::string, int> private_vars_recursive_levels_{};
   std::map<const std::string, std::shared_ptr<DataSrcArray>> arrays_{};
   std::map<const std::string, int> names_to_addresses_{};
   std::map<const int, std::string> addresses_to_names_{};
   std::vector<float> vfm_memory_{};
   std::set<std::string> internal_variables_{};
   std::map<const std::string, bool*> static_external_bool_addresses_{};
   std::map<const std::string, char*> static_external_char_addresses_{};
   std::map<const std::string, float*> static_external_float_addresses_{};

   std::set<std::string> const_variables_{};

   std::map<const std::string, std::map<int, std::string>> enums_{};
   std::map<const std::string, std::string> enum_variables_{};

   std::set<std::string> hidden_vars_{};

   mutable std::vector<std::shared_ptr<std::vector<std::shared_ptr<Term>>>> stack_terms_for_meta_{};

   void setArrayAt(const std::string& arr_name, const int& index, const float& val) const;
   void registerArray(const std::string& name, const std::shared_ptr<DataSrcArray>& dt);
   void registerAddress(const std::string& name, const bool var = true);
   bool checkHeapAddress(const int address) const;
   bool pedanticCheckHeapAddress(const int address) const;
};

using DataPackPtr = std::shared_ptr<DataPack>;
using ::operator<<;

} // vfm
 