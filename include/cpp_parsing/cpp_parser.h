//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "cpp_parsing/cpp_type_struct.h"
#include "parsable.h"
#include "parser.h"
#include "cpp_parsing/options.h"
#include "static_helper.h"
#include <map>
#include <vector>
#include <string>


namespace vfm {

class Term;

namespace mc {
namespace smv {
   class Module;
}
}

using CppFunctionSignature = std::pair<CppType, std::vector<CppFullTypeWithName>>;

const std::string NATIVE_SMV_ENV_MODEL_DENOTER_OPEN = "$<";
const std::string NATIVE_SMV_ENV_MODEL_DENOTER_ALWAYS_REGENERATE = "G"; // Or else use cached if there.
const std::string NATIVE_SMV_ENV_MODEL_DENOTER_CLOSE = ">";
const std::string NATIVE_SMV_ENV_MODEL_SEPARATOR = ",";
const std::string NATIVE_SMV_ENV_MODEL_ASSIGNMENT = "=";

const std::string VFM_CONTINUING_COMMENT_DENOTER = "\\$\\$"; // Allows comments to continue in next line ==> // COMMENT $$ \n // COMMENT_GOING_ON ("$$ \n //" will be deleted)
const std::string VFM_BEGIN = "// #vfm-begin"; // Parse only what is between these two tags.
const std::string VFM_END = "// #vfm-end";
const std::string VFM_CUTOUT_DENOTER_BEGIN = "// #vfm-cutout-begin"; // ...Except for what is between these two tags.
const std::string VFM_CUTOUT_DENOTER_END = "// #vfm-cutout-end";
const std::string VFM_INSERT_CODE = "// #vfm:"; // Treat remaining line as plain uncommented code to vfm. (I.e., vfm code that C++ doesn't have to know of.)
const std::string VFM_GENCODE_DENOTER_BEGIN = "// #vfm-gencode-begin"; // Remove what is between these two tags and replace it...
const std::string VFM_GENCODE_DENOTER_END = "// #vfm-gencode-end"; // ...with generated vfm state machine code.
const std::string VFM_GENCODE_PERSISTING_DENOTER = ":::vfm.gencode.begin.tag:::"; // ...with generated vfm state machine code.
const std::string VFM_INCLUDES_DENOTER = "// #vfm-insert-includes";
const std::string VFM_ASSOCIATIVITY_PRETEXT_DENOTER = "// #vfm-insert-associativity-pretext";

const std::string VFM_TAL_INFO_DENOTER = "// #vfm-tal";
const std::string VFM_AKA_INFO_DENOTER = "// #vfm-aka";
const std::string VFM_INIT_INFO_DENOTER = "// #vfm-init";
const std::string VFM_LTL_SPEC_DENOTER = "// #vfm-ltl-spec";
const std::string VFM_CTL_SPEC_DENOTER = "// #vfm-ctl-spec";
const std::string VFM_KRATOS_SPEC_DENOTER = "// #vfm-kratos-spec";
const std::string VFM_INITIAL_STATE_CONSTRAINT_DENOTER = "// #vfm-init-constraint";
const std::string VFM_INVARIANT_DENOTER = "// #vfm-invar";

const std::string VFM_INJECTION_DENOTER = "// #vfm-inject"; // Injects the given code into vfm before the parsing starts (as opposed to: at the position where written, which can be achieved via '// #vfm:').
const std::string VFM_OPTION_DENOTER = "// #vfm-option";


enum class VariableSpecifierModeEnum {
   AKA,
   TAL,
   INIT
};

const std::map<VariableSpecifierModeEnum, std::string> variable_specifier_mode_map{
   { VariableSpecifierModeEnum::AKA, VFM_AKA_INFO_DENOTER },
   { VariableSpecifierModeEnum::TAL, VFM_TAL_INFO_DENOTER },
   { VariableSpecifierModeEnum::INIT, VFM_INIT_INFO_DENOTER },
};

class VariableSpecifierMode : public fsm::EnumWrapper<VariableSpecifierModeEnum>
{
public:
   explicit VariableSpecifierMode(const VariableSpecifierModeEnum& enum_val) : EnumWrapper("VariableSpecifierMode", variable_specifier_mode_map, enum_val) {}
   explicit VariableSpecifierMode(const int enum_val_as_num) : EnumWrapper("VariableSpecifierMode", variable_specifier_mode_map, enum_val_as_num) {}
   explicit VariableSpecifierMode(const std::string& enum_val_as_string) : EnumWrapper("VariableSpecifierMode", variable_specifier_mode_map, enum_val_as_string) {}
   explicit VariableSpecifierMode() : EnumWrapper("VariableSpecifierMode", variable_specifier_mode_map, VariableSpecifierModeEnum::TAL) {}
};

enum class ChunkTypeEnum {
   Enum,
   EnumClass,
   Struct,
   Function,
   Namespace,
   BraceInitializer,
   Unknown,
   None
};

const std::map<ChunkTypeEnum, std::string> chunk_type_map{
   { ChunkTypeEnum::Enum, "Enum" },
   { ChunkTypeEnum::EnumClass, "EnumClass" },
   { ChunkTypeEnum::Struct, "Struct" },
   { ChunkTypeEnum::Function, "Function" },
   { ChunkTypeEnum::Namespace, "Namespace" },
   { ChunkTypeEnum::BraceInitializer, "BraceInitializer" },
   { ChunkTypeEnum::Unknown, "Unknown" },
   { ChunkTypeEnum::None, "None" },
};

class ChunkType : public fsm::EnumWrapper<ChunkTypeEnum>
{
public:
   explicit ChunkType(const ChunkTypeEnum& enum_val) : EnumWrapper("ChunkType", chunk_type_map, enum_val) {}
   explicit ChunkType(const int enum_val_as_num) : EnumWrapper("ChunkType", chunk_type_map, enum_val_as_num) {}
   explicit ChunkType(const std::string& enum_val_as_string) : EnumWrapper("ChunkType", chunk_type_map, enum_val_as_string) {}
   explicit ChunkType() : EnumWrapper("ChunkType", chunk_type_map, ChunkTypeEnum::Unknown) {}
};

const std::string GENERATED_FSM_NAME = "__fsm__";
const std::string GENERATED_PROGRAM_NAME = "__program__";
const std::string VARIABLE_NAME_FOR_MAIN_FUNCTION_DENOTER = "...main_fsm_extracted_from_here...";
const std::string ERROR_STRING = "##-MALFORMED-##";

const std::string TAG_OPTION_OPENING_BRACKET = "[[";
const std::string TAG_OPTION_CLOSING_BRACKET = "]]";
constexpr char TAG_OPTION_DELIMITER = ',';
constexpr char TAG_OPTION_ASSIGNMENT_SIGN = '=';

const std::string TAG_OPTION_GEN_CODE_CONDITION = "condition";

const std::string NULL_SIGNATURE_RETURN_TYPE = "#NULL$DUMMY#"; // TODO: Could be also nullptr in NULL_SIGNATURE return value, if this cannot happen otherwise.
const CppFunctionSignature NULL_SIGNATURE = { std::make_shared<CppTypeStruct>(NULL_SIGNATURE_RETURN_TYPE), {} };

const std::string AUTHOR_NAME = "Lukas Koenig";
const std::string AUTHOR_SHORT = "okl2abt";
const std::string AUTHOR_EMAIL = "lukas.koenig@de.bosch.com";

const std::string START_SYMBOL_ENUM_CLASS = "EnumClass~";

const std::string GRAMMAR_GENERAL_RULES = R"(
   SpaceSeq :=> :| *WSP*'SpaceSeq$
   Char :=> 'UpperCase' :| 'LowerCase'$
   UpperCase :=>  A :| B :| C :| D :| E :| F :| G :| H :| I :| J :| K :| L :| M :| N :| O :| P :| Q :| R :| S :| T :| U :| V :| W :| X :| Y :| Z$
   LowerCase :=>  a :| b :| c :| d :| e :| f :| g :| h :| i :| j :| k :| l :| m :| n :| o :| p :| q :| r :| s :| t :| u :| v :| w :| x :| y :| z$
   Num :=> 0 :| 1 :| 2 :| 3 :| 4 :| 5 :| 6 :| 7 :| 8 :| 9$
   NumSeq :=> :| Num'NumSeq$
   CharOrNumSeq :=> :| 'Char'CharOrNumSeq' :| 'Num'CharOrNumSeq'$
   VarNamePlain :=> 'Char'CharOrNumSeq'$
   )";

const std::string GRAMMAR_RULES_FOR_ENUM_CLASS = START_SYMBOL_ENUM_CLASS + R"(
    :=> 'Enum'SpaceSeq'Class'SpaceSeq'VarNamePlain'SpaceSeq'{'SpaceSeq'EnumElements'}'$
   Enum :=> 'e'n'u'm'$
   Class :=> 'c'l'a's's'$
   EnumElements :=> EnumElement'SpaceSeq :| EnumElement'SpaceSeq','SpaceSeq'EnumElements$
   EnumElement :=> VarNamePlain :| VarNamePlain'SpaceSeq'='SpaceSeq'Num'NumSeq$
   )";

const std::string SYMB_RETURN = "return";

static std::ostream* ADDITIONAL_LOGGING_PIPE{};

/// Note that all the "magic" nubers and strings in this class are C++ keywords or
/// other fixed parts of the language. Since there is no reason to expect these values
/// to change any time in the future, there seems to be no good reason to encapsulate
/// something like "switch" into a SWITCH_KEY_WORD_CPP constant.
class CppParser : public Parsable {
public:
   static std::string TARGET_PATH_FOR_MORTY_GUY_PROGRESS;

   CppParser(const std::string& command_line_argument);
   void reset(const std::shared_ptr<DataPack> data_to_take_over = nullptr, const bool has_env_model_data = false);

   std::vector<std::string> readFileListFromFile(const std::string& file_path);
   std::string preprocessProgram(const std::string& program, const bool preliminary);
   bool parseProgram(const std::string& program) override;
   bool parseProgramFromFiles(const std::vector<std::string>& file_list);
   bool parseProgramByLoadingFileListFromFile(const std::string& file_path);

   std::map<std::string, CppFunctionSignature>& getFunctionSignatures();
   std::vector<std::shared_ptr<CppTypeStruct>>& getDataTypes();

   std::string getTypeBaseName(const std::string& member) const;

   void processTypeForTAL(
      const vfm::TypeWithInitAndRangeOrAka& pair,
      const std::string& NO_INIT_DENOTER,
      const std::string& member,
      const vfm::CppType& type,
      const std::shared_ptr<mc::TypeAbstractionLayer> tal) const;

   bool fillTypeIntoTAL(const std::string& member, const std::shared_ptr<mc::TypeAbstractionLayer> tal) const;

   std::vector<std::string> findAllRecursiveVariableNames(const bool quiet) const;

   void printInfos();
   bool performFSMCodeGenerationWrapper(
      const char* path_to_file_list_file,
      const char* target_path_for_generated_code,
      const bool create_live_pdf,
      std::string& path_to_envmodel_file,
      const std::vector<std::string>& template_options,
      const bool env_model_info_available = false);

   void setTypeOrSubTypeConstSafe(const std::string& m_var_name, const float const_value, const std::shared_ptr<vfm::TermVar>& m_var);

   void createConstantsFromPointRangeVariables();
   void insertLiteralValuesForConstants();
   void inferRangeForLocalVariables();
   void postprocessFinalModel();
   void applyPrecisionIncreaseTo(const TermPtr fmla) const;

   std::string getInitVarStringNuSMVOrKratos(const std::string& evar, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, const bool init = false, const bool next = false, const bool kratos = false, const std::map<std::string, std::string>& enum_values = {}) const;

   bool performCodeGenerationAndMCCodeCreation(
      const char* path_to_file_list_file_code,
      const char* path_to_file_list_file_env_model,
      const char* target_path_for_generated_code,
      const bool create_live_pdf);

   /// Basically puts all variables into fsm_->data_ and sets them to their
   /// correct init value. The returned TAL is the unchanged type_abstraction_layer_.
   std::shared_ptr<vfm::mc::TypeAbstractionLayer> prepareVariablesForPrematureTal() const;

   std::pair<float, float> deriveGlobalRangeFromLeafRanges(const MathStructPtr m) const;

   std::map<std::string, std::string> collectEnumsForKratos() const;
   std::string generateGraphvizWithTypes(const MathStructPtr m) const;
   bool checkForVariablesWithoutType(const MathStructPtr formula) const;

   std::string getCommandLineArgument() const;

   inline std::vector<TermPtr>& getInitialStateConstraints() const
   {
      return all_initial_state_constraints_;
   }

   inline std::vector<TermPtr>& getInvariants() const
   {
      return all_invariants_;
   }

   inline std::vector<TermPtr>& getKratosSpecs() const
   {
      return all_kratos_constraints_;
   }

   static std::string toStringFunctionSignature(const CppFunctionSignature& func_sig);
   static bool signaturesAreEqual(const CppFunctionSignature& s1, const CppFunctionSignature& s2);
   static bool typesAreEqual(const CppFullTypeWithName& s1, const CppFullTypeWithName& s2, const bool ignore_qualifiers = false);
   static std::string findSurroundingNamespace(const std::string code, const int pos);

private:
   ChunkTypeEnum findIndices(
      std::string& program,
      std::string& chunk,
      std::string& name,
      std::string& type,
      std::vector<std::string>& signature,
      int& beg,
      int& end,
      std::vector<int>& except,
      std::string& namespace_str,
      std::deque<std::string>& relevant_prefix_tokens,
      std::vector<std::string>& father_classes) const;

   void writeMortyGUIProgressFile(const int percent, const std::string& stage_name) const;
   void deleteMortyGUIProgressFile() const;

   void applyConstnessToAllMCItems();

   void applyDefines(const std::string& name, std::string& type) const;
   void applyInits(const std::string& name);
   bool parseNativeEnvModel(const std::string& program);

   std::pair<float, float> getSingleRange(const MathStructPtr m, const std::pair<float, float>& range1, const std::pair<float, float>& range2) const;
   void findVariableNamesInArrayAccess(std::string& original_arr_full_access, std::string& arr_full_access, std::set<std::string>& var_names_in_single_array);
   void createDimsVector(std::set<std::string>& var_names_in_single_array, std::shared_ptr<vfm::mc::TypeAbstractionLayer>& tal, std::vector<int>& dims_counter, std::vector<int>& dims, std::vector<int>& dims_min);
   void createIfStatement(std::vector<int>& dims_counter, std::shared_ptr<vfm::Term>& current_if, std::vector<int>& dims, std::string& arr_full_access, std::vector<int>& dims_min, std::set<std::string>& var_names_in_single_array, std::shared_ptr<vfm::TermVar>& m2_var, vfm::TermPtr& m2_var_right_side, std::string& original_arr_full_access, std::shared_ptr<vfm::mc::TypeAbstractionLayer>& tal);

   void applySMVOperatorsToParser() const;
   void addAtomicDataTypes();

   std::string recoverRangeDescriptionForIntVariables(const std::shared_ptr<std::string> range_description, int& low, int& high, const std::string& member) const;

   bool isFunctionStyle(const std::string& string, const int pos, const std::string& function_name);

   VariablesWithKinds kreateKratosFilesAndCheckFormula(const std::string& target_path_for_generated_code) const;
   void createResultFiles(const std::string& target_path_for_generated_code, const MCSpecification& ltl_specs, const std::string& path_to_envmodel_template, const std::string& path_to_envmodel_file);

   void addEnum(const std::string& enum_name, const std::string& chunk, const std::string& namespace_name);
   void addStruct(const std::string& struct_name, const std::string& chunk, const std::string& namespace_name, const std::vector<std::string>& father_classes);
   std::string addFunction(const std::string& function_name, const std::string& return_type, const std::vector<std::string>& signature, const std::string& chunk, const std::string& namespace_name);

   bool gatekeeper(const ChunkTypeEnum type, const std::string& inner, const std::deque<std::string>& prefix) const;
   bool checkForUndeclaredVariables(const MathStructPtr m) const;

   std::string localVarName(const std::string& function_name, const std::string& raw_var_name) const;
   std::string isLocalVarName(const std::string& qualified_var_name) const; /// Returns "" if not local var name.
   std::string localVarNameActualCall(const std::string& function_name, const std::string& qualified_var_name);

   /// \brief Looks for arrays in a struct or a function signature*, and performs a conversion:
   /// struct X {
   ///   std::array<std::array<int, 3>, 2> a2 { { { {1, 2, 3} }, { { 4, 5, 6} } } }; // Also possible: int[2][3] a2 {...};
   /// };
   /// ===>
   /// struct X {
   ///   int a2.1.2 = 6;
   ///   int a2.0.2 = 3;
   ///   int a2.1.1 = 5;
   ///   int a2.0.1 = 2;
   ///   int a2.1.0 = 4;
   ///   int a2.0.0 = 1;
   /// };
   /// An optional TAL/AKA info at the end of the array declaration is copied to each of the newly created variables.
   /// * Surprisingly (or not so surprisingly, probably, if you know C++), a function signature (without return type) 
   ///   is, syntactically, the same thing as a struct (without member functions) if:
   ///   - the delimiter ";" is replaced by "," and
   ///   - the delimiter is avoided after the last item.
   std::string preprocessArraysInStructOrSignature(
      const std::string& chunk,
      const char begin_bracket,
      const char end_bracket,
      const std::string& delimiter,
      const bool insert_delimiter_after_last_item,
      const std::string& chunk_name) const;

   /// \brief Like the string-based version, only accepts and returns a list of tokens.
   std::vector<std::string> preprocessArraysInStructOrSignature(
      const std::vector<std::string>& tokens,
      const char begin_bracket,
      const char end_bracket,
      const std::string& delimiter,
      const bool insert_delimiter_after_last_item,
      const std::string& chunk_name) const;

   void populateDatatypesToLookAt(std::map<std::string, CppFunctionSignature>& tokens_to_look_at) const;

   void replaceReturnWithAssignment(const TermPtr enc_function, const std::string& set_var_name);

   /// Embed a function f into a function g at the position f is called from g. E.g.:
   /// f(var) { return var + 1; }
   /// g() { ... y = f(x); ... }
   /// ==>
   /// g() { ... y = x + 1; ... }
   /// Note that it has to be valid that:
   /// - the function f has no sideeffects    (function_call_is_isolated_in_line = ?)    OR
   /// - the function is isolated in its line (function_call_is_isolated_in_line = true) OR
   /// - both                                 (function_call_is_isolated_in_line = true).
   void inlineFunction(
      std::string& result,
      const int index_begin,
      const std::string& function_name,
      const vfm::CppFunctionSignature& sig,
      const int index_begin_orig,
      const bool function_call_is_isolated_in_line);

   std::string preprocessCppFunctionInlineFunctionsAndConstructors(const std::string& function_name, const std::string& function);
   std::string preprocessCppFunctionComplexDatatypes(const std::string& function_name, const std::string& function); // Assuming preprocessCppFunctionInlineFunctionsAndConstructors has been called before, so no inline constructors of datatypes exist anymore.

   /// Removes "..safestring" from varnames and inserts the info in global_primitives_tal_range_descriptions_.
   void processInFunctionTalDescriptions(const TermPtr formula);

   void collectDatatypesInFunction(const vfm::TermPtr& formula);
   void postprocessSemanticStuff(const vfm::TermPtr& formula, const std::string& main_function_chunk);
   void removeUneffectiveLines(const vfm::TermPtr& formula);
   TermPtr flattenNestedArrayAccessSafe(const vfm::TermPtr formula);
   TermPtr removeUneffectiveLinesSafe(const vfm::TermPtr formula);
   std::string getAKAFromHeap(const MathStructPtr new_var_name_part_fmla);
   void collectRemainingDatatypesAndScopeVariables(const TermPtr formula);

   std::shared_ptr<Term> parseFunctionFromCpp(const std::string& name, const std::string& function);
   void extractFunctionSignature(const std::string& function_name, const std::string& return_type, const std::vector<std::string>& tokens);
   CppType findType(const std::string& name, const std::vector<std::shared_ptr<CppTypeStruct>> in_types) const;
   void processTypeForVariableName(const std::string& n);
   void processTypesForVariablesInFormula(const std::shared_ptr<Term> formula);
   bool processTypeForVariableName(const std::string& name, const CppType type, const std::vector<std::string>& qualifiers);
   CppType getTypeOf(const std::string& var_name);
   void createGrammarTreesForDebugging(const earley::Grammar& grammar, const std::vector<std::string>& whole_tokens) const;
   void performCodeInjection(const std::string& program) const;

   void performTalOrAkaConversion(std::string& program, const VariableSpecifierMode mode) const;

   void collectLTLSpecs(const std::string& program);
   void collectCTLSpecs(const std::string& program);
   void collectKratosSpecs(const std::string& program);
   void collectInitConstraints(const std::string& program);
   void collectInvariants(const std::string& program);
   void parseOptions(const std::string& program);
   void processUsingKeyword(std::string& code_without_comments);
   void insertTypeOfVariable(const std::string& var_name, const CppTypeWithQualifiers& type, const bool force_override = false);

   static std::string toStringFullType(const CppFullTypeWithName& type);
   static bool containsConstQualifier(const CppFullTypeWithName& type, const bool include_paramconst = false);
   static bool containsConstQualifier(const CppTypeWithQualifiers& type, const bool include_paramconst = false);

   void renameLocalVariablesAndCreateInitsForComplexDatatypes(
      const std::string& function_name, const std::vector<std::pair<std::string, CppType>>& local_variables_raw_names_with_type, std::vector<std::string>& tokens2);

   std::string serializeNuSMV(
      const MCTranslationTypeEnum translation_type,
      const MCSpecification& MC_specs,
      const std::set<std::string> blacklist);

   /// Looks if the function "f(...)" starting at begin_pos_of_internal_function is
   /// isolated on its line, i.e.,
   ///   - "f(...);" or
   ///   - "* = f(...);"
   /// This is done in a purely syntactic way, which might be instable if used in
   /// somewhat different contexts.
   bool functionIsIsolatedOnLine(const std::string& enclosing_function, const int begin_pos_of_internal_function) const;

   /// Assumes that a function call starts at dummy_pos. Finds and returns tokens beginning there and ending at the next ";" or "}". 
   /// Finds also the _token_ position of the closing bracket marking the end of that call and stores it in dummy_pos.
   /// Note that dummy_pos as input denotes a position in the string, but as ouput means the index of the closing bracket token.
   std::vector<std::string> findTokensAfterFunctionCall(
      const std::string& enclosing_function,
      int& dummy_pos) const;
   
   std::vector<std::string> processTemplateOptions(const std::string& template_file_path);

   std::string getFullPathToStoreCachedVersionIn(
      std::string& generated_name,
      const std::string& template_file_path,
      const std::string& generated_filepath,
      const std::vector<std::string>& template_options
   ) const;

   std::string generateEnvModel(
      const std::string& template_file_path, 
      const std::string& generated_filepath,
      const std::vector<std::string>& template_options);

   bool hasCppFunctionSideeffects(const std::string& function_name) const;
   
   void addConstraint(const std::string& constraint_raw, std::vector<TermPtr>& constraint_list, const std::shared_ptr<FormulaParser> parser_raw = nullptr);
   bool checkConstraintsForConsistency(const std::map<std::string, CppTypeWithQualifiers>& types_of_variables) const;

   inline bool isCurrentlyDoingNativeEnvModelGeneration() const
   {
      return use_or_generate_native_smv_env_model_ && in_envmodel_mode_;
   }

   std::string findSomePossibleValueFor(const std::string& inner, const std::string& var_name_right = "");
   void inferType(const std::string& var_name_to_infer_type_for, const std::string& var_name_with_temporarrays_replaced);
   void flattenNestedArrayAccess(const TermPtr formula); /// ...x[y[z]]... ==> temp = y[z]; ...x[tenp]...
   void processVariableAccessOnce(const TermPtr formula, const float node_count, float& current_count, int& old_percent, const std::string& message);
   void processVariableArrayAccess(const TermPtr formula);
   void handleGlobalAKAs(const MathStructPtr formula);
   //void translateK2IntoSMVCode(const std::string& kratos_file_path, const std::string& origin_file, const std::string& target_file) const;
   void generateMainFileForNativeEnvModel(
      const std::string& generated_folder,
      const std::string& planner_filename,
      const std::string& env_model_template_filename,
      const VariablesWithKinds& fsm_controlled_variables,
      const std::string& path_to_envmodel_file) const;

   std::map<std::string, std::string> getAllVariablesExceptImmutableEnumAndBoolTypes() const;

   /// <summary>
   /// Applies a function to all MC formulas including the compound structures. The main function is always included, other elements are optional.
   /// </summary>
   /// <param name="f">The function to apply.</param>
   /// <param name="include_constraints">Iff the constraints are included.</param>
   /// <param name="include_all_functions">Iff all functions are included, not only the main function.</param>
   void applyToAllMCItems(const std::function<void(std::shared_ptr<MathStruct>)>& f, const bool include_constraints, const bool include_all_functions = false);

   std::string serializeKratos(
      const TermPtr main_formula,
      const std::string& entry_function_name,
      const std::map<std::string, std::string>& enum_values,
      const std::vector<std::string>& variables_ordered,
      const std::set<std::string> blacklist,
      const bool native_env_model_wf,
      const std::map<std::string, std::string>& all_variables_except_immutable_enum_and_bool_with_type,
      VariablesWithKinds& fsm_controlled) const;

   /// <summary>
   /// Prints most members of the CppParser class.
   /// </summary>
   /// <returns>A string containing almost all members of the CppParser class.</returns>
   std::string toStringCppParser() const;
   void dumpCurrentState(const std::string& path);

   void addStateVariablesToTAL(fsm::FSMs& fsm);
   void addPlainEnumVariable(const std::string& enum_name, const std::shared_ptr<std::map<float, std::string>> values);

   std::map<std::string, int> function_call_counters_{};
   std::string main_function_name_{};
   std::string main_namespace_name_{};
   std::map<std::string, std::string> using_mapping_{};
   std::map<std::string, std::shared_ptr<Term>> function_definitions_{};
   std::map<std::string, CppFunctionSignature> function_signatures_{}; // { Return type, [ parameters ] }
   std::map<std::string, CppTypeWithQualifiers> types_of_variables_{};
   std::vector<std::string> variables_ordering_{};
   std::vector<CppType> data_types_{};
   std::set<std::string> ignore_tokens_for_functions_{ "const", "&" };
   
   std::vector<TermPtr> list_of_nusmv_constraints_{};

   std::map<ChunkTypeEnum, earley::Grammar> grammars_for_chunks_{};

   std::shared_ptr<mc::TypeAbstractionLayer> type_abstraction_layer_{};
   std::string command_line_argument_{};

   std::map<std::string, std::string> global_primitives_tal_range_descriptions_{}; // Of all primitive global, function-local,...
   std::map<std::string, std::string> global_primitives_aka_descriptions_{};       // ...or function-parameter variables,
   std::map<std::string, std::string> global_primitives_init_values_{};            // the TAL, AKA or (except in the latter case) INIT values.
   // Note: TAL and AKA are stored as ".." + safeString(actual_tal) / "..." + safeString(actual_aka), resp. Init as plain string.

   //std::map<std::string, std::string> 

   std::map<std::string, std::string> function_aka_from_new_to_old_{}; // From new to old var name, actually replaced in a function.

   bool has_envmodel_{};
   bool in_envmodel_mode_{};
   bool use_or_generate_native_smv_env_model_{};

   std::shared_ptr<mc::smv::Module> smv_module_{};

   int temp_variable_counter_{ 0 };

   MCSpecification mc_specs_{};

   std::shared_ptr<FormulaParser> ctl_parser_{}; // TODO: Don't think we need these anymore.
   std::shared_ptr<FormulaParser> ltl_parser_{}; // TODO: Don't think we need these anymore.

   std::set<std::string> all_fsm_controlled_variables_{};   // TODO: Double code from FSM.
   std::set<std::string> all_external_variables_{};         // TODO: Double code from FSM.
   std::set<std::string> all_constants_to_model_checker_{}; // TODO: Double code from FSM.

   std::shared_ptr<DataPack> data_{};
   std::shared_ptr<FormulaParser> parser_{};

   mutable std::vector<TermPtr> all_initial_state_constraints_{};
   mutable std::vector<TermPtr> all_invariants_{};
   mutable std::vector<TermPtr> all_kratos_constraints_{};

   mutable bool is_ltl_mode_{ false }; // ...or else invar mode.

   mutable Options options_{};
};

extern "C" bool performFSMCodeGeneration(
   const char* path_to_file_list_file_code, 
   const char* path_to_file_list_file_env_model, 
   const char* target_path_for_generated_code,
   const char* command_line_argument);

void insertIfNotAlreadyThere(
   std::vector<std::pair<std::string, vfm::CppType>>&local_variables_raw_names_with_type, 
   const std::string &insert_name, 
   const CppType type);

} // vfm
