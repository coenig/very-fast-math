//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"
#include "meta_rule.h"
#include "term_compound_operator.h"


namespace vfm {

enum class HasSideeffectsEnum {
   yes,
   no,
   unknown,
   running
};

/// Compound terms have no fixed structure but can be constructed by
/// using other (possibly also compound) terms as building blocks. For this,
/// a compound Term expects in its constructor another Term called 
/// "compound_structure", that defines how the compound Term is built.
/// The TermMetaCompound type is used as denoter of Term variables given by the
/// operands of this TermCompound. All terms can be used with their usual meaning
/// as building blocks. 
/// 
/// Examples: 
/// The "square" function f(x) == x * x is defined as:
///    *COMPOUND_STRUCTURE* = "p_(0) * p_(0)"
/// The TermSquare has one operand which is referenced by the MetaTerm p_(0) in the
/// compound_structure. The built-in "not" function is formulated as "p_(0) == 0",
/// and the approximately equal function, including a variable for the maximally
/// acceptable difference between the operands is: "abs(p_(0) - p_(1)) <= const_approx".
/// Compound structures can be easily created from a string *formula* such as the ones
/// above by using:
///    MathStruct::parseMathStruct(*formula*)->toTermIfApplicable()
/// 
/// Assembly: As long as all terms in a compound term can be JIT-compiled into
///           assembly (see limitations documented in math_struct.h), the compound 
///           term itself can also be JIT-compiled.
class TermCompound final : public Term
{
public:
   const static OperatorStructure term_compound_my_struct_;

   /// Note that the eval function and the createSubAssembly function
   /// are implemented using completely different algorithms.
   /// This is due to the former approach not being easily transferrable
   /// to the latter. As a positive side effect, the two functions
   /// can be used in tests to control each other, both yielding the same
   /// result indicating a strong assurance that the calculation is correct.
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif

   bool isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas = nullptr,
      const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited = nullptr) override;

   OperatorStructure getOpStruct() const override;
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;

   std::shared_ptr<Term> getCompoundStructure() const;
   void setCompoundStructure(std::shared_ptr<Term> new_compound_structure);
   std::shared_ptr<TermCompoundOperator> getCompoundOperator() const;
   void setCompoundOperator(std::shared_ptr<Term> new_compound_operator);

   bool isTermCompound() const override;
   bool isTermIf() const override;
   bool isTermIfelse() const override;
   bool isTermSequence() const override;
   bool isTermNot() const override;
   bool isTermBool() const override;
   bool hasBooleanResult() const override;
   bool isTermSetVarOrAssignment() const override;
   bool isTermGetVar() const override;

   /// \brief Removes the nested TermCompound from this formula and replaces
   /// it by the actual code of the underlying compound structure. Usually
   /// MathStruct::flattenFormula should be used instead of this method.
   ///
   /// Note 1: There are hard-coded exceptions that are not flattened. These are:
   /// * not
   /// * if
   /// * ifelse
   /// * >>
   /// In future these are supposed to be replaced by a more general exceptions
   /// mechanism.
   /// 
   /// Note 2: Doesn't work for recursive compounds.
   /// Note 3: Crashes if the compound is on the top level of a formula.
   /// Note 4: Changes the structure of the formula pointed to by this.
   ///
   /// In conclusion: Don't ever use this function directly but better call
   ///                MathStruct::flattenFormula which resolves notes 3 and 4.
   void getFlatFormula();

   std::string getNuSMVOperator() const override;
   std::string getK2Operator() const override;

   std::string serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const override;

   std::shared_ptr<TermCompound> toTermCompoundIfApplicable() override;
   std::shared_ptr<TermSetVar> toSetVarIfApplicable() override; /// This returns a new TermSetVar which is not part of the original formula.

   std::string printComplete(const std::string& buffer = "", const std::shared_ptr<std::vector<std::shared_ptr<const MathStruct>>> visited = nullptr, const bool print_root = true) const override;
   std::string generateGraphviz(
      const int spawn_children_threshold = MAX_STRING_LENGTH_TO_DISPLAY_AS_TEXT_ONLY,
      const bool go_into_compounds = true,
      const std::shared_ptr<std::map<std::shared_ptr<MathStruct>, std::string>> follow_fathers = nullptr,
      const std::string cmp_counter = "0",
      const bool root = true,
      const std::string& node_name = "r",
      const std::shared_ptr<std::map<std::shared_ptr<Term>, std::string>> visited = nullptr,
      const std::shared_ptr<std::string> only_compound_edges_to = nullptr) override;
   bool hasSideeffectsThis() const override;

   std::vector<std::string> generatePostfixVector(const bool nestDownToCompoundStructures = false) override;

   void applyToMeAndMyChildren(
      const std::function<void(MathStructPtr)>& f,
      const TraverseCompoundsType go_into_compounds = TraverseCompoundsType::avoid_compound_structures,
      const std::shared_ptr<bool> trigger_abandon_children_once = nullptr, // Set these triggers to true from within a lambda expression
      const std::shared_ptr<bool> trigger_break = nullptr,                 // to direct the control flow in the respective way.
      const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> blacklist = nullptr,
      const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> visited = nullptr,
      const std::function<bool(MathStructPtr)>& reverse_ordering_of_children = [](const MathStructPtr) { return false;  }) override;

   std::string serialize() const;

   void serialize(
      std::stringstream& os,
      const std::shared_ptr<MathStruct> highlight,
      const SerializationStyle style,
      const SerializationSpecial special,
      const int indent,
      const int indent_step,
      const int line_len,
      const std::shared_ptr<DataPack> data,
      std::set<std::shared_ptr<const MathStruct>>& visited) const override;

   std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const override;

   std::string getOptorOnCompoundLevel() const override;

   static std::string privateVarName(const std::string& var_name, const int var_num);
   static std::string basePart(const std::string& var_name);
   static std::map<std::string, std::pair<OperatorStructure, TLType>> getNuSMVOperators();

   static std::shared_ptr<TermCompound> compoundFactory(
      const std::vector<std::shared_ptr<Term>>& opds,
      const std::shared_ptr<Term> compound_structure,
      const OperatorStructure& operator_structure);

private:
   static std::map<std::string, std::pair<OperatorStructure, TLType>> NUSMV_OPERATORS;
   //std::shared_ptr<int> first_free_private_var_num_ = nullptr; // Keep this for future switch to non-static version of first_free_private_var_num_ variable.

   TermCompound(
      const std::vector<std::shared_ptr<Term>>& opds,
      const std::shared_ptr<Term> compound_structure,
      const OperatorStructure& operator_structure);

   void renamePrivateVars();

   void registerPrivateVar(
      const std::string& var_name,
      const std::shared_ptr<std::map<std::string, int>>& private_vars_map);

   //void registerPrivateVarsCounter(); // Keep this for future switch to non-static version of first_free_private_var_num_ variable.

   mutable HasSideeffectsEnum has_sideeffects_{ HasSideeffectsEnum::unknown };
   std::shared_ptr<std::vector<std::string>> private_var_names_{};
};

} // vfm