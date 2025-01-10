//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

// #define COMPOUND_DEBUG // (Un-)comment to add/remove debug output.
// #define PRINT_ADDRESSES_IN_GRAPHVIZ // (Un-)comment to add/remove addresses from graph view.

#include "term_compound.h"

using namespace vfm;

std::map<std::string, std::pair<OperatorStructure, TLType>> TermCompound::NUSMV_OPERATORS;

const OperatorStructure TermCompound::term_compound_my_struct_(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_COMPOUND, PRIO_COMPOUND, SYMB_COMPOUND, false, false);

OperatorStructure TermCompound::getOpStruct() const
{
   return term_compound_my_struct_;
}

TermCompound::TermCompound(
   const std::vector<std::shared_ptr<Term>>& opds,
   const std::shared_ptr<Term> compound_structure,
   const OperatorStructure& operator_structure) :
   Term({ std::make_shared<TermCompoundOperator>(opds, operator_structure), compound_structure }),
   private_var_names_(std::make_shared<std::vector<std::string>>())
{}

float TermCompound::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

#if defined(COMPOUND_DEBUG)
   std::cout << "\n(c) ";
   varVals->printMetaStack();
   std::cout << std::endl << "(c) Evaluating: " << std::endl << printComplete() << std::endl;
   std::cout << std::endl << std::endl << "DATA" << std::endl << *varVals;
#endif

   int recursion_level_id_before = 0;
   if (!private_var_names_->empty()) {
      recursion_level_id_before = varVals->getVarRecursiveID(private_var_names_->front());
      varVals->setVarsRecursiveIDs(*private_var_names_, varVals->getRecursionVarDepthID());
      varVals->incrementRecursionVarDepthID();
   }

   varVals->addSingleValsIfUndeclared(*private_var_names_, true);
   varVals->addOrSetSingleVals(*private_var_names_, 0);

   const auto temp_vec = std::make_shared<std::vector<std::shared_ptr<Term>>>();
   for (const auto& term : getCompoundOperator()->getOperands()) {
      temp_vec->push_back(term);
   }

   varVals->pushValuesForMeta(temp_vec);
   auto res = getCompoundStructure()->eval(varVals, parser, suppress_sideeffects);
   varVals->popValuesForMeta();

   varVals->setVarsRecursiveIDs(*private_var_names_, recursion_level_id_before);

   if (!private_var_names_->empty()) {
      varVals->decrementRecursionVarDepthID();
   }

#if defined(COMPOUND_DEBUG)
   std::cout << "\n(c) ";
   varVals->printMetaStack();
#endif

   return res;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermCompound::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   if (isRecursive()) {
      return nullptr;
   }

   d->addSingleValsIfUndeclared(*private_var_names_);

   auto ops{ getCompoundOperator()->getOperands() };
   auto cs_copy{ getCompoundStructure()->copy() };

   if (cs_copy->isMetaCompound()) {
      cs_copy = _plus(cs_copy, _val(0)); // As meta cannot be assemblyfied, there has to be a non-meta term on top-level.
   }

   auto blacklist{ std::make_shared<std::vector<std::shared_ptr<MathStruct>>>() };
   auto break_trigger{ std::make_shared<bool>(false) };
   bool non_const_meta_detected{ false };

   cs_copy->applyToMeAndMyChildren(
      [ops, d, blacklist, break_trigger, &non_const_meta_detected](std::shared_ptr<MathStruct> term)
      {
         for (size_t i = 0; i < term->getOperands().size(); i++) {
            if (term->getOperands()[i]->isMetaCompound()) {
               const auto meta_argument{ term->getOperands()[i]->getOperands()[0] };

               if (!meta_argument->isOverallConstant()) {
                  non_const_meta_detected = true;
                  *break_trigger = true;
               }

               term->getOperands()[i] = ops[meta_argument->eval(d)]->copy();
               term->setChildrensFathers(false, false); // Not functionally required, see below; useful for debugging, though (and fast).
               blacklist->push_back(term->getOperands()[i]);
            }
         }
      },
      TraverseCompoundsType::avoid_compound_structures, nullptr, break_trigger, blacklist
   );

   cs_copy->setChildrensFathers();
   auto ass_func{ non_const_meta_detected
      ? nullptr // Compounds with non-const metas (like "p_(i)") currently not compilable into assembly.
      : cs_copy->createSubAssembly(cc, d, p) };

   if (!ass_func) {
      getCompoundStructure()->createAssembly(d, p);
   }

   return ass_func;
}
#endif

bool vfm::TermCompound::isOverallConstant(
   const std::shared_ptr<std::vector<int>> non_const_metas,
   const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   auto visited_real{ visited };

   if (!visited_real) {
      visited_real = std::make_shared<std::set<std::shared_ptr<MathStruct>>>();
   }

   auto ptr_to_this{ toTermIfApplicable() };

   if (std::find(visited_real->begin(), visited_real->end(), ptr_to_this) != visited_real->end()) {
      return false;
   }

   visited_real->insert(ptr_to_this);

   std::shared_ptr<std::vector<int>> non_const_metas_next{ std::make_shared<std::vector<int>>() };

   for (int i = 0; i < getCompoundOperator()->getOperands().size(); i++) {
      auto term = getCompoundOperator()->getOperands()[i];
      if (!term->isOverallConstant(non_const_metas, visited_real)) {
         non_const_metas_next->push_back(i);
      }
   }

   //if (non_const_metas_next->empty()) {
   //   return true;
   //}
   //else {
   return getCompoundStructure()->isOverallConstant(non_const_metas_next, visited_real);
   //}
}

std::shared_ptr<Term> vfm::TermCompound::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied)
{
   auto already_copied_real{ already_copied };
   if (!already_copied_real) {
      already_copied_real = std::make_shared<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>>();
   }

   auto ptr_to_this{ toTermIfApplicable() };
   auto copied_element{ already_copied_real->find(ptr_to_this) };

   if (copied_element != already_copied_real->end()) {
      return copied_element->second;
   }

   auto emptyVec{ std::vector<std::shared_ptr<Term>>() };
   std::shared_ptr<TermCompound> mainCopy{ compoundFactory(emptyVec, nullptr, getCompoundOperator()->getOpStruct()) };
   already_copied_real->insert(std::pair<std::shared_ptr<Term>, std::shared_ptr<Term>>(ptr_to_this, mainCopy));

   std::vector<std::shared_ptr<Term>> opc{};
   auto my_compound_opstruct_operators{ getCompoundOperator()->getOperands() };
   opc.reserve(my_compound_opstruct_operators.size());

   for (size_t i = 0; i < my_compound_opstruct_operators.size(); ++i) {
      opc.push_back(my_compound_opstruct_operators[i]->copy(already_copied_real));
   }

   std::shared_ptr<Term> cp{ getCompoundStructure()->copy(already_copied_real) };

   mainCopy->setCompoundStructure(cp);
   mainCopy->getCompoundOperator()->setOpnds(opc);
   mainCopy->setChildrensFathers();

   mainCopy->private_var_names_ = std::make_shared<std::vector<std::string>>();
   for (const auto& var : *private_var_names_) {
      mainCopy->private_var_names_->push_back(var);
   }

   mainCopy->has_sideeffects_ = has_sideeffects_;

   return mainCopy;
}

std::shared_ptr<TermCompoundOperator> vfm::TermCompound::getCompoundOperator() const
{
   return std::dynamic_pointer_cast<TermCompoundOperator>(getOperandsConst()[0]);
}

std::shared_ptr<Term> vfm::TermCompound::getCompoundStructure() const
{
   return getOperandsConst()[1];
}

void TermCompound::setCompoundStructure(const std::shared_ptr<Term> new_compound_structure)
{
   auto& cs{ getOperands()[1] };

   if (!cs || !cs->getFather()) {
      cs = new_compound_structure;
   }
   else {
      cs->replace(new_compound_structure);
   }

   cs->addCompoundStructureReference(this);

   renamePrivateVars();
}

void vfm::TermCompound::setCompoundOperator(std::shared_ptr<Term> new_compound_operator)
{
   getCompoundOperator()->replace(new_compound_operator);
}

void TermCompound::getFlatFormula()
{
   //if (getCompoundOperator()->getOptor() == SYMB_NOT 
   //   || getCompoundOperator()->getOptor() == SYMB_SEQUENCE 
   //   || getCompoundOperator()->getOptor() == "if" 
   //   || getCompoundOperator()->getOptor() == "ifelse") {
   //   return;
   //}

   auto cs{ getCompoundStructure()->copy() };
   std::vector<std::shared_ptr<MathStruct>> ops{};

   for (const auto& opnd : getCompoundOperator()->getOperands()) {
      ops.push_back(opnd->copy());
   }

   if (cs->isMetaCompound()) {
      cs = ops[cs->getOperands()[0]->constEval()]->toTermIfApplicable();
   }
   else {
      cs->applyToMeAndMyChildren([ops](const std::shared_ptr<MathStruct> m) {
         if (m->isMetaCompound()) {
            auto replace = ops[m->getOperands()[0]->constEval()];
            m->getFather()->replaceOperand(m->toMetaCompoundIfApplicable(), replace->toTermIfApplicable());
         }
         });
   }

   getFather()->replaceOperand(toTermIfApplicable(), cs);
}

std::string TermCompound::getNuSMVOperator() const
{
   if (isTermNot()) {
      return "!";
   }
   else if (isTermBool() || isTermIdentity()) {
      return "";
   }
   else if (getCompoundOperator()->getOptor() == "=>") {
      return "->";
   }
   else if (getCompoundOperator()->getOptor() == "<=>") {
      return "<->";
   }
   else if (getCompoundOperator()->isTermIf() || getCompoundOperator()->isTermIfelse()) {
      return "#ERROR-IF-IS-NOT-SUPPORTED-AS-NUSMV-OPERATOR";
   }

   return MathStruct::getNuSMVOperator();
}

std::string vfm::TermCompound::getK2Operator() const
{
   if (getCompoundOperator()->getOptor() == SYMB_SET_VAR_A) {
      return "assign";
   }

   return MathStruct::getK2Operator();
}

std::string vfm::TermCompound::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   std::string s{};
   auto dummy_this{ const_cast<TermCompound*>(this) };

   if (getCompoundOperator()->getOptor() == SYMB_IF_ELSE || getCompoundOperator()->getOptor() == SYMB_IF) {
      const int my_label{ label_counter++ };

      s += StaticHelper::makeString(OPENING_BRACKET)
         + "condjump " + _not_plain(dummy_this->getCompoundOperator()->child0())->serializeK2(enum_values, additional_functions, label_counter) + " "
         + StaticHelper::makeString(OPENING_BRACKET)
         + "label else" + std::to_string(my_label)
         + StaticHelper::makeString(CLOSING_BRACKET)
         + StaticHelper::makeString(CLOSING_BRACKET) + "\n";

      s += dummy_this->getCompoundOperator()->child1()->serializeK2(enum_values, additional_functions, label_counter) + "\n";
      s += StaticHelper::makeString(OPENING_BRACKET)
         + "jump "
         + StaticHelper::makeString(OPENING_BRACKET)
         + "label endif" + std::to_string(my_label)
         + StaticHelper::makeString(CLOSING_BRACKET)
         + StaticHelper::makeString(CLOSING_BRACKET) + "\n";

      s += StaticHelper::makeString(OPENING_BRACKET)
         + "label else" + std::to_string(my_label)
         + StaticHelper::makeString(CLOSING_BRACKET) + "\n";

      if (getCompoundOperator()->getOptor() == SYMB_IF_ELSE) {
         s += dummy_this->getCompoundOperator()->child2()->serializeK2(enum_values, additional_functions, label_counter) + "\n";
      }

      s += StaticHelper::makeString(OPENING_BRACKET)
         + "label endif" + std::to_string(my_label)
         + StaticHelper::makeString(CLOSING_BRACKET);
   }
   else if (getCompoundOperator()->getOptor() == SYMB_WHILE) {
      const int my_label = label_counter++;

      s += StaticHelper::makeString(OPENING_BRACKET)
         + "label while" + std::to_string(my_label)
         + StaticHelper::makeString(CLOSING_BRACKET) + "\n";

      s += dummy_this->getCompoundOperator()->child1()->serializeK2(enum_values, additional_functions, label_counter) + "\n";

      s += dummy_this->getCompoundOperator()->child0()->isAlwaysTrue()
         ? StaticHelper::makeString(OPENING_BRACKET)
         + "jump "
         + StaticHelper::makeString(OPENING_BRACKET)
         + "label while" + std::to_string(my_label)
         + StaticHelper::makeString(CLOSING_BRACKET)
         + StaticHelper::makeString(CLOSING_BRACKET) + "\n"
         : StaticHelper::makeString(OPENING_BRACKET)
         + "condjump " + dummy_this->getCompoundOperator()->child0()->serializeK2(enum_values, additional_functions, label_counter) + " "
         + StaticHelper::makeString(OPENING_BRACKET)
         + "label while" + std::to_string(my_label)
         + StaticHelper::makeString(CLOSING_BRACKET)
         + StaticHelper::makeString(CLOSING_BRACKET) + "\n";
   }
   else if (getCompoundOperator()->getOptor() == SYMB_SEQUENCE) {
      std::string elements{};
      auto ptr{ shared_from_this() };

      while (ptr->getOptorJumpIntoCompound() == SYMB_SEQUENCE) {
         elements = ptr->child1JumpIntoCompounds()->serializeK2(enum_values, additional_functions, label_counter) + "\n" + elements;
         ptr = ptr->child0JumpIntoCompounds();
      }

      elements = ptr->serializeK2(enum_values, additional_functions, label_counter) + "\n" + elements;

      s = StaticHelper::makeString(OPENING_BRACKET) + "seq\n" + elements + StaticHelper::makeString(CLOSING_BRACKET);
   }
   else if (getCompoundOperator()->getOptor() == SYMB_NEG_ALT) {
      s += StaticHelper::makeString(OPENING_BRACKET) + "neg " + dummy_this->getCompoundOperator()->child0()->serializeK2(enum_values, additional_functions, label_counter) + StaticHelper::makeString(CLOSING_BRACKET);
   }
   else if (getCompoundOperator()->getOptor() == SYMB_NOT) {
      s += StaticHelper::makeString(OPENING_BRACKET) + "not " + dummy_this->getCompoundOperator()->child0()->serializeK2(enum_values, additional_functions, label_counter) + StaticHelper::makeString(CLOSING_BRACKET);
   }
   else if (getCompoundOperator()->getOptor() == SYMB_SET_VAR_A && dummy_this->getCompoundOperator()->child1JumpIntoCompounds()->getOptorJumpIntoCompound() == "rndet") {
      // (call rndet (const 0 int) (const 6 int) veh___619___.a)

      auto first { dummy_this->getCompoundOperator()->child1JumpIntoCompounds()->child0()->child0()->serializeK2(enum_values, additional_functions, label_counter) };
      auto second{ dummy_this->getCompoundOperator()->child1JumpIntoCompounds()->child0()->child1()->serializeK2(enum_values, additional_functions, label_counter) };
      auto third { dummy_this->getCompoundOperator()->child0JumpIntoCompounds()->serializeK2(enum_values, additional_functions, label_counter) };

      s += StaticHelper::makeString(OPENING_BRACKET) + "call rndet " + first + " " + second + " " + third + StaticHelper::makeString(CLOSING_BRACKET);
   }
   else if (getCompoundOperator()->getOptor() == SYMB_SET_VAR_A && dummy_this->getCompoundOperator()->child1()->getOptorJumpIntoCompound() == "ndet") {
      auto choices{ dummy_this->getCompoundOperator()->child1()->getTermsJumpIntoCompounds() };
      std::string function_name{ "#MALFORMED" };

      if (choices.empty()) {
         Failable::getSingleton()->addError("Found zero choices for ndet operator '" + serializeWithinSurroundingFormula(100, 100) + "'.");
      }
      else {
         std::string enum_type = choices.at(0)->toVariableIfApplicable()->getVariableName();

         if (enum_values.count(enum_type)) {
            if (additional_functions.first.count(enum_type)) {
               function_name = additional_functions.first.at(enum_type);
            }
            else {
               std::vector<std::string> choices_vec;
               function_name = "ndet_enum" + std::to_string(additional_functions.first.size()) + "_" + std::to_string(choices.size());

               for (const auto& choice : choices) {
                  choices_vec.push_back("(enum " + enum_values.at(enum_type) + ")");
               }

               additional_functions.second += "\n" + StaticHelper::createKratosNdetFunction(function_name, choices_vec);
               additional_functions.first.insert({ enum_type, function_name });
            }
         }
         else if (choices.size() == 2 && choices.at(0)->toVariableIfApplicable()
            && (choices.at(0)->toVariableIfApplicable()->getVariableName() == "true"
               || choices.at(0)->toVariableIfApplicable()->getVariableName() == "false")) { // It's a bool.
            function_name = "ndet_bool" + std::to_string(choices.size());
         }
         else {                                                                     // Leaves only int (TODO: real not supported yet).
            function_name = "ndet_int" + std::to_string(choices.size());
         }
      }

      s += StaticHelper::makeString(OPENING_BRACKET) + "call " + function_name;

      for (const auto& choice : choices) {
         s += " " + choice->serializeK2(enum_values, additional_functions, label_counter);
      }

      s += " " + dummy_this->getCompoundOperator()->child0()->serializeK2(enum_values, additional_functions, label_counter) + StaticHelper::makeString(CLOSING_BRACKET);
   }
   else if (getCompoundOperator()->getOptor() == "std::abs") { // TODO magic string!
      s = _abs_plain(getCompoundOperator()->child0())->serializeK2(enum_values, additional_functions, label_counter);
   }
   else {
      s = MathStruct::serializeK2(enum_values, additional_functions, label_counter);
   }

   return s;
}

bool TermCompound::isTermSequence() const {
   return getCompoundOperator()->getOptor() == SYMB_SEQUENCE;
}

bool vfm::TermCompound::isTermNot() const
{
   // Meta: p_(0) == 0.
   return getCompoundStructure()->getOptor() == SYMB_EQ
      && (getCompoundStructure()->getOperands()[0]->isAlwaysFalse() && getCompoundStructure()->getOperands()[1]->isMetaCompound()
         || getCompoundStructure()->getOperands()[1]->isAlwaysFalse() && getCompoundStructure()->getOperands()[0]->isMetaCompound());
}

bool vfm::TermCompound::isTermBool() const
{
   // Meta: p_(0) != 0.
   return getCompoundStructure()->getOptor() == SYMB_NEQ
      && (getCompoundStructure()->getOperands()[0]->isAlwaysFalse() && getCompoundStructure()->getOperands()[1]->isMetaCompound()
         || getCompoundStructure()->getOperands()[1]->isAlwaysFalse() && getCompoundStructure()->getOperands()[0]->isMetaCompound());
}

bool vfm::TermCompound::isTermSetVarOrAssignment() const
{
   return SYMB_SET_VAR_A == getCompoundOperator()->getOptor(); // TODO: This doesn't capture custom-defined set operators: Better look at compound structure.
}

bool vfm::TermCompound::isTermGetVar() const
{
   return SYMB_GET_VAR == getCompoundOperator()->getOptor();
}

bool vfm::TermCompound::hasBooleanResult() const
{
   return getCompoundStructure()->hasBooleanResult();
}

bool TermCompound::isTermCompound() const
{
   return true;
}

bool vfm::TermCompound::isTermIf() const
{
   return getCompoundOperator()->getOptor() == SYMB_IF;
}

bool vfm::TermCompound::isTermIfelse() const
{
   return getCompoundOperator()->getOptor() == SYMB_IF_ELSE;
}

std::shared_ptr<TermCompound> TermCompound::toTermCompoundIfApplicable()
{
   return std::dynamic_pointer_cast<TermCompound>(shared_from_this());
}

std::shared_ptr<TermSetVar> vfm::TermCompound::toSetVarIfApplicable()
{
   if (!isTermSetVarOrAssignment()) {
      return MathStruct::toSetVarIfApplicable();
   }

   return std::make_shared<TermSetVar>(getCompoundOperator()->getOperands()[0]->copy(), getCompoundOperator()->getOperands()[1]->copy());
}

std::string TermCompound::printComplete(const std::string& buffer, const std::shared_ptr<std::vector<std::shared_ptr<const MathStruct>>> visited, const bool print_root) const
{
   const auto recDepth = -1; // MathStruct::getRecursionVarDepthID();
   std::string s = getCompoundOperator()->printComplete(buffer, visited, print_root)/* + "\t<DEPTH: " + std::to_string(recDepth) + ">"*/;
   s += "\n" + buffer + INDENTATION_DEBUG + "With Compound (" + getCompoundOperator()->getOptor() + "): " + getCompoundStructure()->printComplete(buffer + "", visited, true);
   return s;
}

std::string vfm::TermCompound::generateGraphviz(
   const int spawn_children_threshold,
   const bool go_into_compounds,
   const std::shared_ptr<std::map<std::shared_ptr<MathStruct>, std::string>> follow_fathers,
   const std::string cmp_counter,
   const bool root,
   const std::string& node_name,
   const std::shared_ptr<std::map<std::shared_ptr<Term>, std::string>> visited,
   const std::shared_ptr<std::string> only_compound_edges_to)
{
   auto ser_size{ getCompoundStructure()->serialize().size() };
   bool spawn_child_nodes_compound{ ser_size > spawn_children_threshold };

   auto visited_real = visited;
   if (!visited_real) {
      visited_real = std::make_shared<std::map<std::shared_ptr<Term>, std::string>>();
   }

   auto ptr_to_this = getCompoundOperator();
   auto visited_element = visited_real->find(ptr_to_this);

   if (visited_element != visited_real->end()) {
       return "$$$" + visited_element->second;
   }

   visited_real->insert(std::pair<std::shared_ptr<Term>, std::string>(ptr_to_this, node_name));
   std::string s = getCompoundOperator()->generateGraphviz(spawn_children_threshold, go_into_compounds, follow_fathers, cmp_counter + "0", false, node_name, visited_real, only_compound_edges_to);

   if (go_into_compounds) {
      if (!private_var_names_->empty()) {
         std::string private_vars = "";
         std::string private_vars_node = node_name + "_pv" + cmp_counter;

         for (const auto& var : *private_var_names_) {
            private_vars += var + ";";
         }
         s += private_vars_node + "[label = \"" + private_vars + "\", shape=none];\n";
         s += node_name + "->" + private_vars_node + " [arrowhead=none, style=dashed];\n";
      }

      std::string cmp_root_name = node_name + "CMP" + cmp_counter + "1";
      std::string cmp_options = std::string("[arrowhead = \"dot\" penwidth=3 color=blue label=\"") + (only_compound_edges_to ? getCompoundOperator()->getOptor() : "") + "\"]";
      std::string serialized = getCompoundStructure()->generateGraphviz(spawn_children_threshold, go_into_compounds, follow_fathers, cmp_counter + "1", false, cmp_root_name, visited_real);

#if defined(PRINT_ADDRESSES_IN_GRAPHVIZ) // Print out address of this compound structure.
      std::string addr_node = cmp_root_name + "_adr";
      s += addr_node + "[label = \"" + StaticHelper::convertPtrAddressToString(this) + "\", shape=cds];\n";
      s += node_name + "->" + addr_node + " [arrowhead=none, style=dotted];\n";
#endif

      if (!serialized.empty() && serialized[0] == '$') {
         s += node_name + "->" + serialized.substr(3) + " " + cmp_options + ";\n";
      }
      else {
         if (spawn_child_nodes_compound) {
            std::string cluster_name = "cluster_compound_" + node_name;
            s += "subgraph " + cluster_name + "{\n" + "label=\"" + getCompoundStructure()->serialize() + "\";\ncolor=blue;\n" + serialized + "}";
         }
         else {
            s += serialized;
         }

         s += node_name + "->" + cmp_root_name + " " + cmp_options + ";\n";
      }
   }

   if (root) {
      s = "digraph G {\n"
         + std::string("root_title [label=\"") + StaticHelper::shortenToMaxSize(MathStruct::serialize(), 250) + "\" color=white];\n"
         + s + "}";
   }

   return s;
}

bool TermCompound::hasSideeffectsThis() const
{
   if (isTermNot()) { // By pure compound structure analysis, NOT would be assumed to possibly have sideeffects, as in "!(a=b)".
      return getCompoundOperator()->child0()->hasSideeffectsThis();
   }

   if (has_sideeffects_ == HasSideeffectsEnum::running) {
      return false; // Recursion, avoid infinite cycle.
   }

   if (has_sideeffects_ == HasSideeffectsEnum::unknown) {
      has_sideeffects_ = HasSideeffectsEnum::running;
      has_sideeffects_ = getCompoundStructure()->hasSideeffects() ? HasSideeffectsEnum::yes : HasSideeffectsEnum::no;
   }

   return has_sideeffects_ == HasSideeffectsEnum::yes; // If the compound structure has side effects, it's a side effect.
}

std::vector<std::string> TermCompound::generatePostfixVector(const bool nestDownToCompoundStructures)
{
   std::vector<std::string> tokens;
   auto sub_vector1 = MathStruct::generatePostfixVector();
   tokens.insert(tokens.end(), sub_vector1.begin(), sub_vector1.end());

   if (nestDownToCompoundStructures) {
      auto sub_vector2 = getCompoundStructure()->generatePostfixVector();
      tokens.push_back(StaticHelper::makeString(OPENING_MOD_BRACKET));
      tokens.insert(tokens.end(), sub_vector2.begin(), sub_vector2.end());
      tokens.push_back(StaticHelper::makeString(CLOSING_MOD_BRACKET));
   }

   return tokens;
}

void TermCompound::applyToMeAndMyChildren(
   const std::function<void(std::shared_ptr<MathStruct>)>& f,
   const TraverseCompoundsType go_into_compounds,
   const std::shared_ptr<bool> trigger_abandon_children_once,
   const std::shared_ptr<bool> trigger_break,
   const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> blacklist,
   const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> visited,
   const std::function<bool(const std::shared_ptr<MathStruct>)>& reverse_ordering_of_children)
{
   auto ptr_to_this = shared_from_this();

   if (trigger_break && *trigger_break || blacklist && std::find(blacklist->begin(), blacklist->end(), ptr_to_this) != blacklist->end()) {
      return;
   }

   f(shared_from_this());

   if (trigger_abandon_children_once && *trigger_abandon_children_once) {
      *trigger_abandon_children_once = false;
      return;
   }

   getCompoundOperator()->applyToMeAndMyChildren(f, go_into_compounds, trigger_abandon_children_once, trigger_break, blacklist, visited, reverse_ordering_of_children);

   if (go_into_compounds == TraverseCompoundsType::go_into_compound_structures) {
      std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> visited_real{ visited };

      if (!visited_real) {
         visited_real = std::make_shared<std::vector<std::shared_ptr<MathStruct>>>();
      }

      if (std::find(visited_real->begin(), visited_real->end(), ptr_to_this) == visited_real->end()) {
         visited_real->push_back(ptr_to_this);
         getCompoundStructure()->applyToMeAndMyChildren(f, go_into_compounds, trigger_abandon_children_once, trigger_break, blacklist, visited_real), reverse_ordering_of_children;
      }
   }
}

std::string vfm::TermCompound::serialize() const
{
   std::stringstream os{};
   std::set<std::shared_ptr<const MathStruct>> visited{};
   serialize(os, nullptr, SerializationStyle::cpp, SerializationSpecial::none, 0, 3, 200, nullptr, visited);
   return os.str();
}

void TermCompound::serialize(
   std::stringstream& os,
   const std::shared_ptr<MathStruct> highlight,
   const SerializationStyle style,
   const SerializationSpecial special,
   const int indent,
   const int indent_step,
   const int line_len,
   const std::shared_ptr<DataPack> data,
   std::set<std::shared_ptr<const MathStruct>>& visited) const
{
   auto nonconst_this{ const_cast<TermCompound*>(this) };
   TermPtr compound_operator{ getCompoundOperator() };

   if (!compound_operator) {
      compound_operator = _var("NULL");
   }

   const auto array_treatment = [compound_operator, &os, &highlight, &style, &special, &indent, &indent_step, &line_len, &data, &visited]() {
      std::string s{};
      bool first{ true };
      const std::string OPENING_ARRAY_BRACKET{ style == SerializationStyle::nusmv ? "___6" : "[" };
      const std::string CLOSING_ARRAY_BRACKET{ style == SerializationStyle::nusmv ? "9___" : "]" };

      auto current{ compound_operator->thisPtrGoUpToCompound()->toTermIfApplicable() };
      while (current->isTermGetVar() && current->getTermsJumpIntoCompounds()[0]->isTermPlus()) {
         s = current->getTermsJumpIntoCompounds()[0]->getOperands()[1]->serialize(highlight, style, special, indent, indent_step, line_len) + (first ? "" : ", ") + s;
         current = current->getTermsJumpIntoCompounds()[0]->getOperands()[0];
         first = false;
      }

      if (s.empty()) s = "0"; // Special treatment for "get(x)" (there is no "+ 0") ==> "x[0]".

      if (current->isTermVar() || current->getOptorOnCompoundLevel() == SYMB_DOT || current->getOptorOnCompoundLevel() == SYMB_DOT_ALT) { // Handle actual vars, but also treat "x dot y" as "x.y".
         os << current->serialize(highlight, style, special, indent, indent_step, line_len) << OPENING_ARRAY_BRACKET << s << CLOSING_ARRAY_BRACKET;
      }
      else {
         os << SYMB_GET_VAR << "(";
         compound_operator->getOperands()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << ")";
      }
   };

   if (style == SerializationStyle::nusmv && special != SerializationSpecial::enforce_postfix && special != SerializationSpecial::enforce_postfix_and_print_compound_structures) {
      if (isTermIf() || isTermIfelse()) {
         std::shared_ptr<Term> op0{ compound_operator->getOperands()[0] };
         std::shared_ptr<Term> op1{ compound_operator->getOperands()[1] };
         std::shared_ptr<Term> op2{ compound_operator->getOperands().size() > 2 ? compound_operator->getOperands()[2] : _val(0) };
         os << "((";
         op0->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << ") ? (";
         op1->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << ") : (";
         op2->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << "))";
      }
      else if (compound_operator->getOptor() == SYMB_NONDET_FUNCTION_FOR_MODEL_CHECKING) {
         os << "{";

         if (!compound_operator->getOperands().empty()) {
            compound_operator->getOperands()[0]->serialize(os, highlight, style, SerializationSpecial::none, 0, 3, line_len, data, visited);
         }

         for (size_t i = 1; i < compound_operator->getOperands().size(); i++) {
            os << ", ";
            compound_operator->getOperands()[i]->serialize(os, highlight, style, SerializationSpecial::none, 0, 3, line_len, data, visited);
         }

         os << "}";
      }
      else if (compound_operator->getOptor() == SYMB_NONDET_RANGE_FUNCTION_FOR_MODEL_CHECKING) {
         compound_operator->getOperands()[0]->serialize(os, highlight, style, SerializationSpecial::none, 0, 3, line_len, data, visited);
         os << "..";
         compound_operator->getOperands()[1]->serialize(os, highlight, style, SerializationSpecial::none, 0, 3, line_len, data, visited);
      }
      else if (compound_operator->getOptor() == "EU" || getOptor() == "AU") {
         os << compound_operator->getOptor()[0] << "[(";
         compound_operator->getOperands()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << ") " << getOptor()[1] << " (";
         compound_operator->getOperands()[1]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << ")]";
      }
      else if (compound_operator->getOptor() == "_def_") {
         compound_operator->getOperands()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << " := ";
         compound_operator->getOperands()[1]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
      else if (compound_operator->getOptor() == "<=>") {
         compound_operator->getOperands()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << " <-> ";
         compound_operator->getOperands()[1]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
      else if (compound_operator->getOptor() == "=>") {
         compound_operator->getOperands()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << " -> ";
         compound_operator->getOperands()[1]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
      else if (getNuSMVOperators().count(compound_operator->getOptor())) {
         compound_operator->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
      else if (isTermGetVar() && (compound_operator->getTermsJumpIntoCompounds()[0]->isTermPlus() || compound_operator->getTermsJumpIntoCompounds()[0]->isTermVar())) {
         array_treatment();
      }
      else {
         auto cs = MathStruct::flattenFormula(const_cast<TermCompound*>(this)->toTermIfApplicable());
         cs->thisPtrGoIntoCompound()->MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
   }
   else if (compound_operator->getOptor() == SYMB_DOT) {
      compound_operator->getOperands()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      os << ".";
      compound_operator->getOperands()[1]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
   }
   else if ((style == SerializationStyle::cpp || special == SerializationSpecial::enforce_square_array_brackets) 
              && isTermGetVar() && (compound_operator->getTermsJumpIntoCompounds()[0]->isTermPlus() || compound_operator->getTermsJumpIntoCompounds()[0]->isTermVar())) {
      array_treatment();
   }
   else {
      if (special == SerializationSpecial::print_compound_structures || special == SerializationSpecial::enforce_postfix_and_print_compound_structures) {
         MathStruct::serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
      else {
         nonconst_this->getOperands()[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
      }
   }

   visited.insert(shared_from_this());
}

std::set<MetaRule> vfm::TermCompound::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec{ MathStruct::getSimplificationRules(parser) };
   auto mystruct{ getCompoundOperator()->getOpStruct() };

   if (mystruct.op_name == SYMB_SET_VAR_A) {
      vec.insert(MetaRule(
         _set_alt(_vara(), _varb()),
         _set_alt(_vara(), _varb()),
         SIMPLIFICATION_STAGE_MAIN,
         {
            { _vara(), ConditionType::has_no_sideeffects },
            { _varb(), ConditionType::has_no_sideeffects },
            { _varb(), ConditionType::is_any_var_of_second_on_any_left_side_earlier_AND_INSERT },
         }
      )); // a = b
   }
   else if (mystruct.op_name == SYMB_SEQUENCE) {
      vec.insert(MetaRule(_seq(_vara(), _varb()), _varb(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::is_constant/*has_no_sideeffects*/ } })); // TODO: has_no_sideeffects would be good enough, only screws up simplification of nested top-level formula in FSM::createFullStatesFSMFromFormula
      vec.insert(MetaRule(_seq(_seq(_vara(), _varb()), _varc()), _seq(_vara(), _varc()), SIMPLIFICATION_STAGE_MAIN, { { _varb(), ConditionType::has_no_sideeffects } }));
      //vec.insert(MetaRule(_seq(_if(_vard(), _seq(_vara(), _varb())), _vare()), _seq(_if(_vard(), _vara()), _vare()), { { _varb(), ConditionType::has_no_sideeffects } }));
      //vec.insert(MetaRule(_seq(_ifelse(_vard(), _seq(_vara(), _varb()), _varc()), _vare()), _seq(_ifelse(_vard(), _vara(), _varc()), _vare()), { { _varb(), ConditionType::has_no_sideeffects } }));
      //vec.insert(MetaRule(_seq(_ifelse(_vard(), _varc(), _seq(_vara(), _varb())), _vare()), _seq(_ifelse(_vard(), _varc(), _vara()), _vare()), { { _varb(), ConditionType::has_no_sideeffects } }));

      vec.insert(MetaRule( // Needed for: @a = 4; @a = 5.
         _seq(_seq(_optional("e", 0), _set_alt(_vara(), _varb())), _set_alt(_vara(), _varc())),
         _seq(_vare(), _set_alt(_vara(), _varc())),
         SIMPLIFICATION_STAGE_MAIN,
         {
            { _seq(_varc(), _vara()), ConditionType::second_operand_not_contained_in_first_operand },
            { _vara(), ConditionType::has_no_sideeffects },
            { _varb(), ConditionType::has_no_sideeffects },
            { _varc(), ConditionType::has_no_sideeffects },
         }
      )); // ([e;] a = b); a = c ==> ([e;] a = c)

      vec.insert(MetaRule(
         _seq(_seq(_optional("e", 0), _set_alt(_vara(), _varb())), _set_alt(_varc(), _vard())),
         _seq(_seq(_vare(), _set_alt(_varc(), _vard())), _set_alt(_vara(), _varb())),
         SIMPLIFICATION_STAGE_MAIN,
         {
            { _seq(_vara(), _varc()), ConditionType::of_first_two_operands_second_is_alphabetically_above },
            { _seq(_varb(), _varc()), ConditionType::second_operand_not_contained_in_first_operand },
            { _seq(_vard(), _vara()), ConditionType::second_operand_not_contained_in_first_operand },
            { _vara(), ConditionType::has_no_sideeffects },
            { _varb(), ConditionType::has_no_sideeffects },
            { _varc(), ConditionType::has_no_sideeffects },
            { _vard(), ConditionType::has_no_sideeffects },
         }
      )); // ([e;] a = b); c = d => ([e;] c = d); a = b
   }
   else if (mystruct.op_name == SYMB_BOOLIFY) {
      vec.insert(MetaRule(_boolify(_not(_vara())), _not(_vara()), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_boolify(_vara()), _vara(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_boolean_result } }));
   }
   else if (mystruct.op_name == SYMB_NOT) {
      vec.insert(MetaRule(_not(_boolify(_vara())), _not(_vara()), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_not(_eq(_vara(), _varb())), _neq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_not(_neq(_vara(), _varb())), _eq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));

      vec.insert(MetaRule(_not(_sm(_vara(), _varb())), _greq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_not(_gr(_vara(), _varb())), _smeq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_not(_smeq(_vara(), _varb())), _gr(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_not(_greq(_vara(), _varb())), _sm(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_not(_not(_vara())), _boolify(_vara()), SIMPLIFICATION_STAGE_MAIN));

      vec.insert(MetaRule(_not(_and(_vara(), _varb())), _or(_not(_vara()), _not(_varb())), SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule(_not(_or(_vara(), _varb())), _and(_not(_vara()), _not(_varb())), SIMPLIFICATION_STAGE_MAIN));

      vec.insert(MetaRule("!(G(x)) ==> F(!(x))", SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule("!(F(x)) ==> G(!(x))", SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule("!(X(x)) ==> X(!(x))", SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule("!(x U y) ==> (!(x)) V (!(y))", SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule("!(x V y) ==> (!(x)) U (!(y))", SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule("!(x <=> y) ==> (!(x)) <=> (!(y))", SIMPLIFICATION_STAGE_MAIN));
      vec.insert(MetaRule("!(x => y) ==> x && !(y)", SIMPLIFICATION_STAGE_MAIN));
   }
   else if (mystruct.op_name == SYMB_IF) {
      vec.insert(MetaRule(
         _if(_vara(), _varb()),
         _if(_vara(), _varb()), 
         SIMPLIFICATION_STAGE_MAIN,
         {
            { _vara(), ConditionType::has_no_sideeffects },
            { _vara(), ConditionType::is_any_var_of_second_on_any_left_side_earlier_AND_INSERT },
         }
      )); // if (a) {b}

      vec.insert(MetaRule(_if(_vara(), _varb()), _varb(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::is_constant_and_evaluates_to_true } }));
      vec.insert(MetaRule(_if(_vara(), _varb()), _val0(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::is_constant_and_evaluates_to_false } }));

      vec.insert(MetaRule(_if(_vara(), _varb()), _val0(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_no_sideeffects }, { _varb(), ConditionType::is_constant_and_evaluates_to_false } }));
      vec.insert(MetaRule(_if(_vara(), _if(_not(_vara()), _varb())), _val0(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_no_sideeffects } }));

      vec.insert(MetaRule(_if(_boolify(_vara()), _varb()), _if(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_no_sideeffects } }));

      // if (a) { if (b) { c } } ==> if (a && b) { c }
      vec.insert(MetaRule(_if(_vara(), _if(_varb(), _varc())), _if(_and(_vara(), _varb()), _varc()), SIMPLIFICATION_STAGE_MAIN));

      // if (a) { if (!a) {c} } ==> 0
      vec.insert(MetaRule(_if(_vara(), _if(_varb(), _varc())), _val0(), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other }, {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { if (a) {c} } ==> if (a) {c}
      vec.insert(MetaRule(_if(_vara(), _if(_vara(), _varc())), _if(_vara(), _varc()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    if (!a) { c } else { d }
      // } ==> if (a) {d}
      vec.insert(MetaRule(_if(_vara(), _ifelse(_varb(), _varc(), _vard())), _if(_vara(), _vard()), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other }, {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    if (a) { c } else { d }
      // } ==> if (a) {c}
      vec.insert(MetaRule(_if(_vara(), _ifelse(_vara(), _varc(), _vard())), _if(_vara(), _varc()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects} }));
   }
   else if (mystruct.op_name == SYMB_IF_ELSE) { // ifelse(p_(0), p_(1), p_(2)) ==> if(p_(0), p_(1)) [p_(2) {1}]
      auto from_and_to{ _ifelse(_vara(), _varb(), _varc()) };
      vec.insert(MetaRule(
         from_and_to,
         from_and_to->copy(), 
         SIMPLIFICATION_STAGE_MAIN,
         {
            { _vara(), ConditionType::has_no_sideeffects },
            { _vara(), ConditionType::is_any_var_of_second_on_any_left_side_earlier_AND_INSERT},
         }
      )); // if(a) {b} else {c}

      vec.insert(MetaRule(_ifelse(_vara(), _varb(), _varc()), _varb(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::is_constant_and_evaluates_to_true } }));
      vec.insert(MetaRule(_ifelse(_vara(), _varb(), _varc()), _varc(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::is_constant_and_evaluates_to_false } }));

      //vec.insert(MetaRule(_ifelse(_vara(), _varb(), _varc()), _if(_vara(), _varb()), { { _varc(), ConditionType::is_constant_and_evaluates_to_false } }));       // ifelse (a, b, 0) ==> if (a, b)
      //vec.insert(MetaRule(_ifelse(_vara(), _varb(), _varc()), _if(_not(_vara()), _varc()), { { _varb(), ConditionType::is_constant_and_evaluates_to_false } })); // ifelse (a, 0, c) ==> if (!(a), c)
      // TODO: The above two lines are not quite correct since "returning 0" might be desired. Also, they led to an infinite loop in some cases ==> find out why.

      vec.insert(MetaRule(_ifelse(_vara(), _varb(), _varc()), _val0(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_no_sideeffects }, { _varb(), ConditionType::is_constant_and_evaluates_to_false }, { _varc(), ConditionType::is_constant_and_evaluates_to_false } }));
      vec.insert(MetaRule(_ifelse(_vara(), _varb(), _varb()), _varb(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_no_sideeffects } }));

      vec.insert(MetaRule(_ifelse(_boolify(_vara()), _varb(), _varc()), _ifelse(_vara(), _varb(), _varc()), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_no_sideeffects } }));

      // if (a) { 
      //    if (!a) { c } 
      // } else {
      //    d
      // } ==> if (!a) {d}
      vec.insert(MetaRule(_ifelse(_vara(), _if(_varb(), _varc()), _vard()), _if(_not(_vara()), _vard()), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other }, {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    if (!a) { c } else { d }
      // } else {
      //    e
      // }
      // ==> if (a) {d} else {e}
      vec.insert(MetaRule(_ifelse(_vara(), _ifelse(_varb(), _varc(), _vard()), _vare()), _ifelse(_vara(), _vard(), _vare()), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other }, {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    if (a) { c } 
      // } else {
      //    d
      // } ==> if (a) {c} else {d}
      vec.insert(MetaRule(_ifelse(_vara(), _if(_vara(), _varc()), _vard()), _ifelse(_vara(), _varc(), _vard()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    if (a) { c } else { d }
      // } else {
      //    e
      // }
      // ==> if (a) {c} else {e}
      vec.insert(MetaRule(_ifelse(_vara(), _ifelse(_vara(), _varc(), _vard()), _vare()), _ifelse(_vara(), _varc(), _vare()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects} }));

      ///////////////////

      // if (a) { 
      //    d
      // } else {
      //    if (!a) { c } 
      // } ==> if (a) {d} else {c}
      vec.insert(MetaRule(_ifelse(_vara(), _vard(), _if(_varb(), _varc())), _ifelse(_vara(), _vard(), _varc()), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other }, {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    e
      // } else {
      //    if (!a) { c } else { d }
      // }
      // ==> if (a) {e} else {c}
      vec.insert(MetaRule(_ifelse(_vara(), _vare(), _ifelse(_varb(), _varc(), _vard())), _ifelse(_vara(), _vare(), _varc()), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other }, {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    d
      // } else {
      //    if (a) { c } 
      // } ==> if (a) {d}
      vec.insert(MetaRule(_ifelse(_vara(), _vard(), _if(_vara(), _varc())), _if(_vara(), _vard()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects} }));

      // if (a) { 
      //    e
      // } else {
      //    if (a) { c } else { d }
      // }
      // ==> if (a) {e} else {d}
      vec.insert(MetaRule(_ifelse(_vara(), _vare(), _ifelse(_vara(), _varc(), _vard())), _ifelse(_vara(), _vare(), _vard()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects} }));
   }

   return vec;
}

std::string vfm::TermCompound::getOptorOnCompoundLevel() const
{
   return getOperandsConst()[0]->getOpStruct().op_name;
}

void TermCompound::renamePrivateVars()
{
   std::shared_ptr<std::map<std::string, int>> private_vars_map = std::make_shared<std::map<std::string, int>>();
   private_var_names_->clear();

   getCompoundStructure()->applyToMeAndMyChildren([this, private_vars_map](const std::shared_ptr<MathStruct>& m)
      {
         if (m->isTermVar()) {
            std::string var_name = m->getOptor();
            if (StaticHelper::stringStartsWith(var_name, PRIVATE_VARIABLES_PREFIX) || StaticHelper::stringStartsWith(var_name, SYMB_REF + PRIVATE_VARIABLES_PREFIX)) {
#if defined(COMPOUND_DEBUG)
               std::cout << std::endl << "(c) Found private variable: " << m->getOptor() << " ('" << *m->getPtrToRoot() << "')";
#endif
               auto term_var = m->toVariableIfApplicable();
               bool has_ref = StaticHelper::stringStartsWith(var_name, SYMB_REF);
               std::string var_name_without_ref = has_ref ? var_name.substr(1) : var_name;
               std::string real_name = StaticHelper::isRenamedPrivateVar(var_name_without_ref) ? basePart(var_name_without_ref) : var_name_without_ref;
               std::string real_name_with_ref = has_ref ? SYMB_REF + real_name : real_name;

               this->registerPrivateVar(var_name_without_ref, private_vars_map);
               term_var->setOptor(privateVarName(real_name_with_ref, private_vars_map->at(real_name)));
               term_var->setPrivateVariable(true);

#if defined(COMPOUND_DEBUG)
               std::cout << std::endl << "(c) New name              : " << m->getOptor() << " ('" << *m->getPtrToRoot() << "')";
#endif
            }
         }
      }
   );

   for (const auto& el : *private_vars_map) {
      private_var_names_->push_back(privateVarName(el.first, private_vars_map->at(el.first)));
   }
}

void TermCompound::registerPrivateVar(
   const std::string& var_name,
   const std::shared_ptr<std::map<std::string, int>>& private_vars_map)
{
   if (!private_vars_map->count(var_name)) {
      if (StaticHelper::isRenamedPrivateVar(var_name)) {
         std::string base_part = vfm::TermCompound::basePart(var_name);
         int num_part = std::stoi(var_name.substr(3, var_name.length() - 3));
         private_vars_map->insert(std::pair<std::string, int>(base_part, num_part));
      }
      else {
         private_vars_map->insert(std::pair<std::string, int>(var_name, MathStruct::first_free_private_var_num_));
         MathStruct::first_free_private_var_num_++;

         // Keep this for future switch to non-static version of first_free_private_var_num_ variable.
         //registerPrivateVarsCounter();
         //private_vars_map->insert(std::pair<std::string, int>(var_name, *first_free_private_var_num_));
         //(*first_free_private_var_num_)++;
      }
   }
}

// Keep this for future switch to non-static version of first_free_private_var_num_ variable.
//void vfm::TermCompound::registerPrivateVarsCounter()
//{
//   if (!first_free_private_var_num_) {
//      std::shared_ptr<int> pointer_to_existing_counter = nullptr;
//      std::shared_ptr<bool> break_condition = std::make_shared<bool>(false);
//      auto root_ptr = getPtrToRoot(true);
//
//      root_ptr->applyToMeAndMyChildren([&pointer_to_existing_counter, break_condition](const std::shared_ptr<MathStruct> m) {
//         if (m->isCompound()) {
//            auto ptr_to_first_free_var_num = m->toCompoundIfApplicable()->first_free_private_var_num_;
//
//            if (ptr_to_first_free_var_num) {
//               pointer_to_existing_counter = ptr_to_first_free_var_num;
//               *break_condition = true;
//            }
//         }
//      }, true, nullptr, break_condition);
//
//      if (pointer_to_existing_counter) {
//         first_free_private_var_num_ = pointer_to_existing_counter;
//      }
//      else {
//         first_free_private_var_num_ = std::make_shared<int>(0);
//      }
//   }
//}

std::string vfm::TermCompound::privateVarName(const std::string& var_name, const int var_num)
{
   return var_name.substr(0, 2 + StaticHelper::stringStartsWith(var_name, SYMB_REF) * SYMB_REF.length()) + PRIVATE_VARIABLES_INTERNAL_SYMBOL + std::to_string(var_num);
}

std::string vfm::TermCompound::basePart(const std::string& var_name)
{
   return var_name.substr(0, 2);
}

std::map<std::string, std::pair<OperatorStructure, TLType>> vfm::TermCompound::getNuSMVOperators()
{
   if (NUSMV_OPERATORS.empty()) {
      for (const auto& opstruct : NUSMV_TL_OERATORS) {
         NUSMV_OPERATORS.insert({ opstruct.op_name, { opstruct, TLType::both } });
      }

      for (const auto& opstruct : NUSMV_LTL_OERATORS) {
         NUSMV_OPERATORS.insert({ opstruct.op_name, { opstruct, TLType::LTL } });
      }

      for (const auto& opstruct : NUSMV_CTL_OERATORS) {
         NUSMV_OPERATORS.insert({ opstruct.op_name, { opstruct, TLType::CTL } });
      }
   }

   return NUSMV_OPERATORS;
}

std::shared_ptr<TermCompound> vfm::TermCompound::compoundFactory(
   const std::vector<std::shared_ptr<Term>>& opds, 
   const std::shared_ptr<Term> compound_structure, 
   const OperatorStructure& operator_structure)
{
   auto cmp{ std::shared_ptr<TermCompound>(new TermCompound(opds, compound_structure, operator_structure)) };

   if (compound_structure) { // Otherwise the compound term is unfinished and has to be completed via setCompoundStructure before usage. 
      auto cmp_str_copy{ compound_structure->copy() };
      cmp->setCompoundStructure(cmp_str_copy); // TODO: Investigate if this is really needed.
   }

   if (cmp->getCompoundOperator()) {
      cmp->getCompoundOperator()->setFather(cmp);
   }

   if (cmp->getCompoundStructure()) {
      //cmp->getCompoundStructure()->setFather(cmp); // TODO: For now, compound structures root at nullptr, compound references go to all possible fathers.
   }

   return cmp;
}
