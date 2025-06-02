#include "simplification/code_block.h"
#include "static_helper.h"
#include "term_val.h"
#include "parser.h"
#include "meta_rule.h"
#include <fstream>

using namespace vfm;
using namespace code_block;

static const std::string SIMPLIFICATION_REGULAR_FUNCTION_NAMESPACE{ "MathStruct" };
static const std::string SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS{ "simplification_pos" };
static const std::string SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG{ "simplification" };
static const std::string SIMPLIFICATION_VERY_FAST_FUNCTION_NAMESPACE{ SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG };
static const std::string SIMPLIFICATION_REGULAR_FUNCTION_NAME{ "simplify" };
static const std::string SIMPLIFICATION_FAST_FUNCTION_NAME{ "simplifyFast" };
static const std::string SIMPLIFICATION_VERY_FAST_FUNCTION_NAME{ "simplifyVeryFast" };
static const std::string SIMPLIFICATION_REGULAR_FUNCTION_NAME_FULL{ SIMPLIFICATION_REGULAR_FUNCTION_NAMESPACE + "::" + SIMPLIFICATION_REGULAR_FUNCTION_NAME };
static const std::string SIMPLIFICATION_FAST_FUNCTION_NAME_FULL_POS{ SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS + "::" + SIMPLIFICATION_FAST_FUNCTION_NAME };
static const std::string SIMPLIFICATION_FAST_FUNCTION_NAME_FULL_NEG{ SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG + "::" + SIMPLIFICATION_FAST_FUNCTION_NAME };
static const std::string SIMPLIFICATION_VERY_FAST_FUNCTION_NAME_FULL{ SIMPLIFICATION_VERY_FAST_FUNCTION_NAMESPACE + "::" + SIMPLIFICATION_VERY_FAST_FUNCTION_NAME };
static constexpr bool OPTIMIZE_AWAY_FIRST_CONDITION_FOR_FAST_SIMPLIFICATION{ true }; // Currently only implemented for "negative" version. (Compiler optimizes away, anyway?)

std::string CodeBlock::indentation(const int length, const std::string symbol)
{
   std::string result;
   for (int i = 0; i < length; i++)
      result += symbol;
   return result;
}

CodeBlock::CodeBlock(const std::string& content, const std::string& comment)
   : CodeBlock(content, comment, "/*", "*/", "//") {}

CodeBlock::CodeBlock(const std::string& content, const std::string& comment, const std::string& comment_denoter_before, const std::string& comment_denoter_after, const std::string& comment_denoter_inline)
   : Failable("CodeBlock"), content_(content), comment_(comment), comment_denoter_before_{ comment_denoter_before }, comment_denoter_after_{ comment_denoter_after }, comment_denoter_inline_{ comment_denoter_inline } {}

std::string CodeBlock::serializeSingleLine() const { return serializeSingleLine(0, "formula"); }

std::string CodeBlock::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   return indentation(indent) 
      + (isComment() ? (comment_denoter_before_ + " ") : "") 
      + getContent() + (comment_.empty() ? "" : " " + comment_denoter_inline_ + " " + comment_) 
      + (isComment() ? (" " + comment_denoter_after_) : "") + "\n"
      + missingPredecessorNote();
}

std::string CodeBlock::missingPredecessorNote() const
{
   return successor_ && !successor_->predecessor_ ? "<<< ?? <<< " : (successor_ && successor_->predecessor_ != shared_from_this() ? "<<< ? -<<< " : "");
}

void CodeBlock::applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<CodeBlock>)>& f)
{
   bool dummy{ false };
   applyToMeAndMyChildren(f, dummy);
}

void CodeBlock::applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<CodeBlock>)>& f, bool& breakitoff)
{
   if (!breakitoff) {
      f(shared_from_this());
   }

   if (!breakitoff) {
      if (successor_) successor_->applyToMeAndMyChildren(f, breakitoff);
   }
}

void CodeBlock::applyToMeAndMyParents(
   const std::function<void(const std::shared_ptr<CodeBlock>)>& f,
   const bool exclude_me
)
{
   if (!exclude_me) f(shared_from_this());

   assert(!(predecessor_ && father_.lock())); // Only one can be true at a time.

   if (predecessor_) predecessor_->applyToMeAndMyParents(f);
   else if (father_.lock()) father_.lock()->applyToMeAndMyParents(f);
}

void CodeBlock::replace(const std::shared_ptr<CodeBlock> line)
{
   if (predecessor_) {
      line->predecessor_ = predecessor_;
      predecessor_->successor_ = line;

      if (successor_) {
         successor_->predecessor_ = line->getLastLine();
      }
   }
   else if (father_.lock()) {
      addFatalError("Not implemented.");
   }
   else {
      addFatalError("Replace failed. Neither predecessor nor father exists for '" + serializeSingleLine() + "'.");
   }
}

bool vfm::code_block::CodeBlock::isEmptyLine() const
{
   return const_cast<CodeBlock*>(this)->toNullIfApplicable() 
      || const_cast<CodeBlock*>(this)->toPlaceholderIfApplicable();
}

bool CodeBlock::isEmptyFromThisLineOn() const
{
   if (!isEmptyLine()) return false;
   if (!successor_) return true;

   return successor_->isEmptyFromThisLineOn();
}

enum class MarkerMode {
   never,
   always,
   automatic
};

std::string CodeBlock::serializeBlock() const
{
   return serializeBlock(0, "m");
}

std::string CodeBlock::serializeBlock(const int indent, const std::string& formula_name, const MarkerMode marker_mode) const
{
   std::string s{};
   auto temp{ shared_from_this() };
   bool mark_current{ marker_mode == MarkerMode::automatic ? (bool)temp->getPredecessor() : marker_mode == MarkerMode::always };

   while (temp->getPredecessor()) temp = temp->getPredecessor();

   while (temp) {
      if (mark_current && temp == shared_from_this()) {
         s += ">>> " + StaticHelper::trimAndReturn(temp->serializeSingleLine(indent, formula_name)) + " <<<\n";
      }
      else {
         s += temp->serializeSingleLine(indent, formula_name);
      }

      temp = temp->getSuccessor();
   }

   return s;
}

bool CodeBlock::isStructurallyEqual(const std::shared_ptr<CodeBlock> other)
{
   if (!other || getContent() != other->getContent()) {
      return false;
   }

   if (getSuccessor()) {
      if (other->getSuccessor()) {
         return getSuccessor()->isStructurallyEqual(other->getSuccessor());
      }
      else {
         return false;
      }
   }
   else {
      return !other->getSuccessor();
   }
}

std::shared_ptr<CodeBlockIf> toIfIfApplicable() { return nullptr; }
std::shared_ptr<CodeBlockReturn> toReturnIfApplicable() { return nullptr; }
std::shared_ptr<CodeBlockDecl> toDeclIfApplicable() { return nullptr; }
std::shared_ptr<CodeBlockDef> toDefIfApplicable() { return nullptr; }
std::shared_ptr<CodeBlockCustom> toCustomIfApplicable() { return nullptr; }
std::shared_ptr<CodeBlockNull> toNullIfApplicable() { return nullptr; }
std::shared_ptr<CodeBlockPlaceholder> toPlaceholderIfApplicable() { return nullptr; }

std::string CodeBlock::getContent() const
{
   return content_;
}

bool CodeBlock::containsLineBelowIncludingThis(const std::string& line) const
{
   if (getContent() == line) {
      return true;
   }

   if (getSuccessor()) {
      return getSuccessor()->containsLineBelowIncludingThis(line);
   }

   return false;
}

bool CodeBlock::containsLineAboveIncludingThis(const std::string& line) const
{
   if (getContent() == line) {
      return true;
   }

   if (getPredecessor()) {
      return getPredecessor()->containsLineAboveIncludingThis(line);
   }

   return false;
}

void CodeBlock::setContent(const std::string& content)
{
   content_ = content;
}

std::shared_ptr<CodeBlock> CodeBlock::getSuccessor() const
{
   return successor_;
}

std::shared_ptr<CodeBlock> CodeBlock::getPredecessor() const
{
   return predecessor_;
}

std::shared_ptr<CodeBlock> CodeBlock::getLastLine()
{
   auto el = shared_from_this();

   while (el->getSuccessor()) {
      el = el->getSuccessor();
   }

   return el;
}

std::shared_ptr<CodeBlock> CodeBlock::getFirstLine()
{
   auto el = shared_from_this();

   while (el->getPredecessor()) {
      el = el->getPredecessor();
   }

   return el;
}

bool CodeBlock::checkForEqualDefinitions()
{
   auto temp{ shared_from_this() };
   //while (temp->getPredecessor()) temp = temp->getPredecessor();

   while (temp) {
      if (toDefIfApplicable() && containsLineBelowIncludingThis(getContent())) {
         return true;
      }

      temp = temp->getSuccessor();
   }

   return false;
}

void CodeBlock::setSuccessor(const std::shared_ptr<CodeBlock> successor)
{
   if (successor) {
      successor->predecessor_ = shared_from_this();
      successor->setFather(nullptr);
   }
   successor_ = successor;
}

void CodeBlock::setPredecessor(const std::shared_ptr<CodeBlock> predecessor)
{
   if (predecessor) {
      predecessor->successor_ = shared_from_this();
      setFather(nullptr);
   }

   predecessor_ = predecessor;
}

void CodeBlock::appendAtTheEnd(const std::shared_ptr<CodeBlock> line, const bool replace_last_line, const bool insert_one_before_end)
{
   if (!line) return;

   auto current = shared_from_this();
   auto last = current;

   while (current->successor_) {
      last = current;
      current = current->successor_;
   }

   if (replace_last_line) {
      last->successor_ = line;
      line->predecessor_ = last;
      line->setFather(nullptr);
   }
   else if (insert_one_before_end) {
      auto ending = current;
      last->successor_ = line;
      line->predecessor_ = last;
      line->setFather(nullptr);
      last->appendAtTheEnd(ending);
   }
   else {
      current->successor_ = line;
      line->predecessor_ = current;
      line->setFather(nullptr);
   }
}

std::string CodeBlock::getComment() const
{
   return comment_;
}

void CodeBlock::setComment(const std::string& comment)
{
   comment_ = comment;
}

void CodeBlock::setFather(const std::shared_ptr< CodeBlock> father)
{
   father_ = father;

   if (father) {
      setPredecessor(nullptr);
   }
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlock::getFather() const
{
   return father_.lock();
}

void CodeBlock::makeComment(const bool do_it)
{
   make_comment_ = do_it;
}

bool CodeBlock::isComment() const
{
   return make_comment_;
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlock::goToNextNonEmptyLine()
{
   if (isEmptyLine()) {
      if (successor_) {
         return successor_->goToNextNonEmptyLine();
      }
      else {
         return nullptr;
      }
   }

   return shared_from_this();
}

std::shared_ptr<CodeBlockIf> vfm::code_block::CodeBlock::goToLastIfInBlock()
{
   if (toIfIfApplicable()) {
      auto behind{ successor_ ? successor_->goToLastIfInBlock() : nullptr };
      return behind ? behind : toIfIfApplicable();
   } else if (successor_) {
      return successor_->goToLastIfInBlock();
   } else {
      return nullptr;
   }
}

void CodeBlock::removeAllRedefinitionsOrReinitializations()
{
   addNote("Fixing re-declarations and re-initializations.");

   applyToMeAndMyChildren([](const std::shared_ptr<CodeBlock> m) {
      if (m->toDeclIfApplicable() || m->toDefIfApplicable()) {
         bool found{};

         m->applyToMeAndMyParents([&found, m](const std::shared_ptr<CodeBlock> m2) {
            if (!m->isComment()) {
               auto m2_content{ m2->getContent() };

               if (m2_content == m->getContent()) {
                  m->setComment("Re-declaration and/or re-initialization.");
                  m->makeComment(true);
               }
               else if (m->toDefIfApplicable() && m2->toDeclIfApplicable()) {
                  auto m_def{ m->toDefIfApplicable() };
                  if (m_def->isDeclare()) {
                     if (m_def->getLeftSide() + ";" == m2_content) {
                        m->setComment("Re-initialization.");
                        m_def->setDeclare(false);
                     }
                  }
               }
            }
            }, true);
      }
   });
}

void CodeBlock::mergeWithOtherSimplificationFunction(const std::shared_ptr<CodeBlock> other, const FormulaParser& p)
{
   auto line_me = goToLastIfInBlock();
   auto line_fct = other->goToLastIfInBlock();

   if (line_me && line_fct) {
      auto cond_me{ line_me->getCondition() };
      auto cond_fct{ line_fct->getCondition() };

      if (cond_me.isOtherEqual(cond_fct)) {
         line_me->getBodyIf()->mergeWithOtherSimplificationFunction(line_fct->getBodyIf(), p);
      }
      else if (cond_me.isOtherMoreSpecific(cond_fct, p)) {
         line_me->getBodyIf()->mergeWithOtherSimplificationFunction(line_fct, p);
      }
      else if (cond_me.isOtherUnrelated(cond_fct, p)) {
         line_me->getBodyElse()->mergeWithOtherSimplificationFunction(other, p);
      }
      else {
         line_me->getBodyElse()->mergeWithOtherSimplificationFunction(other, p);
      }
   }
   else if (!line_me && line_fct) { // Ended up in a leaf, just include the new code there.
      appendAtTheEnd(other->copy());
   } // If both are empty, do nothing.
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlock::copy()
{
   auto copy{ copy_core() };
   copy->make_comment_ = make_comment_;
   copy->content_ = content_;
   copy->comment_ = comment_;

   if (successor_) {
      copy->successor_ = successor_->copy();
      copy->successor_->predecessor_ = copy;
   }

   if (toIfIfApplicable()) {
      auto copy_if{ copy->toIfIfApplicable() };
      copy_if->setBodyIf(toIfIfApplicable()->getBodyIf()->copy());
      copy_if->setBodyElse(toIfIfApplicable()->getBodyElse()->copy());

      for (const auto& elsif : toIfIfApplicable()->getElseIfs()) {
         copy_if->addElseIf(elsif->getCondition(), elsif->getBodyIf()->copy());
      }
   }

   copy->checkIntegrity();

   return copy;
}

bool vfm::code_block::CodeBlock::checkIntegrity()
{
   bool fine{ true };

   applyToMeAndMyChildren([&fine, this](const std::shared_ptr<CodeBlock> l) {
      if (l->getPredecessor() && l->getFather()) {
         addError("Integrity check failed at '" + l->serializeBlock() + "'. We have a predecessor and father at the same time.");
         fine = false;
      }
      else if (l->getFather()) {
         if (!l->getFather()->toIfIfApplicable()) {
            addError("Integrity check failed at '" + l->serializeBlock() + "'. Father is not an IF.");
            fine = false;
         }
         else {
            auto father{ l->getFather()->toIfIfApplicable() };
            bool found{ false };

            if (father->getBodyIf() == l) found = true;
            if (father->getBodyElse() == l) found = true;

            for (const auto elseif : father->getElseIfs()) {
               if (elseif == l) found = true;
            }

            if (!found) {
               addError("Integrity check failed at '" + l->serializeBlock() + "'. Father IF '" + father->serializeBlock() + "' does not contain <this> as a child.");
               fine = false;
            }
         }
      }

      if (l->toIfIfApplicable()) {
         auto l_if{ l->toIfIfApplicable() };

         if (l_if->getBodyIf()->getFather() != l_if) {
            addError("Integrity check failed at '" + l->serializeBlock() + "'. Body IF'" + l_if->getBodyIf()->serializeBlock() + "' does not contain this as a father.");
            fine = false;
         }
         if (l_if->getBodyElse()->getFather() != l_if) {
            addError("Integrity check failed at '" + l->serializeBlock() + "'. Body ELSE'" + l_if->getBodyElse()->serializeBlock() + "' does not contain this as a father.");
            fine = false;
         }

         for (const auto elseif : l_if->getElseIfs()) {
            if (elseif->getFather() != l_if) {
               addError("Integrity check failed at '" + l->serializeBlock() + "'. Body ELSE IF'" + elseif->serializeBlock() + "' does not contain this as a father.");
               fine = false;
            }
         }
      }
   });

   return fine;
}

int vfm::code_block::CodeBlock::nodeCount()
{
   int cnt{ 0 };

   applyToMeAndMyChildren([&cnt](const std::shared_ptr<CodeBlock> l) {
      cnt++;
   });

   return cnt;
}

IfConditionComparisonSide::IfConditionComparisonSide() : mode_{ IfConditionComparisonSideMode::irregular } {}

IfConditionComparisonSide::IfConditionComparisonSide(const IfConditionComparisonSideMode mode, const std::string& formula_name, const std::vector<int>& trail)
   : mode_{ mode }, formula_name_{ formula_name }, trail_{ trail }
{
   assert(mode == IfConditionComparisonSideMode::operator_name_specifier
      || mode == IfConditionComparisonSideMode::value_check
      || mode == IfConditionComparisonSideMode::terms_name_specifier
      || mode == IfConditionComparisonSideMode::value_getter);
}

IfConditionComparisonSide::IfConditionComparisonSide(const IfConditionComparisonSideMode mode, const float number)
   : mode_{ mode }, number_{ number }
{
   assert(mode == IfConditionComparisonSideMode::number);
}

IfConditionComparisonSide::IfConditionComparisonSide(const IfConditionComparisonSideMode mode, const std::string& operator_name)
   : mode_{ mode }, operator_name_{ operator_name }
{
   assert(mode == IfConditionComparisonSideMode::operator_name);
}

std::string IfConditionComparisonSide::serialize() const
{
   if (mode_ == IfConditionComparisonSideMode::operator_name_specifier) {
      // formula_1->getOptor()
      return formula_name_ + CodeGenerator::getTrailForMetaRuleCodeGeneration(trail_, make_last_trail_item_native_) + "->getOptorOnCompoundLevel()";
   }
   else if (mode_ == IfConditionComparisonSideMode::terms_name_specifier) {
      // formula_1->getTerms().size() == 2
      return formula_name_ + CodeGenerator::getTrailForMetaRuleCodeGeneration(trail_, make_last_trail_item_native_) + "->getTermsJumpIntoCompounds().size()";
   }
   else if (mode_ == IfConditionComparisonSideMode::value_check) {
      // formula_0->isTermVal()
      return formula_name_ + CodeGenerator::getTrailForMetaRuleCodeGeneration(trail_, make_last_trail_item_native_) + "->isTermVal()";
   }
   else if (mode_ == IfConditionComparisonSideMode::value_getter) {
      // formula_0->toValueIfApplicable()->getValue()
      return formula_name_ + CodeGenerator::getTrailForMetaRuleCodeGeneration(trail_, make_last_trail_item_native_) + "->toValueIfApplicable()->getValue()";
   }
   else if (mode_ == IfConditionComparisonSideMode::number) {
      return std::to_string(number_);
   }
   else if (mode_ == IfConditionComparisonSideMode::operator_name) {
      return "\"" + operator_name_ + "\"";
   }

   return "#IRREGULAR_CONDITION_SIDE";
}

void IfConditionComparisonSide::setFormulaName(const std::string& name)
{
   formula_name_ = name;
}

IfConditionComparisonSideMode vfm::code_block::IfConditionComparisonSide::getMode() const
{
   return mode_;
}

std::string vfm::code_block::IfConditionComparisonSide::getFormulaName() const
{
   return formula_name_;
}

std::vector<int> vfm::code_block::IfConditionComparisonSide::getTrail() const
{
   return trail_;
}

std::string vfm::code_block::IfConditionComparisonSide::getOperatorName() const
{
   return operator_name_;
}

float vfm::code_block::IfConditionComparisonSide::getNumber() const
{
   return number_;
}

void vfm::code_block::IfConditionComparisonSide::makeLastTrailItemNative()
{
   make_last_trail_item_native_ = true;
}

CodeBlockIf::CodeBlockIf(
   const IfCondition& condition,
   const std::shared_ptr<CodeBlock> body_if,
   const std::shared_ptr<CodeBlock> body_else,
   const std::string& comment)
   : CodeBlock("", comment), body_if_(body_if), body_else_(body_else), condition_{ condition }
{
   if (!body_if_) body_if_ = std::make_shared<CodeBlockNull>();
   if (!body_else_) body_else_ = std::make_shared<CodeBlockNull>();
}

std::shared_ptr<CodeBlockIf> vfm::code_block::CodeBlockIf::getInstance(const IfCondition& condition, const std::shared_ptr<CodeBlock> body_if, const std::shared_ptr<CodeBlock> body_else, const std::string& comment)
{
   auto instance{ std::make_shared<CodeBlockIf>(condition, body_if, body_else, comment) };
   instance->body_if_->setFather(instance);
   instance->body_else_->setFather(instance);

   for (const auto& else_if : instance->else_ifs_) {
      else_if->setFather(instance);
   }

   return instance;
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlockIf::copy_core() const
{
   auto copy{ getInstance(getCondition(), body_if_->copy(), body_else_->copy()) };

   for (const auto& elsif : getElseIfs()) {
      copy->addElseIf(elsif->getCondition(), elsif->getBodyIf()->copy());
   }

   return copy;
}

IfCondition::IfCondition() : mode_{ IfConditionMode::irregular } {}

// formula_1->getOptor() == "-" && formula_1->getTerms().size() == 2
IfCondition::IfCondition(const IfConditionMode mode, const std::string& formula_name, const std::vector<int>& trail, const std::string& operator_name, const float value) :
   mode_{ mode },
   side1_{ IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name_specifier, formula_name, trail) },
   side2_{ IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name, operator_name) },
   side3_{ IfConditionComparisonSide(IfConditionComparisonSideMode::terms_name_specifier, formula_name, trail) },
   side4_{ IfConditionComparisonSide(IfConditionComparisonSideMode::number, value) }
{
   assert(mode == IfConditionMode::overloaded_comparison);
}

// formula->getTerms().size() == 2.000000 && formula->getOptor() == formula_1->getOptor()
// trail1                                    trail2                 trail3
IfCondition::IfCondition(const IfConditionMode mode, const std::string& formula_name, const std::vector<int>& trail1, const std::vector<int>& trail2, const std::vector<int>& trail3, const float value) :
   mode_{ mode },
   side1_{ IfConditionComparisonSide(IfConditionComparisonSideMode::terms_name_specifier, formula_name, trail1) },
   side2_{ IfConditionComparisonSide(IfConditionComparisonSideMode::number, value) },
   side3_{ IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name_specifier, formula_name, trail2) },
   side4_{ IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name_specifier, formula_name, trail3) }
{
   assert(mode == IfConditionMode::anyway_comparison);

   if (trail1 == trail2) {
      side4_.makeLastTrailItemNative();
   }
   else if (trail1 == trail3) {
      side3_.makeLastTrailItemNative();
   }
   else {
      Failable::getSingleton()->addFatalError("Unexpectedly received three different trails for anyway condition.");
   }
}

IfCondition::IfCondition(const IfConditionMode mode, const std::string& formula_name, const std::vector<int>& trail, const float value) :
   mode_{ mode },
   side1_{ IfConditionComparisonSide(IfConditionComparisonSideMode::value_check, formula_name, trail) },
   side2_{ IfConditionComparisonSide(IfConditionComparisonSideMode::value_getter, formula_name, trail) },
   side3_{ IfConditionComparisonSide(IfConditionComparisonSideMode::number, value) }
{
   assert(mode == IfConditionMode::value_comparison);
}

IfCondition::IfCondition(const IfConditionMode mode, const IfConditionComparisonSide& side1, const IfConditionComparisonSide& side2)
   : mode_{ mode }, side1_{ side1 }, side2_{ side2 }
{
   assert(mode == IfConditionMode::equal_comparison);
}

IfCondition::IfCondition(const IfConditionMode mode, const std::string& free_string)
   : mode_{ mode }, free_string_{ free_string }
{
   assert(mode == IfConditionMode::free);
}

std::string IfCondition::serialize() const
{
   if (mode_ == IfConditionMode::equal_comparison) {
      return side1_.serialize() + " == " + side2_.serialize();
   }
   else if (mode_ == IfConditionMode::value_comparison) {
      // formula_0->isTermVal() && formula_0->toValueIfApplicable()->getValue() == 1.000000
      return side1_.serialize() + " && " + side2_.serialize() + " == " + side3_.serialize();
   }
   else if (mode_ == IfConditionMode::overloaded_comparison || mode_ == IfConditionMode::anyway_comparison) {
      // overloaded: formula_1->getOptor() == "-" && formula_1->getTerms().size() == 2
      // anyway:     formula->getTerms().size() == 2.000000 && formula->getOptor() == formula_1->getOptor()
      return side1_.serialize() + " == " + side2_.serialize() + " && " + side3_.serialize() + " == " + side4_.serialize();
   }
   else if (mode_ == IfConditionMode::free) {
      return free_string_;
   }

   return "#IRREGULAR_CONDITION";
}

void IfCondition::setFormulaName(const std::string& name)
{
   side1_.setFormulaName(name);
   side2_.setFormulaName(name);
}

IfConditionMode vfm::code_block::IfCondition::getMode() const
{
   return mode_;
}

std::vector<IfConditionComparisonSide> vfm::code_block::IfCondition::getSides() const
{
   return { side1_, side2_, side3_, side4_ };
}

bool vfm::code_block::IfCondition::isAnywayConditionFirstPart() const
{
   return mode_ == IfConditionMode::equal_comparison
      && side1_.getMode() == IfConditionComparisonSideMode::terms_name_specifier
      && side2_.getMode() == IfConditionComparisonSideMode::number;
}

bool vfm::code_block::IfCondition::isAnywayConditionSecondPart() const
{
   return mode_ == IfConditionMode::equal_comparison
      && side1_.getMode() == IfConditionComparisonSideMode::operator_name_specifier
      && side2_.getMode() == IfConditionComparisonSideMode::operator_name_specifier;
}

int vfm::code_block::IfCondition::isSizeComparison() const
{
   return (mode_ == IfConditionMode::equal_comparison 
      && side1_.getMode() == IfConditionComparisonSideMode::terms_name_specifier
      && side2_.getMode() == IfConditionComparisonSideMode::number)
      ? side2_.getNumber()
      : -1;
}

int vfm::code_block::IfCondition::isOptorComparison(const FormulaParser& p) const
{
   if (mode_ == IfConditionMode::equal_comparison
      && side1_.getMode() == IfConditionComparisonSideMode::operator_name_specifier
      && side2_.getMode() == IfConditionComparisonSideMode::operator_name) {

      auto possible_num_params{ const_cast<FormulaParser&>(p).getNumParams(side2_.getOperatorName()) };

      if (possible_num_params.size() == 1) { // Only non-overloaded operators for now.
         return *possible_num_params.begin();
      }
   }

   return -2;
}

void vfm::code_block::IfCondition::reInitialize(const IfCondition other)
{
   auto sides{ other.getSides() };
   auto mode{ other.getMode() };

   mode_ = mode;
   side1_ = sides[0];
   side2_ = sides[1];
   side3_ = sides[2];
   side4_ = sides[3];
   free_string_ = other.free_string_;
}

bool vfm::code_block::IfCondition::isOtherEqual(const IfCondition& other) const
{
   return serialize() == other.serialize();
}

bool vfm::code_block::IfCondition::isOtherMoreSpecific(const IfCondition& other, const FormulaParser& p) const
{
   return isSizeComparison() == other.isOptorComparison(p);
}

bool vfm::code_block::IfCondition::isOtherUnrelated(const IfCondition& other, const FormulaParser& p) const
{
   return !isOtherEqual(other) && !isOtherMoreSpecific(other, p) && !other.isOtherMoreSpecific(*this, p)
      || mode_ == IfConditionMode::free || other.mode_ == IfConditionMode::free;
}

 void CodeBlockIf::applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<CodeBlock>)>& f, bool& breakitoff)
{
   auto if_line{ toIfIfApplicable() };
   auto if_body{ if_line->getBodyIf() };
   auto if_body_else{ if_line->getBodyElse() };
   auto if_body_else_ifs{ if_line->getElseIfs() };

   if (if_body) {
      if_body->applyToMeAndMyChildren(f, breakitoff);
   }

   if (if_body_else) {
      if_body_else->applyToMeAndMyChildren(f, breakitoff);
   }

   for (const auto& else_if : if_body_else_ifs) {
      if (else_if) {
         else_if->applyToMeAndMyChildren(f, breakitoff);
      }
   }

   CodeBlock::applyToMeAndMyChildren(f, breakitoff);
}

 std::shared_ptr<CodeBlockIf> CodeBlockIf::toIfIfApplicable()
{
   return std::dynamic_pointer_cast<CodeBlockIf>(shared_from_this());
}

 std::string CodeBlockIf::getContent() const
{
   return condition_.serialize();
}

 std::string CodeBlockIf::serializeSingleLine() const { return serializeSingleLine(0, "formula"); }

 std::string CodeBlockIf::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   condition_.setFormulaName(formula_name);

   std::string s = indentation(indent) + "if (" + getConditionString() + ") {" + (getComment().empty() ? "" : " // " + getComment()) + "\n";
   if (body_if_) {
      s += body_if_->serializeBlock(indent + 3, formula_name, MarkerMode::never);
   }
   else {
      s += indentation(indent + 3) + "UNFINISHED\n";
   }

   for (const auto& elseif : else_ifs_) {
      s += indentation(indent) + "} else if (" + elseif->getConditionString() + ") {\n";
      s += elseif->getBodyIf()->serializeBlock(indent + 3, formula_name, MarkerMode::never);
   }

   if (body_else_ && !body_else_->isEmptyFromThisLineOn()) {
      s += indentation(indent) + "} else {\n";
      s += body_else_->serializeBlock(indent + 3, formula_name, MarkerMode::never);
   }

   return s + indentation(indent) + "}\n" + missingPredecessorNote();
}

IfCondition vfm::code_block::CodeBlockIf::getCondition() const
{
   return condition_;
}

std::string CodeBlockIf::getConditionString() const {
   return getContent();
}

std::shared_ptr<CodeBlock> CodeBlockIf::getBodyIf() const
{
   return body_if_;
}

void CodeBlockIf::setBodyIf(const std::shared_ptr<CodeBlock> body)
{
   body->setFather(toIfIfApplicable());
   body->setPredecessor(nullptr);
   body_if_ = body;
}

std::shared_ptr<CodeBlock> CodeBlockIf::getBodyElse() const
{
   return body_else_;
}

void CodeBlockIf::setBodyElse(const std::shared_ptr<CodeBlock> body)
{
   body->setFather(toIfIfApplicable());
   body->setPredecessor(nullptr);
   body_else_ = body;
}

std::vector<std::shared_ptr<CodeBlockIf>> CodeBlockIf::getElseIfs() const {
   return else_ifs_;
}

void CodeBlockIf::addElseIf(const IfCondition& condition, const std::shared_ptr<CodeBlock> body) {
   auto subfather{ CodeBlockIf::getInstance(condition, body) };
   subfather->setFather(toIfIfApplicable());
   else_ifs_.push_back(subfather);
   body->setFather(subfather);
   body->setPredecessor(nullptr);
}

void vfm::code_block::CodeBlockIf::setCondition(const IfCondition& condition)
{
   condition_ = condition;
}

CodeBlockDef::CodeBlockDef(
   const std::vector<int>& trace_up_to,
   const int next,
   const std::string& custom_right_side,
   const std::string& comment,
   const bool declare)
   : CodeBlock("", comment), trace_up_to_(trace_up_to), next_(next), custom_rhs_(custom_right_side), declare_(declare) {}

std::shared_ptr<CodeBlockDef> CodeBlockDef::toDefIfApplicable()
{
   return std::dynamic_pointer_cast<CodeBlockDef>(shared_from_this());
}

std::string CodeBlockDef::getLeftSide() const
{
   return StaticHelper::trimAndReturn(StaticHelper::split(getContent(), "=")[0]);
}

std::string CodeBlockDef::getContent() const
{
   std::string s{};
   std::string trace{};

   for (const auto& el : trace_up_to_) {
      trace += "_" + std::to_string(el);
   }

   return (declare_ ? "TermPtr " : "") + PLACEHOLDER_FORMULA_NAME + trace + "_" + std::to_string(next_) + " = "
      + (custom_rhs_.empty() ? PLACEHOLDER_FORMULA_NAME + trace + "->getTermsJumpIntoCompounds()[" + std::to_string(next_) + "]" : custom_rhs_) + ";";
}

std::vector<int> CodeBlockDef::getTraceUpTo() const
{
   return trace_up_to_;
}

int CodeBlockDef::getNext() const
{
   return next_;
}

std::string CodeBlockDef::serializeSingleLine() const { return serializeSingleLine(0, "formula"); }

std::string CodeBlockDef::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   return indentation(indent) + (isComment() ? "// " : "") + StaticHelper::replaceAll(getContent(), PLACEHOLDER_FORMULA_NAME, formula_name)
      + (getComment().empty() ? "" : " // " + getComment()) + "\n" + missingPredecessorNote();
}

void CodeBlockDef::setDeclare(const bool do_it)
{
   declare_ = do_it;
}

bool CodeBlockDef::isDeclare() const
{
   return declare_;
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlockDef::copy_core() const
{
   auto copy{ std::make_shared<CodeBlockDef>(trace_up_to_, next_, custom_rhs_, getComment(), declare_) };
   return copy;
}

CodeBlockDecl::CodeBlockDecl(const std::vector<int>& trace_up_to, const int next, const std::string& comment)
   : CodeBlock("", comment), trace_up_to_(trace_up_to), next_(next) {}

std::shared_ptr<CodeBlockDecl> CodeBlockDecl::toDeclIfApplicable()
{
   return std::dynamic_pointer_cast<CodeBlockDecl>(shared_from_this());
}

std::string CodeBlockDecl::getContent() const
{
   std::string s{};
   std::string trace{};

   for (const auto& el : trace_up_to_) {
      trace += "_" + std::to_string(el);
   }

   return "TermPtr " + PLACEHOLDER_FORMULA_NAME + trace + "_" + std::to_string(next_) + ";";
}

std::vector<int> CodeBlockDecl::getTraceUpTo() const
{
   return trace_up_to_;
}

int CodeBlockDecl::getNext() const
{
   return next_;
}

std::string CodeBlockDecl::serializeSingleLine() const { return serializeSingleLine(0, "formula"); }

std::string CodeBlockDecl::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   return indentation(indent) + (isComment() ? "// " : "") + StaticHelper::replaceAll(getContent(), PLACEHOLDER_FORMULA_NAME, formula_name)
      + (getComment().empty() ? "" : " // " + getComment()) + "\n" + missingPredecessorNote();
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlockDecl::copy_core() const
{
   auto copy{ std::make_shared<CodeBlockDecl>(trace_up_to_, next_, getComment()) };
   return copy;
}

CodeBlockReturn::CodeBlockReturn(const bool value, const std::string& comment)
   : CodeBlock("return " + std::string(value ? "true" : "false") + ";", comment), ret_value_(value) {}

std::shared_ptr<CodeBlockReturn> CodeBlockReturn::toReturnIfApplicable()
{
   return std::dynamic_pointer_cast<CodeBlockReturn>(shared_from_this());
}

std::string CodeBlockReturn::serializeSingleLine() const { return serializeSingleLine(0, "formula"); }

std::string CodeBlockReturn::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   if (formula_name == "formula") { // TODO: Extremely ugly, but it's just this one specific usecase, so don't care for now.
      return CodeBlock::serializeSingleLine(indent, formula_name) + missingPredecessorNote();
   }
   else {
      std::string changed_string = ret_value_ ? indentation(indent) + (isComment() ? "// " : "") + "changed_this_time = true;\n" : "";
      return changed_string + indentation(indent) + "break;\n" + missingPredecessorNote();
   }
}

// std::string getContent() const override
//{
//   return serializeSingleLine(0, PLACEHOLDER_FORMULA_NAME);
//}

bool CodeBlockReturn::getRetValue() const
{
   return ret_value_;
}

void CodeBlockReturn::setRetValue(const bool value)
{
   ret_value_ = value;
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlockReturn::copy_core() const
{
   auto copy{ std::make_shared<CodeBlockReturn>(ret_value_, getComment()) };
   return copy;
}

CodeBlockCustom::CodeBlockCustom(const std::string& arbitrary_content, const std::string& comment)
   : CodeBlock(arbitrary_content, comment, "/*", "*/", "//"), arbitrary_content_(arbitrary_content) {}

CodeBlockCustom::CodeBlockCustom(const std::string& arbitrary_content, const std::string& comment, const std::string& comment_denoter_before, const std::string& comment_denoter_after, const std::string& comment_denoter_inline)
   : CodeBlock(arbitrary_content, comment, comment_denoter_before, comment_denoter_after, comment_denoter_inline), arbitrary_content_(arbitrary_content) {}

std::shared_ptr<CodeBlockCustom> CodeBlockCustom::toCustomIfApplicable()
{
   return std::dynamic_pointer_cast<CodeBlockCustom>(shared_from_this());
}

std::string CodeBlockCustom::getRetValue() const
{
   return arbitrary_content_;
}

void CodeBlockCustom::setRetValue(const bool arbitrary_content)
{
   arbitrary_content_ = arbitrary_content;
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlockCustom::copy_core() const
{
   auto copy{ std::make_shared<CodeBlockCustom>(arbitrary_content_, getComment()) };
   return copy;
}

CodeBlockNull::CodeBlockNull()
   : CodeBlock("", "") {}

std::shared_ptr<CodeBlockNull> CodeBlockNull::toNullIfApplicable()
{
   return std::dynamic_pointer_cast<CodeBlockNull>(shared_from_this());
}

std::string CodeBlockNull::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   return ""; // Return "**NULL**" or so to get position of null lines for debugging.
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlockNull::copy_core() const
{
   auto copy{ std::make_shared<CodeBlockNull>() };
   return copy;
}

CodeBlockPlaceholder::CodeBlockPlaceholder() : CodeBlockNull() {}

std::shared_ptr<CodeBlockPlaceholder> CodeBlockPlaceholder::toPlaceholderIfApplicable()
{
   return std::dynamic_pointer_cast<CodeBlockPlaceholder>(shared_from_this());
}

std::string CodeBlockPlaceholder::serializeSingleLine(const int indent, const std::string& formula_name) const
{
   return ""; // Return "**PLACEHOLDER**" or so to get position of null lines for debugging.
}

std::shared_ptr<CodeBlock> vfm::code_block::CodeBlockPlaceholder::copy_core() const
{
   auto copy{ std::make_shared<CodeBlockPlaceholder>() };
   return copy;
}

std::string CodeGenerator::getTrailForMetaRuleCodeGeneration(const std::vector<int>& trail_orig, const bool make_last_item_native)
{
   std::string trail{};

   for (int i = 0; i < trail_orig.size(); i++) {
      const auto& el{ trail_orig[i] };

      if (i == trail_orig.size() - 1 && make_last_item_native) {
         trail += "->getTermsJumpIntoCompounds()[" + std::to_string(el) + "]";
      }
      else {
         trail += "_" + std::to_string(el);
      }
   }

   return trail;
}

std::shared_ptr<CodeBlock> createRelevantDefinitionsForMetaRuleCodeNegativeStyle(
   const TermPtr from_part_raw,
   const bool optimize_away_first_condition,
   const std::string& formula_name,
   std::map<int, std::vector<std::pair<std::string, std::vector<int>>>>& anyway_mapping,
   const std::string parser_name,
   const std::shared_ptr<FormulaParser> parser,
   const std::vector<int>& trail_orig = {})
{
   auto function_line{ std::make_shared<CodeBlockNull>() };
   auto from_part{ from_part_raw };

   if (from_part->isTermCompound()) {
      return createRelevantDefinitionsForMetaRuleCodeNegativeStyle(
         from_part_raw->getOperands()[0],
         optimize_away_first_condition,
         formula_name,
         anyway_mapping,
         parser_name,
         parser,
         trail_orig);
   }

   if (from_part->isMetaSimplification()) {
      return function_line;
   }

   std::string trail{};

   for (const auto& el : trail_orig) {
      trail += "_" + std::to_string(el);
   }

   std::shared_ptr<IfCondition> condition{};

   int optional_index = from_part->getOperands().size() == 2 && from_part->getOperands()[0]->isTermOptional() // Optional only allowed for two-operand terms.
      ? 0
      : (from_part->getOperands().size() >= 2 && from_part->getOperands()[1]->isTermOptional()
         ? 1
         : -1);

   std::shared_ptr<CodeBlock> possible_anyway_condition = nullptr;

   if (from_part->isTermVal()) {
      condition = std::make_shared<IfCondition>(IfConditionMode::free, "!" + formula_name + trail + "->isTermVal() || " + formula_name + trail + "->toValueIfApplicable()->getValue() != " + std::to_string(from_part->toValueIfApplicable()->getValue()) + "");

   }
   else if (from_part->toTermAnywayIfApplicable()) {
      condition = std::make_shared<IfCondition>(IfConditionMode::free, formula_name + trail + "->getTermsJumpIntoCompounds().size() != " + std::to_string(from_part->getOperands().size() - 1));
      int anyway_num = (int)from_part->getOperands()[0]->constEval();
      std::string anyway_name = formula_name + trail;

      anyway_mapping.insert({ anyway_num, {} });

      if (!anyway_mapping.at(anyway_num).empty()) {
         std::string other_anyway_name = anyway_mapping.at(anyway_num).front().first;
         possible_anyway_condition = CodeBlockIf::getInstance(
            IfCondition(IfConditionMode::free, other_anyway_name + "->getOptorOnCompoundLevel() != " + anyway_name + "->getOptorOnCompoundLevel()"), 
            std::make_shared<CodeBlockReturn>(false));

         if (optional_index < 0) { // If the anyway is optional, we need to do the check only if it actually exists, i.e., in the below IF.
            function_line->appendAtTheEnd(possible_anyway_condition);
            possible_anyway_condition = nullptr;
         }
      }

      anyway_mapping.at(anyway_num).push_back({ formula_name, trail_orig });
   }
   else {
      std::string overloaded_cond = "";

      if (parser->getOperatorDeclarationState(from_part->getOptor()) == OperatorDeclarationState::overloaded) {
         overloaded_cond = " || " + formula_name + trail + "->getTermsJumpIntoCompounds().size() != " + std::to_string(from_part->getOperands().size());
      }

      condition = std::make_shared<IfCondition>(IfConditionMode::free, formula_name + trail + "->getOptorOnCompoundLevel() != \"" + from_part->getOptor() + "\"" + overloaded_cond);
   }

   int cnt_begin = from_part->toTermAnywayIfApplicable() ? 1 : 0;
   int target_optional_index = optional_index - cnt_begin; // Subtract cnt_begin to account for first index in an anyway being the id.

   if (optional_index >= 0) {
      function_line->appendAtTheEnd(std::make_shared<CodeBlockDecl>(trail_orig, 0, (target_optional_index == 0 ? "This is an optional term." : "")));
      function_line->appendAtTheEnd(std::make_shared<CodeBlockDecl>(trail_orig, 1, (target_optional_index == 1 ? "This is an optional term." : "")));

      auto body_if = std::make_shared<CodeBlockNull>();
      auto body_else = std::make_shared<CodeBlockNull>();

      if (target_optional_index == 0) {
         body_if->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 0, _getCodeFromFormula(from_part->getOperands()[optional_index]->getOperands()[1], true, anyway_mapping, parser_name, true), "", false));
         body_if->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 1, formula_name + trail, "", false));
      }
      else { // optional_index == 1
         body_if->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 0, formula_name + trail, "", false));
         body_if->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 1, _getCodeFromFormula(from_part->getOperands()[optional_index]->getOperands()[1], true, anyway_mapping, parser_name, true), "", false));
      }

      body_else->appendAtTheEnd(possible_anyway_condition);
      body_else->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 0, "", "", false));
      body_else->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 1, "", "", false));

      function_line->appendAtTheEnd(CodeBlockIf::getInstance(*condition, body_if, body_else));
   }
   else {
      std::shared_ptr<CodeBlock> compact_if;

      if (optimize_away_first_condition && trail_orig.empty()) {
         compact_if = std::make_shared<CodeBlockCustom>("// if (" + condition->serialize() + ") return false;");

      }
      else {
         compact_if = CodeBlockIf::getInstance(*condition, std::make_shared<CodeBlockReturn>(false));
      }

      function_line->appendAtTheEnd(compact_if);

      for (int cnt = cnt_begin; cnt < from_part->getOperands().size(); cnt++) {
         function_line->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, cnt - cnt_begin));
      }
   }

   for (int cnt = cnt_begin; cnt < from_part->getOperands().size(); cnt++) {
      if (target_optional_index != cnt - (bool)from_part->toTermAnywayIfApplicable()) {
         auto child{ from_part->getOperands()[cnt - 0] };
         auto new_trail{ trail_orig };
         new_trail.push_back(cnt - (bool)from_part->toTermAnywayIfApplicable());
         auto rest{ createRelevantDefinitionsForMetaRuleCodeNegativeStyle(child, optimize_away_first_condition, formula_name, anyway_mapping, parser_name, parser, new_trail) };
         function_line->appendAtTheEnd(rest);
      }
   }

   return function_line; 
}

std::shared_ptr<CodeBlock> createRelevantDefinitionsForMetaRuleCode(
   const TermPtr from_part_raw,
   const bool optimize_away_first_condition,
   const std::string& formula_name,
   std::map<int, std::vector<std::pair<std::string, std::vector<int>>>>& anyway_mapping,
   const std::string parser_name,
   const std::shared_ptr<FormulaParser> parser,
   bool& deepest,
   const std::vector<int>& trail_orig = {})
{
   auto function_line{ std::make_shared<CodeBlockNull>() };
   auto from_part{ from_part_raw->thisPtrGoIntoCompound() };

   if (from_part->isMetaSimplification()) {
      return function_line;
   }

   IfCondition condition{};

   int optional_index = from_part->getOperands().size() == 2 && from_part->getOperands()[0]->isTermOptional() // Optional only allowed for two-operand terms.
      ? 0
      : (from_part->getOperands().size() >= 2 && from_part->getOperands()[1]->isTermOptional()
         ? 1
         : -1);

   std::shared_ptr<CodeBlockIf> possible_anyway_condition{ nullptr };

   if (from_part->isTermVal()) {
      //condition = "" + formula_name + trail + "->isTermVal() && " + formula_name + trail + "->toValueIfApplicable()->getValue() == " + std::to_string(from_part->toValueIfApplicable()->getValue()) + "";
      condition = IfCondition(IfConditionMode::value_comparison, formula_name, trail_orig, from_part->toValueIfApplicable()->getValue());
   }
   else if (from_part->toTermAnywayIfApplicable()) {
      //condition = formula_name + trail + "->getTerms().size() == " + std::to_string(from_part->getTerms().size() - 1);
      condition = IfCondition(IfConditionMode::equal_comparison,
         IfConditionComparisonSide(IfConditionComparisonSideMode::terms_name_specifier, formula_name, trail_orig),
         IfConditionComparisonSide(IfConditionComparisonSideMode::number, from_part->getOperands().size() - 1)
      );
      int anyway_num = (int)from_part->getOperands()[0]->constEval();

      anyway_mapping.insert({ anyway_num, {} });

      if (!anyway_mapping.at(anyway_num).empty()) {
         //std::string other_anyway_name = anyway_mapping.at(anyway_num).front().first + StaticHelper::getTrailForMetaRuleCodeGeneration(anyway_mapping.at(anyway_num).front().second);
         // other_anyway_name + "->getOptor() == " + anyway_name + "->getOptor()"
         auto other_anyway_info{ anyway_mapping.at(anyway_num).front() };
         auto if_body{ std::make_shared<CodeBlockNull>() };
         IfCondition cond{ IfCondition(IfConditionMode::equal_comparison,
                                       IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name_specifier, other_anyway_info.first, other_anyway_info.second),
                                       IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name_specifier, formula_name, trail_orig)) };

         possible_anyway_condition = CodeBlockIf::getInstance(
            cond,
            if_body);

         if_body->setFather(possible_anyway_condition);
      }

      anyway_mapping.at(anyway_num).push_back({ formula_name, trail_orig });
   }
   else {
      std::string overloaded_cond{ "" };

      if (parser->getOperatorDeclarationState(from_part->getOptor()) == OperatorDeclarationState::overloaded) {
         //overloaded_cond = " && " + formula_name + trail + "->getTerms().size() == " + std::to_string(from_part->getTerms().size());
         condition = IfCondition(IfConditionMode::overloaded_comparison, formula_name, trail_orig, from_part->getOptor(), from_part->getOperands().size());
      }
      else {
         //condition = formula_name + trail + "->getOptor() == \"" + from_part->getOptor() + "\"" + overloaded_cond;
         condition = IfCondition(IfConditionMode::equal_comparison,
            IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name_specifier, formula_name, trail_orig),
            IfConditionComparisonSide(IfConditionComparisonSideMode::operator_name, from_part->getOptor())
         );
      }

   }

   int cnt_begin = from_part->toTermAnywayIfApplicable() ? 1 : 0;
   int target_optional_index = optional_index - cnt_begin; // Subtract cnt_begin to account for first index in an anyway being the id.
   bool is_optional{ optional_index >= 0 };

   auto body_if = std::make_shared<CodeBlockNull>();
   auto body_else = std::make_shared<CodeBlockNull>();
   auto if_line{ CodeBlockIf::getInstance(condition, body_if, body_else) };
   body_if->setFather(if_line);
   body_else->setFather(if_line);

   if (is_optional) {
      function_line->appendAtTheEnd(std::make_shared<CodeBlockDecl>(trail_orig, 0, (target_optional_index == 0 ? "This is an optional term." : "")));
      function_line->appendAtTheEnd(std::make_shared<CodeBlockDecl>(trail_orig, 1, (target_optional_index == 1 ? "This is an optional term." : "")));
      const std::string trail{ CodeGenerator::getTrailForMetaRuleCodeGeneration(trail_orig) };

      if (target_optional_index == 0) {
         body_else->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 0, _getCodeFromFormula(from_part->getOperands()[optional_index]->getOperands()[1], true, anyway_mapping, parser_name, true), "", false));
         body_else->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 1, formula_name + trail, "", false));
      }
      else { // optional_index == 1
         body_else->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 0, formula_name + trail, "", false));
         body_else->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 1, _getCodeFromFormula(from_part->getOperands()[optional_index]->getOperands()[1], true, anyway_mapping, parser_name, true), "", false));
      }

      body_if->appendAtTheEnd(possible_anyway_condition);
      body_if->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 0, "", "", false));
      body_if->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, 1, "", "", false));

      function_line->appendAtTheEnd(if_line);
   }
   else {
      // TODO: If we want to optimize away first condition (although it doesn't seem to make a difference at all).
      //if (optimize_away_first_condition && trail_orig.empty()) {
      //   compact_if = std::make_shared<CodeBlockCustom>("// if (" + condition + ") return false;");
      //}
      //else {
      //}

      if (possible_anyway_condition) {
         possible_anyway_condition->setBodyIf(if_line);
         function_line->appendAtTheEnd(possible_anyway_condition);
      }
      else {
         function_line->appendAtTheEnd(if_line);
      }

      for (int cnt = cnt_begin; cnt < from_part->getOperands().size(); cnt++) {
         body_if->appendAtTheEnd(std::make_shared<CodeBlockDef>(trail_orig, cnt - cnt_begin));
      }
   }

   //auto s{function_line->serializeBlock()}; // Comment in to get address of serializeBlock function to debugger.

   auto num_terms{ from_part->getOperands().size() };

   if (num_terms > 0) {
      for (int cnt = cnt_begin; cnt < num_terms; cnt++) {
         if (target_optional_index != cnt - (bool)from_part->toTermAnywayIfApplicable()) {
            auto child{ from_part->getOperands()[cnt - 0] };
            auto new_trail{ trail_orig };
            new_trail.push_back(cnt - (bool)from_part->toTermAnywayIfApplicable());

            function_line->applyToMeAndMyChildren([&deepest](const std::shared_ptr<CodeBlock> line) // Nest deeper if placeholder already available.
               {
                  if (line->toPlaceholderIfApplicable()) {
                     deepest = true;
                  }
               }, deepest);

            auto rest = createRelevantDefinitionsForMetaRuleCode(child, optimize_away_first_condition, formula_name, anyway_mapping, parser_name, parser, deepest, new_trail);

            if (!rest->isEmptyFromThisLineOn()) {
               // Insert rest into the this function_line.

               auto first_non_empty_line_of_rest{ rest->goToNextNonEmptyLine() };
               assert(first_non_empty_line_of_rest);
               auto first_line_of_rest_if{ first_non_empty_line_of_rest->toIfIfApplicable() };

               if (first_line_of_rest_if && first_line_of_rest_if->getCondition().isAnywayConditionSecondPart()) {
                  // Squash "anyway" conditions of types "size == ..." and "optor == optor".
                  auto first_line_of_function{ function_line->goToNextNonEmptyLine() };
                  assert(first_line_of_function);
                  assert(first_line_of_function->toIfIfApplicable()); // If it's an "anyway" situation, this needs to hold.
                  auto first_line_of_function_if{ first_line_of_function->toIfIfApplicable() };
                  assert(first_line_of_function_if->getCondition().isAnywayConditionFirstPart());

                  auto main_cond_sides{ first_line_of_function->toIfIfApplicable()->getCondition().getSides() }; // the "formula->getTerms().size() == 2.000000" part
                  auto sub_cond_sides{ first_line_of_rest_if->getCondition().getSides() };                       // the "formula->getOptor() == formula_0->getOptor()" part

                  //const std::vector<int>& trail1, const std::vector<int>& trail2, const std::vector<int>& trail3, const float value);
                  auto squashed_condition{ IfCondition(
                     IfConditionMode::anyway_comparison,
                     main_cond_sides[0].getFormulaName(),
                     main_cond_sides[0].getTrail(),
                     sub_cond_sides[0].getTrail(),
                     sub_cond_sides[1].getTrail(),
                     main_cond_sides[1].getNumber()
                  )};

                  first_line_of_function_if->setCondition(squashed_condition);
                  first_line_of_function_if->getBodyIf()->appendAtTheEnd(first_line_of_rest_if->getBodyIf());
               }
               else {
                  bool breakitoff{ false };
                  function_line->applyToMeAndMyChildren([&rest, &breakitoff](const std::shared_ptr<CodeBlock> line)
                     {
                        if (line->toPlaceholderIfApplicable()) {
                           line->replace(rest);
                           breakitoff = true;
                        }
                     }, breakitoff);

                  if (!breakitoff) {
                     (is_optional ? function_line : body_if)->appendAtTheEnd(rest);
                  }
               }
            }
         }
      }
   }

   if (deepest) {
      // Only at deepest level, insert denoter for where remaining logic needs to be placed at caller side.
      (is_optional ? function_line : body_if)->appendAtTheEnd(std::make_shared<CodeBlockPlaceholder>());
      deepest = false;
   }

   return function_line;
}

void findTrailsWithSpecificMetaNumberForMetaRuleCode(
   const TermPtr from_part,
   const std::string& formula_name,
   const int meta_num, 
   std::vector<std::string>& trails,
   const std::string& trail = "")
{
   if (from_part->isTermCompound()) {
      findTrailsWithSpecificMetaNumberForMetaRuleCode(from_part->getOperands()[0], formula_name, meta_num, trails, trail);
      return;
   }

   if (from_part->isMetaSimplification() && (int)from_part->getOperands()[0]->constEval() == meta_num) {
      if (from_part->getFather() && from_part->getFather()->isTermOptional()) {
         std::string new_trail{ trail.substr(0, trail.find_last_of('_')) };
         trails.push_back(new_trail);
      }
      else {
         trails.push_back(trail);
      }

      return;
   }

   int cnt_begin{ from_part->toTermAnywayIfApplicable() ? 1 : 0 };

   for (int cnt = cnt_begin; cnt < from_part->getOperands().size(); cnt++) {
      auto child = from_part->getOperands()[cnt];
      std::string new_trail = trail + "_" + std::to_string(cnt - cnt_begin);
      findTrailsWithSpecificMetaNumberForMetaRuleCode(child, formula_name, meta_num, trails, new_trail);
   }
}

std::string equalityConditionsForMetaRuleCode(const std::vector<std::string>& trails, const std::string& formula_name)
{
   std::string s;

   for (int i = 0; i < trails.size(); i++) {
      for (int j = i + 1; j < trails.size(); j++) {
         s += (i != 0 || j != 1 ? std::string(" && ") : std::string("")) + formula_name + trails[i] + "->isStructurallyEqual(" + formula_name + trails[j] + ")";
      }
   }

   return s;
}

std::shared_ptr<code_block::CodeBlock> CodeGenerator::createCodeFromMetaRule(
   const MetaRule& rule, 
   const CodeGenerationMode code_generation_mode,
   const bool optimize_away_first_condition,
   const std::string& formula_name,
   const std::string& parser_name, 
   const std::shared_ptr<FormulaParser> parser)
{
   static constexpr int MAX_META_NUM_TO_CONSIDER{ 100 }; // In practice, no rule ever had more than 5 metas.
   std::map<int, std::vector<std::pair<std::string, std::vector<int>>>> anyway_mapping{};

   auto func = [&formula_name](const TermPtr current_term, const std::vector<std::vector<std::string>>& all_trails) {
      current_term->applyToMeAndMyChildrenIterative([&all_trails, &formula_name](const MathStructPtr m) {
         if (m->isMetaSimplification()) {
            m->replace(_var(formula_name + all_trails[(int)m->getOperands()[0]->constEval()][0]));
         }
      }, TraverseCompoundsType::avoid_compound_structures);
   };

   auto function_line{ std::make_shared<CodeBlockNull>() };

   auto real_rule{ rule.copy(true, true, true) };
   real_rule.convertVeriablesToMetas();
   auto real_rule_copy{ real_rule.copy(true, true, true) };

   bool deepest{ true };
   auto definitions{ code_generation_mode == CodeGenerationMode::positive
      ? createRelevantDefinitionsForMetaRuleCode(
         real_rule.getFrom(), 
         false, // Currently optimizing away first condition is not implemented in the positive case.
         formula_name, 
         anyway_mapping, 
         parser_name, 
         parser, 
         deepest) 
      : createRelevantDefinitionsForMetaRuleCodeNegativeStyle(
         real_rule.getFrom(),
         optimize_away_first_condition,
         formula_name,
         anyway_mapping,
         parser_name,
         parser)
   };

   function_line->appendAtTheEnd(definitions);

   std::string condition{};

   std::vector<std::vector<std::string>> all_trails{};

   for (int i = 0; i < MAX_META_NUM_TO_CONSIDER; i++) {
      all_trails.push_back({});
      findTrailsWithSpecificMetaNumberForMetaRuleCode(real_rule.getFrom(), formula_name, i, all_trails[i]);
      condition += equalityConditionsForMetaRuleCode(all_trails[i], formula_name);
   }

   for (const auto& el : real_rule.getConditions()) {
      auto current_term = _id(el.first->copy());
      auto current_cond = el.second;
      func(current_term, all_trails);
      condition += " && MetaRule::" + ConditionTypeWrapper(current_cond).getEnumAsString() + "("
         + _getCodeFromFormula(current_term->getOperands()[0], true, anyway_mapping, parser_name, false) + ")";
   }

   condition += " && MetaRule::" + ConditionTypeWrapper(real_rule.getGlobalCondition()).getEnumAsString() + "(" + formula_name + ")";

   StaticHelper::stripAtBeginning(condition, { ' ', '\t', '\r', '\n', '&' });

   auto real_to = _id(real_rule.getTo());
   func(real_to, all_trails);

   auto body = std::make_shared<CodeBlockNull>();
   //body->appendAtTheEnd(std::make_shared<CodeBlockCustom>("// " + rule.serialize())); // TODO: For merged code from several rules, this comment is useful.

   auto call = _getCodeFromFormula(real_to->getOperands()[0], false, anyway_mapping, parser_name, true);
   std::string comment_out_replacement_before{ rule.containsModifyingCondition() ? "// " : "" };
   std::string comment_out_replacement_after{ rule.containsModifyingCondition() ? " // Commented out due to modifying condition. Currently, rules with modifying conditions need to be of type x ==> x." : "" };

   if (call.empty()) {
      return nullptr; // If there is no _*func* operator for the given rule.
   }

   body->appendAtTheEnd(std::make_shared<CodeBlockCustom>(comment_out_replacement_before + formula_name + "->replace(" + call + ");" + comment_out_replacement_after));
   body->appendAtTheEnd(std::make_shared<CodeBlockReturn>(true));

   if (code_generation_mode == CodeGenerationMode::positive) { // This is the default.
      auto if_line{ CodeBlockIf::getInstance(IfCondition(IfConditionMode::free, condition), body) };
      body->setFather(if_line);

      bool breakitoff{ false };

      definitions->applyToMeAndMyChildren([&if_line, &breakitoff](const std::shared_ptr<CodeBlock> line)
         {
            if (line->toPlaceholderIfApplicable()) {
               line->replace(if_line);
               breakitoff = true;
            }
         }, breakitoff);
   }
   else {
      function_line->appendAtTheEnd(CodeBlockIf::getInstance(IfCondition(IfConditionMode::free, condition), body));
   }

   function_line->appendAtTheEnd(std::make_shared<CodeBlockReturn>(false));

   return function_line;
}

std::string CodeGenerator::createCodeFromMetaRuleString(const MetaRule& rule, const CodeGenerationMode mode, const std::string& name, const bool optimize_away_first_condition, const std::string& parser_name, const std::shared_ptr<FormulaParser> parser)
{
   std::string s;

   s += "// Autogenerated function for meta rule:\n";
   s += "// " + rule.serialize() + "\n";
   s += "inline bool apply_" + name + "(const TermPtr formula, const std::shared_ptr<FormulaParser> " + parser_name + ")\n{\n";

   auto code = createCodeFromMetaRule(rule, mode, optimize_away_first_condition, "formula", parser_name, parser);

   if (!code) {
      return "";
   }

   s += code->serializeBlock(3, "formula");

   // TODO: Omitting the test for each single function for now; seems to be not so relevant.
   //std::string t = "\n// Autogenerated TEST for autogenerated function for meta rule:\n";
   //t += "// " + rule.serialize() + "\n";
   //t += "inline bool apply_" + name + "_TEST()\n{\n";
   //t += "   auto formula1 = SingletonFormulaParser::getInstance()->termFactoryDummy(\"" + real_rule.getFrom()->getOptor() + "\");\n\n";
   //t += "   for (const auto& term : formula1->getTerms()) {\n";
   //t += "      term->replace(MathStruct::randomTerm({\"x\", \"y\", \"z\"}, 5));\n";
   //t += "   }\n\n";
   //t += "   formula1 = _id(formula1);\n";
   //t += "   auto formula2 = formula1->copy();\n";
   //t += "   MetaRule r(\n";
   //t += "      " + _getCodeFromFormula(real_rule_copy.getFrom(), false) + ",\n";
   //t += "      " + _getCodeFromFormula(real_rule_copy.getTo(), false) + ",\n";

   //std::string conditions;
   //for (const auto& cond : real_rule_copy.getConditions()) {
   //   conditions += "{" + _getCodeFromFormula(cond.first, false) + ", ConditionType::" + ConditionTypeWrapper(cond.second).getEnumAsString() + "}, ";
   //}

   //t += "      { " + conditions + " },\n";
   //t += "      ConditionType::" + ConditionTypeWrapper(real_rule.getGlobalCondition()).getEnumAsString() + ");\n";
   //t += "   \n";
   //t += "}\n";

   s += "}\n";
   return s;
}

std::string CodeGenerator::createHeadPartOfSimplificationCode(
   const bool mc_mode,
   const CodeGenerationMode code_generation_mode,
   const std::set<vfm::MetaRule>& abandoned_rules,
   const std::set<vfm::MetaRule>& additional_rules,
   const std::map<std::string, std::map<int, std::set<MetaRule>>>& all_used_rules,
   const std::shared_ptr<FormulaParser> parser)
{
   std::string s{};
   const std::time_t timestamp{ std::time(nullptr) };
   const std::string timestamp_str{ std::string(std::asctime(std::localtime(&timestamp))) };

   std::string about_overloaded_operators{};
   about_overloaded_operators += "\n// Unique operators at creation time. Overloading one of these might lead to errors in the simplification if they are part of a simplification rule.\n\
static const std::set<std::string> NON_OVERLOADED_OPERATORS = { ";

   for (const auto& op : parser->getAllOps()) {
      if (parser->getOperatorDeclarationState(op.first) == OperatorDeclarationState::unique) {
         about_overloaded_operators += "\"" + op.first + "\", ";
      }
   }

   about_overloaded_operators += "};\n";
   about_overloaded_operators += "\n// Overloaded operators at creation time.\n\
static const std::set<std::string> OVERLOADED_OPERATORS = { ";

   for (const auto& op : parser->getAllOps()) {
      if (parser->getOperatorDeclarationState(op.first) == OperatorDeclarationState::overloaded) {
         about_overloaded_operators += "\"" + op.first + "\", ";
      }
   }

   about_overloaded_operators += "};\n";
   about_overloaded_operators += "\n// All operators used in \"from\" part of simplification rules.\n\
static const std::set<std::string> USED_OPERATORS = { ";

   std::set<std::string> used_operators{};

   for (const auto& rule_op_set : all_used_rules) {
      for (const auto& rule_set : rule_op_set.second) {
         for (const auto& rule: rule_set.second) {
            rule.getFrom()->applyToMeAndMyChildren([&used_operators](const MathStructPtr m) {
               if (!m->isTermVal() && !m->isTermVal()) {
                  used_operators.insert(m->getOptor());
               }
            });
         }
      }
   }

   for (const auto& op : used_operators) {
      about_overloaded_operators += "\"" + op + "\", ";
   }

   about_overloaded_operators += "};\n";

   s += "// Auto-generated file using vfm's 'createCodeForAllSimplificationRules' function.\n";

   if (mc_mode) {
      s += "\n// -----------------------------------------------------MC MODE-----------------------------------------------------------\n";
      s += "// The mc_mode flag has been set to true. This simplification abandons AND adds functions wrt. the regular simplification:\n";
      s += "//   - (ABANDON) Constants will be preserved, i.e., not included in simplification of constant subtrees.\n";

      for (const auto& rule : abandoned_rules) {
         s += "//   - (ABANDON) Rule abandoned for '" + rule.getFrom()->getOptorOnCompoundLevel() + "': " + rule.serialize() + ".\n";
      }

      for (const auto& rule : additional_rules) {
         s += "//   - (ADD) Additional rule for '" + rule.getFrom()->getOptorOnCompoundLevel() + "': " + rule.serialize() + ".\n";
      }

      s += "// -----------------------------------------------------------------------------------------------------------------------\n\n";
   }

   s += "// Creation time: ";
   s += timestamp_str;

   s += "\n// Usage note: There are three simplification modes in vfm (only two are currently working) which have the following benefits (if in doubt use 2):\n"
      "// 1) Runtime simplification via " + SIMPLIFICATION_REGULAR_FUNCTION_NAME_FULL + "(): Slowest, but can be triggered to provide detailed debugging information.\n"
      "//    Allows adding and removing rules at runtime, e.g. via runInterpreter().\n"
      "// 2) Hard-coded simplification via " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAME_FULL_POS : SIMPLIFICATION_FAST_FUNCTION_NAME_FULL_NEG) + "() in this file: About 300x faster than runtime version.\n"
      "//    Very close to a theoretical speed limit, it appears (possibly still marginally slower than " + SIMPLIFICATION_VERY_FAST_FUNCTION_NAME + "(), once established).\n"
      "//    Can be triggered to individually apply simplification rules to a formula via applyToFullFormula() function below.\n"
      "//    Available in 'positive' (" + SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS + ") and 'negative' (" + SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG + ") logic style, which are very similar in terms of runtime, though.\n"
      "//    If in doubt, use the negative version of this function (" + SIMPLIFICATION_FAST_FUNCTION_NAME_FULL_NEG + ") for full-feature simplification, because it's slightly faster.\n"
      "// 3) Hard-coded simplification via " + SIMPLIFICATION_VERY_FAST_FUNCTION_NAME_FULL + "() in this file: Possibly up to 1.2x faster than regular hard-coded version, but monolithic.\n"
      "//    CAUTION: THIS FUNCTION IS CURRENTLY UNDER CONSTRUCTION!\n";

   s += R"(
#pragma once
#include "term.h"
#include "meta_rule.h"
#include "parser.h"

namespace vfm {
)";

   if (mc_mode) {
      s += "namespace mc {\n";
   }

   s += "namespace " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS : SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG) + " {\n";

   s += about_overloaded_operators;

   s += R"(
// Autogenerated function to repeatedly apply a subset of meta rules to a whole formula, until no changes occur anymore.)";

   if (code_generation_mode == CodeGenerationMode::negative && OPTIMIZE_AWAY_FIRST_CONDITION_FOR_FAST_SIMPLIFICATION) { // Currently not implemented for positive logic.
      s += "\n"
         "//\n"
         "// Caution: OPTIMIZE_AWAY_FIRST_CONDITION_FOR_FAST_SIMPLIFICATION is active, you'll not be able to use the rule-wise functions below out of the box\n"
         "// (this does not affect " + SIMPLIFICATION_FAST_FUNCTION_NAME + "() and " + SIMPLIFICATION_VERY_FAST_FUNCTION_NAME + "()).\n"
         "// Instead of:\n"
         "//    " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS : SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG) + "::applyToFullFormula(formula, { " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS : SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG) + "::apply__mult_31 });\n"
         "// You'll need to do, e.g.:\n"
         "//    " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS : SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG) + "::applyToFullFormula(formula, { [](const TermPtr t, const std::shared_ptr<FormulaParser> p) {\n"
         "//       return t->getOptor() != \"*\" ? false : " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS : SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG) + "::apply__mult_31(t, p);\n"
         "//    } });";
   }

   s += R"(
inline TermPtr applyToFullFormula(
   const TermPtr formula_raw, 
   const std::vector<std::function<bool(const TermPtr, const std::shared_ptr<FormulaParser>)>>& funcs, 
   const std::shared_ptr<FormulaParser> parser_raw = nullptr,
   const bool eval_constant_subterms = false) 
{
   auto parser{ parser_raw ? parser_raw : SingletonFormulaParser::getInstance() };
   auto formula{ _id(formula_raw) };
   bool changed{ true };

   while (changed) {
      changed = false;
      formula->applyToMeAndMyChildrenIterative([&funcs, &changed, &parser, eval_constant_subterms](const MathStructPtr m) {
)";

   if (mc_mode) {
      s += R"(
      if (eval_constant_subterms && !MetaRule::is_leaf(m) && !m->isCompoundOperator() && m->consistsPurelyOfNumbers()
            && m->getOptor() != "==" // TODO: This needs to be solved more generally. It prevents "0 == 0" from becoming "1" (instead of "true").
         ) {
)";
   }
   else {
      s += R"(
      if (eval_constant_subterms && !MetaRule::is_leaf(m) && !m->isCompoundOperator() && m->isOverallConstant()) {)";
   }

   s += R"(
         m->replace(_val_plain(m->constEval()));
         changed = true;
      }
      else {
         for (const auto& func : funcs)
            changed = func(m->toTermIfApplicable(), parser) || changed;
      }
)";

   s += R"(
      }, TraverseCompoundsType::avoid_compound_structures, FormulaTraversalType::PostOrder);
   }

   return formula->child0();
}

)";

   return s;
}

std::string CodeGenerator::createFootPartOfSimplificationCode(const CodeGenerationMode code_generation_mode, const bool mc_mode)
{
   return std::string("\n")
      + "} // " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS : SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG) + "\n"
      + (mc_mode ? "} // mc\n" : "")
      + "} // vfm\n";
}

const std::string OVERLOADED_WARNING_CODE{ R"(
   for (const auto& op : p->getAllOps()) {
      if (!OVERLOADED_OPERATORS.count(op.first) && p->getNumParams(op.first).size() > 1 && USED_OPERATORS.count(op.first)) {
         p->addWarning("(W0001) Operator '" + op.first + "' is overloaded, but has not been so at creation time of hard-coded simplification; this could lead to errors. You'll need to recreate the simplification function using the new parser.");
      }
   }
)"
};

std::string CodeGenerator::addTopLevelSimplificationFunction(
   const CodeGenerationMode code_generation_mode,
   const std::string& inner_part_before_raw,
   const std::vector<std::pair<int, std::string>>& inner_part_raw,
   const std::string& inner_part_after_raw,
   const bool very_fast)
{
   const std::string function_name{ very_fast ? SIMPLIFICATION_VERY_FAST_FUNCTION_NAME : SIMPLIFICATION_FAST_FUNCTION_NAME };
   const std::string same_as_notification{ SIMPLIFICATION_REGULAR_FUNCTION_NAME_FULL + (very_fast ? " and " + (code_generation_mode == CodeGenerationMode::positive ? SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_POS : SIMPLIFICATION_FAST_FUNCTION_NAMESPACE_NEG) + "::simplify" : "") };
   const std::string inner_part_before{ very_fast ? inner_part_before_raw + "            else {\n" : inner_part_before_raw };
   const std::string inner_part_after{ very_fast ? "            }\n" + inner_part_after_raw : inner_part_after_raw };
   const std::vector<std::pair<int, std::string>> inner_part{ very_fast
      ? std::vector<std::pair<int, std::string>>
        { {0, "               // This part is commented out for now due to something not working\n"
              "               // in the 'mergeWithOtherSimplificationFunction' function. Apart from\n"
              "               // some most probably minor issue, the most problematic logical problem\n"
              "               // is the exponential blowup when two conditions from separate rules are\n"
              "               // independent of each other, such as 'formula_0->getOptor() == \"*\"'\n"
              "               // vs. 'formula_1->getOptor() == \"+\"', which needs to be solved somehow.\n"
              "               // One possibility is to append such rules after each other, but this requires\n"
              "               // a fundamentally different approach.\n"
              "               // \n"
              "               // Note, however, that the expected speedup is very limited, measured so far\n"
              "               // between about 1.01 and 1.5.\n" } }
      : inner_part_raw };
   std::string s{};

   s += "// Auto-generated simplification function. When freshly generated, it should\n";
   s += "// do the same as " + same_as_notification + ", only faster.\n";
   s += "inline TermPtr " + function_name + "(const TermPtr formula_raw, const std::shared_ptr<FormulaParser> p_raw = nullptr)\n"; // TODO: p shouldn't be hard-coded since it must be the same as in createInnerPartOfSimplificationCode.
   s += "{\n";
   s += "   //static std::string last = \"\";\n";
   s += "   //std::cout << \" ---\\n\";\n";
   s += "   auto p = p_raw ? p_raw : SingletonFormulaParser::getInstance();\n";
   s += OVERLOADED_WARNING_CODE + "\n";
   s += "   bool changed;\n";
   s += "   auto formula = _id(formula_raw);\n\n";

   for (const auto& el : inner_part) {
      s += "   //std::cout << \"Entering stage #\" << " + std::to_string(el.first) + " << \" of simplification.\" << std::endl;\n";
      s += "   changed = true;\n";
      s += "   while (changed) {\n";
      s += "      changed = false;\n";
      s += "      //formula->checkIfAllChildrenHaveConsistentFatherQuiet();\n\n";
      s += "      formula->getOperands()[0]->applyToMeAndMyChildrenIterative([&changed, p](const MathStructPtr m) {\n";
      s += ""
         "         //std::string ser = m->getPtrToRoot()->getOperands()[0]->serialize(m);\n"
         "         //if (last != ser) {\n"
         "         //   std::cout << \"" + std::string(very_fast ? "VERY" : "    ") + " FAST:   \" << ser << std::endl;\n"
         "         //   last = ser;\n"
         "         //}\n";
      s += inner_part_before + el.second + inner_part_after;
      s += "      }, TraverseCompoundsType::go_into_compound_structures, FormulaTraversalType::PostOrder);\n";
      s += "   }\n\n";
   }

   s += "   return formula->getOperands()[0];\n";
   s += "}\n\n";

   return s;
}

std::string CodeGenerator::createInnerPartOfSimplificationCode(
   const CodeGenerationMode code_generation_mode,
   const std::map<std::string, std::map<int, std::set<vfm::MetaRule>>>& all_rules,
   const std::shared_ptr<vfm::FormulaParser> parser,
   std::string& inner_part_before,
   std::vector<std::pair<int, std::string>>& inner_part_fast_raw,
   std::vector<std::pair<int, std::string>>& inner_part_very_fast_raw,
   std::string& inner_part_after,
   const bool mc_mode)
{
   static const std::string PARSER_NAME{ "p" };
   int i = 0;
   std::string s{};

   for (const auto& by_stage : MetaRule::getMetaRulesByStage(all_rules)) {
      std::shared_ptr<CodeBlock> all_rules_code{};
      std::map<int, std::set<std::string>> par_num_to_anyway_name{};
      const int stage_num{ by_stage.first };
      std::string inner_part_fast{};
      std::string inner_part_very_fast{};

      for (const auto& els : by_stage.second) {
         const bool is_overloaded_operator{ els.second.size() > 1 };
         const std::string op_name_outside{ els.first };
         bool has_any_rules{ !els.second.empty() };
         TermPtr any_from{};

         if (has_any_rules) {
            has_any_rules = false;

            for (const auto& r : els.second) {
               if (!r.second.empty()) {
                  has_any_rules = true;
                  any_from = r.second.begin()->getFrom();
               }
            }
         }

         if (has_any_rules && op_name_outside != SYMB_ANYWAY) {
            inner_part_fast += "            else if (m->getOptor" + std::string(any_from->isTermCompound() || is_overloaded_operator ? "OnCompoundLevel" : "") + "() == \"" + op_name_outside + "\") {\n";
         }

         std::string first_if{ "" };

         for (const auto& el : els.second) {
            std::string additional_indentation{ "" };

            if (is_overloaded_operator && !el.second.empty()) {
               additional_indentation = "   ";
               inner_part_fast += "               " + first_if + "if (m->getTermsJumpIntoCompounds().size() == " + std::to_string(el.first) + ") {\n";
               first_if = "else ";
            }

            for (const auto& el2 : el.second) {
               std::string op{ el2.getFrom()->getOptorJumpIntoCompound() };
               std::string func_name{ StaticHelper::replaceManyTimes(op) };
               auto full_name{ func_name + std::to_string(i) };
               assert(op == op_name_outside);

               if (el2.isRuleAbandoned()) {
                  parser->addNote("Abandoning rule " + std::to_string(i) + " for '" + func_name + "'" + (op == func_name ? "" : " ('" + op + "')") + ": " + el2.serialize());
                  s += "// Abandoned rule for '" + full_name + "'.\n\n";
               } else {
                  parser->addNote("Generating rule " + std::to_string(i) + " for '" + func_name + "'" + (op == func_name ? "" : " ('" + op + "')") + ": " + el2.serialize());

                  // Fast Part.
                  if (i == 6) {
                     int x{};
                  }

                  auto func_code{ createCodeFromMetaRuleString(el2, code_generation_mode, full_name, OPTIMIZE_AWAY_FIRST_CONDITION_FOR_FAST_SIMPLIFICATION, PARSER_NAME, parser) };

                  if (func_code.empty()) {
                     parser->addWarning("Rule '" + el2.serialize() + "' could not be generated. Skipping this rule.");
                     std::string error_comment{ "// Rule '" + el2.serialize() + "' skipped. Couldn't figure out how to create it.\n\n" };
                     s += error_comment;
                     inner_part_fast += "               " + error_comment;
                  }
                  else {
                     s += func_code + "\n";

                     if (op == SYMB_ANYWAY) {
                        int par_num_anyway{ (int)el2.getFrom()->getOperands().size() - 1 };
                        par_num_to_anyway_name.insert({ par_num_anyway, {} });
                        par_num_to_anyway_name.at(par_num_anyway).insert(full_name);
                     }
                     else {
                        inner_part_fast += additional_indentation + "               changed_this_time = apply_" + full_name + "(m->toTermIfApplicable(), " + PARSER_NAME + ");\n";
                        inner_part_fast += additional_indentation + "               if (changed_this_time) break;\n";
                     }
                  }
                  // EO Fast Part. 

                  // Very Fast Part.
                  if (all_rules_code) {
                     //el2.getFrom()->applyToMeAndMyChildrenIterative([](const MathStructPtr m) {
                     //   if (m->toTermOptionalIfApplicable()) {
                     //      std::cout << std::endl;
                     //   }
                     //   });
#ifdef ACTIVATE_VERY_FAST_SIMPLIFICATION
                     all_rules_code->mergeWithOtherSimplificationFunction(createCodeFromMetaRule(el2, code_generation_mode, false, "m", PARSER_NAME, parser), *parser);
#endif
                  }
                  else {
                     all_rules_code = createCodeFromMetaRule(el2, code_generation_mode, false, "m", PARSER_NAME, parser);
                  }
                  // EO Very Fast Part.
               }

               i++;
            }

            if (is_overloaded_operator && !el.second.empty()) {
               inner_part_fast += "               }\n";
            }
         }

         if (has_any_rules && op_name_outside != SYMB_ANYWAY) {
            inner_part_fast += "            }\n";
         }
      }

      all_rules_code->removeAllRedefinitionsOrReinitializations();

      all_rules_code->addNote("Checking integrity for code with '" + std::to_string(all_rules_code->nodeCount()) + "' nodes.");
      if (all_rules_code->checkIntegrity()) {
         all_rules_code->addNote("Integrity is fine.");
      }
      else {
         all_rules_code->addError("Integrity is screwed up.");
      }

      inner_part_very_fast = all_rules_code ? all_rules_code->serializeBlock(15, "m") : "";

      std::string else_except_first;

      for (const auto& el : par_num_to_anyway_name) {
         inner_part_fast += "            if (!changed_this_time) {\n";
         inner_part_fast += "               " + else_except_first + "if (m->getTermsJumpIntoCompounds().size() == " + std::to_string(el.first) + ") {\n";

         for (const auto& el2 : el.second) {
            inner_part_fast += "                  changed_this_time = apply_" + el2 + "(m->toTermIfApplicable(), " + PARSER_NAME + ");\n";
            inner_part_fast += "                  if (changed_this_time) break;\n";
         }

         inner_part_fast += "               }\n";
         inner_part_fast += "            }\n";
         else_except_first = "else ";
      }

      inner_part_very_fast_raw.push_back({ stage_num, inner_part_very_fast });
      inner_part_fast_raw.push_back({ stage_num, inner_part_fast });
   }

   inner_part_before += R"(         if (MetaRule::is_leaf(m)) return; // Caution, remove if simplifications on leaf level desired.
         bool changed_this_time = false;

         for (int i = 0; i < 1 && m->getFather(); i++) {
            if (!m->isCompoundOperator() && )";
   inner_part_before += mc_mode ? "!m->isTermCompound() && m->consistsPurelyOfNumbers()" : "m->isOverallConstant()";
   inner_part_before += R"() {
               m->replace(_val_plain(m->constEval()));
               changed_this_time = true; // Since it's not a leaf, there was definitely a change.
            }
)";

   inner_part_after += R"(         }

         changed = changed || changed_this_time;
)";

   return s;
}

std::string CodeGenerator::createCodeForAllSimplificationRules(
   const CodeGenerationMode code_generation_mode, 
   const std::shared_ptr<FormulaParser> parser_raw, 
   const bool mc_mode)
{
   std::shared_ptr<FormulaParser> parser = parser_raw;

   if (!parser) {
      parser = std::make_shared<FormulaParser>();
      parser->addDefaultDynamicTerms();
   }

   parser->addNote("Generating code for all simplification rules" + std::string(mc_mode ? " (MC mode)" : "") + ".");

   std::string inner_part_before{};
   std::vector<std::pair<int, std::string>> inner_part_fast{};
   std::vector<std::pair<int, std::string>> inner_part_very_fast{};
   std::string inner_part_after{};
   auto all_rules{ parser->getAllRelevantSimplificationRulesOrMore() };

   std::set<vfm::MetaRule> abandoned_rules{ mc_mode
      ? std::set<vfm::MetaRule>{
         MetaRule("a == b ==> !(a) [[b: 'is_constant_and_evaluates_to_false', a: 'contains_no_meta']]", -1),
         MetaRule("a == b ==> !(b) [[a: 'is_constant_and_evaluates_to_false', b: 'contains_no_meta']]", -1),
         MetaRule("a != b ==> boolify(a) [[b: 'is_constant_and_evaluates_to_false', a: 'contains_no_meta']]", -1),
         MetaRule("a ~ b ==> a - b ~ 0 [[a ~ b: 'is_comparator_or_equality_operation', b: 'is_not_constant']]", -1),
         MetaRule("a ~ b ==> a - b ~ 0 [[a ~ b: 'is_comparator_or_equality_operation', b: 'is_constant_and_evaluates_to_true']]", -1),
         MetaRule(_not(_not(_vara())), _boolify(_vara()), -1)
   }
   : std::set<vfm::MetaRule>{} };

   std::set<vfm::MetaRule> additional_rules{ mc_mode
      ? std::set<vfm::MetaRule>{
      MetaRule(_seq(_vara(), _varb()), _vara(), SIMPLIFICATION_STAGE_MAIN, { { _varb(), ConditionType::has_no_sideeffects } }),
      MetaRule(_seq(_vara(), _varb()), _varb(), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::has_no_sideeffects } }),
      MetaRule(_ifelse(_vara(), _varb(), _varc()), _if(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN, { { _varc(), ConditionType::has_no_sideeffects } }),
      MetaRule(_ifelse(_vara(), _varb(), _varc()), _if(_not(_vara()), _varc()), SIMPLIFICATION_STAGE_MAIN, { { _varb(), ConditionType::has_no_sideeffects } }),
      MetaRule(_not(_not(_vara())), _vara(), SIMPLIFICATION_STAGE_MAIN),
   }
   : std::set<vfm::MetaRule>{} };

   for (auto rule : additional_rules) {
      rule.convertVeriablesToMetas();
      std::string optor{ rule.getFrom()->getOptorJumpIntoCompound() };
      size_t opnum{ rule.getFrom()->getTermsJumpIntoCompounds().size() };

      if (!all_rules.count(optor)) {
         all_rules.insert({ optor, {} });
      }

      all_rules.at(optor).at(opnum).insert(rule);
   }

   int abandoned_counter{ 0 };
   std::set<std::string> abandonend_rules_found{};

   for (auto rule_to_abandon : abandoned_rules) {
      rule_to_abandon.convertVeriablesToMetas();

      for (auto& pair : all_rules) {
         for (auto& rules : pair.second) {
            for (auto& rule : rules.second) {
               // We want to abandon independently of stage, therefore, we need to calculate match in this way,
               // rather than simply checking rule.serialize() == rule_to_abandon.serialize().
               // TODO: Actually we could do serialize(true, false) now.
               bool is_match{ rule.getTo()
                  && rule.getFrom()->serialize() == rule_to_abandon.getFrom()->serialize()
                  && rule.getTo()->serialize() == rule_to_abandon.getTo()->serialize()
                  && rule.getGlobalCondition() == rule_to_abandon.getGlobalCondition() };

               const auto rule_conditions{ rule.getConditions() };
               const auto rule_to_abandon_conditions{ rule_to_abandon.getConditions() };

               if (is_match && rule_conditions.size() == rule_to_abandon_conditions.size()) {
                  for (int i = 0; i < rule_conditions.size() && is_match; i++) {
                     is_match = is_match
                        && rule_conditions[i].second == rule_to_abandon_conditions[i].second
                        && rule_conditions[i].first->serialize() == rule_to_abandon_conditions[i].first->serialize();
                  }
               }
               else {
                  is_match = false;
               }

               if (is_match) {
                  parser->addNote("Abandoning rule '" + rule.serialize() + "'.");
                  abandonend_rules_found.insert(rule.serialize(true, false));
                  rule.setRuleAbandoned();
                  abandoned_counter++;
               }
            }
         }
      }
   }

   for (const auto& r : abandoned_rules) {
      if (!abandonend_rules_found.count(r.serialize(true, false))) {
         parser->addWarning("Rule '" + r.serialize(true, false) + "', which was marked to be abandoned, was not found in the rules list.");
      }
   }

   std::string s{};

   s += createHeadPartOfSimplificationCode(mc_mode, code_generation_mode, abandoned_rules, additional_rules, all_rules, parser);
   s += createInnerPartOfSimplificationCode(code_generation_mode, all_rules, parser, inner_part_before, inner_part_fast, inner_part_very_fast, inner_part_after, mc_mode);
   s += addTopLevelSimplificationFunction(code_generation_mode, inner_part_before, inner_part_fast, inner_part_after, false);
   s += addTopLevelSimplificationFunction(code_generation_mode, inner_part_before, inner_part_very_fast, inner_part_after, true);
   s += createFootPartOfSimplificationCode(code_generation_mode, mc_mode);

   parser->addNote("Code generation succeeded.");
   return s;
}

void CodeGenerator::deleteAndWriteSimplificationRulesToFile(const CodeGenerationMode mode, const std::string& path, const std::shared_ptr<FormulaParser> parser_raw, const bool mc_mode)
{
   std::string content = createCodeForAllSimplificationRules(mode, parser_raw, mc_mode);
   std::ofstream out(path);
   out << content;
   out.close();
}
