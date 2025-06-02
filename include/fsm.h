//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

//#define FSM_DEBUG // (Un-)comment to add/remove debug output. Or do "ADD_DEFINITIONS(-DFSM_DEBUG)" in CMAKE.

#pragma once
#include "fsm_resolver_factory.h"
#include "term.h"
#include "parsable.h"
#include "fsm_transition.h"
#include "fsm_resolver.h"
#include "fsm_resolver_default.h"
#include "parser.h"
#include "math_struct.h"
#include "enum_wrapper.h"
#include "callback.h"
#include "operator_structure.h"
#include "term_logic_or.h"
#include "term_logic_and.h"
#include "term_logic_eq.h"
#include "static_helper.h"
#include "geometry/images.h"
#include "term_var.h"
#include "meta_rule.h"
#include "cpp_parsing/cpp_type_struct.h"
#include "model_checking/type_abstraction_layer.h"
#include "model_checking/simplification.h"
#include "simplification/simplification.h"
#include "simplification/code_block.h"

#include <ctime>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include <functional>
#include <sstream>
#include <fstream>
#include <limits>
#include <cassert>
#include <deque>
#include <cctype>
#include <random>
#include <chrono>


namespace vfm {
namespace fsm {

enum class SteppingOrderEnum {
   transition_then_execute_callbacks, // Executes callback (cb) states in same step as corresponding non-cb state.
   execute_callbacks_then_transition, // DEFAULT. Executes cb states in same step as corresponding non-cb state.
   transition_only,                   // Executes cb states in same step as corresponding non-cb state.
   execute_callbacks_only,
   transition_then_execute_callbacks_and_stop_at_cb_states, // Stops at each state including cb states.
   execute_callbacks_then_transition_and_stop_at_cb_states, // Stops at each state including cb states.
   transition_only_and_stop_at_cb_states                    // Stops at each state including cb states.
};

enum class NonDeterminismHandling {
    ignore,                           // DEFAULT.
    note_error_on_non_determinism,
    note_error_on_deadlock,
    note_error_on_non_determinism_or_deadlock
};

constexpr bool USE_ASMJIT_FOR_MUTATION = true;

constexpr float MAX_FLOAT = 200, MIN_FLOAT = -200, STEP_FLOAT = 5;

constexpr int INVALID_STATE_NUM = -1;
const std::string PREP_ADDITIONAL_STATE_VARIABLE_NAME = "#STATE_VAR_NAME";
const std::string INVALID_STATE_NAME = "Invalid";
constexpr int INITIAL_STATE_NUM = 1; /// Has to be smallest valid state number.
const std::string CURRENT_STATE_VAR_NAME = ".CurrentState";
const std::string STEP_COUNT_VAR_NAME = ".StepCount";
const std::string CALLBACK_COUNTER_PREFIX = "cb_";
const std::string FIGURE_OUT_RANDOM_VALUE_FROM_RANGE_LATER = "$insert_random_value_from_range$";
const std::string NAME_OF_BASE_ENUM_FOR_STATES = "StateBaseEnum";

class NoCallbacks {};
template<class F> class FSM;
using FSMs = FSM<NoCallbacks>;

typedef int                                                   CBStateID;
typedef std::pair<CBStateID, std::shared_ptr<Term>>           CBIncomingState;
typedef CBStateID                                             CBOutgoingState;
typedef std::vector<CBIncomingState>                          CBIncomingStatesList;
typedef std::vector<CBOutgoingState>                          CBOutgoingStatesList;
typedef std::pair<CBIncomingStatesList, CBOutgoingStatesList> CBInterface;

using PainterVariableDescription = std::pair<std::string, std::pair<int, float>>;
using PainterButtonDescription = std::pair<std::pair<std::string, std::string>, std::pair<std::vector<int>, std::pair<Color, Color>>>;

using PainterFunction = std::function<void(
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
   int& i,
   const std::vector<Image>& ascii_table)>;

enum class GraphvizOutputSelector {
   graph_only,
   data_with_functions,
   data_only,
   functions_only,
   all
};

/// \brief An implementation of a deterministic finite-state machine.
/// The machine creates output when passing a state ("Moore style").
/// Transitions are expressed using arithmetic/logic formulae, providing
/// a MARB/MAPT-compatible architecture.
///
/// Possible future extensions: Mealy-style output; non-determinism.
template<class F>
class FSM : public vfm::Parsable {
public:
   static FSM randomFSM(
      const int numStates,
      const int prob,
      const int cond_depth,
      const int cb_depth,
      const bool simplifyConditions = true,
      const std::set<std::string> var_names = { "x", "y", "z" },
      const long seed = 31415926);

   static bool testFanningOut(
      const int seed = 0,
      const int num_of_states = 5,
      const int num_of_vd_states = 5,
      const int cond_depth = 3,
      const int cb_depth = 3,
      const int simulate_for_steps = 100,
      const bool output_ok_too = false,
      const bool create_pdfs_of_each_step = false);

   FSM(const std::shared_ptr<FSMResolver> resolver = nullptr,
      const NonDeterminismHandling non_determinism_option = NonDeterminismHandling::ignore, // No effect in jit_full mode.
      const std::shared_ptr<DataPack> data = nullptr,
      const std::shared_ptr<FormulaParser> parser = nullptr,
      const FSMJitLevel jit_level = FSMJitLevel::no_jit,
      const std::shared_ptr<mc::TypeAbstractionLayer> type_abstraction_layer = nullptr);

   /// \brief Traverses the FSM to next state according to transitions.
   void step(const SteppingOrderEnum order = SteppingOrderEnum::execute_callbacks_then_transition);

   /// \brief Writes a file containing the visualization of the current FSM in dot format into 
   /// the current working directory.
   /// If \p format is set to "pdf", "png", "jpg" etc. an additional output in the respective
   /// format is produced.
   int createGraficOfCurrentGraph(const std::string& base_filename, const bool mark_current_state = true, const std::string& format = "", const bool mc_mode = false, const GraphvizOutputSelector which = GraphvizOutputSelector::all) const;

   inline void createStateVisualizationOfCurrentState(
      const std::string& filename,
      std::vector<PainterVariableDescription>& variables_to_be_painted,
      std::vector<PainterButtonDescription>& button_descriptions,
      const std::string& background_image_path,
      bool& first,
      Image& background_image,
      Image& sw_chunk,
      int& sw_chunk_x,
      int& sw_chunk_y,
      int& i,
      const OutputType& format = OutputType::png) const
   {
      painter_(
         *data_,
         *painter_image_,
         variables_to_be_painted,
         button_descriptions,
         background_image_path,
         first,
         background_image,
         sw_chunk,
         sw_chunk_x,
         sw_chunk_y,
         i,
         Image::retrieveAsciiTable());

      painter_image_->store(filename, format);
   }

   std::string getDOTCodeOfCurrentGraph(
      const bool mark_current_state = true,
      const bool mc_mode = false,
      const bool point_mark_dead_ends = false,
      const GraphvizOutputSelector what = GraphvizOutputSelector::all) const;

   std::string getXWizardAnimationOfSeveralSteps(const int steps, const SteppingOrderEnum ordering = SteppingOrderEnum::execute_callbacks_then_transition) const;

   std::shared_ptr<Term> generateClosedForm();

   void addInternalActionToState(const int state_id, const std::shared_ptr<Term> formula);
   void addInternalActionToState(const std::string& state_name, const std::shared_ptr<Term> formula);
   void addInternalActionToState(const std::string& state_name, const std::string& formula);
   std::vector<TermPtr> getInternalActionsOfState(const int state_num) const;
   std::vector<TermPtr> getInternalActionsOfState(const std::string& state) const;

   int addAction(
      F* _pt2Object,
      void(F::* _fpt)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName),
      const std::string& action_name = "",
      const int specifyID = -1);

   void associateStateIdToActionId(const int state_id, const int action_id);
   void associateStateNameToActionId(const std::string& state_name, const int action_id);
   void associateStateIdToExistingAction(const int state_id, const std::string& action_name = "");
   void associateStateNameToExistingAction(const std::string& state_name, const std::string& action_name);
   void setActionName(const int action_id, const std::string& action_name);
   void removeActionsFromState(const int state_id);

   void setAdditionalStateVarName(const std::string& name);

   /// Returns the newly associated or already available action id.
   int associateStateIdToAction(
      const int state_id,
      F* _pt2Object,
      void(F::* _fpt)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName),
      const std::string& action_name = "");

   /// Returns the newly associated or already available action id.
   int associateStateToAction(
      const std::string& state_name,
      F* _pt2Object,
      void(F::* _fpt)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName),
      const std::string& action_name = "");

   std::shared_ptr<DataPack> getData() const;
   std::shared_ptr<FormulaParser> getParser() const;

   /// Define a new arithmetic operator (always in prefix, i.e., a function) which
   /// can be used in transitions. E.g.:
   /// defineNewOperator("pi", "3.14")                                     ==> yields pi() = 3.14
   /// defineNewOperator("inc", "p_(0) + 1")                               ==> yields inc(x) = x+1
   /// defineNewOperator("square", "p_(0) * p_0")                          ==> yields square(x) = x^2
   /// defineNewOperator("ellipse", "pi() * 4/3 * p_(0) * p_(1) * p_(2)")  ==> yields ellipse(...), the volume of an 3d ellipsoid.
   virtual void defineNewMathOperator(const std::string& op_name, const std::string& formula);
   virtual void defineNewMathOperator(const std::string& op_name, const std::shared_ptr<Term>& formula);

   void removeTransition(const std::shared_ptr<fsm::FSMTransition> trans);
   void removeState(const int id);

   void applyToAllCallbacksAndConditionsAndMCItems(
      const std::function<void(std::shared_ptr<MathStruct>)>& f,
      const bool include_callbacks = true,
      const bool include_transitions = true,
      const bool include_functions = true);

   void applyConstnessToAllVariablesInCallbacksAndConditionsAndMCItems();

   void addTransition(
      std::shared_ptr<FSMTransition> trans,
      const bool create_even_unnecessary_implicit_transitions = true);

   void addCompletingTransition(
      const int source_state,
      const int destination_state,
      const bool create_even_unnecessary_implicit_transitions = false);

   void addCompletingTransition(
      const std::string& source_state,
      const std::string& destination_state,
      const bool create_even_unnecessary_implicit_transitions = false);

   bool parseProgram(const std::string& fsm_commands) override;

   template <class T>
   void addEnumType(EnumWrapper<T>& enumType)
   {
      data_->addEnumMapping(enumType.getEnumName(), enumType.getNameMapping());
   }

   void addEnumType(const std::string& enum_name, const std::map<int, std::string>& mapping);

   void associateVarToEnum(const std::string& var_name, const std::string& enum_name)
   {
      data_->associateVarToEnum(var_name, enum_name);
   }

   std::string getSingleValAsEnumName(const std::string& var_name) const
   {
      return data_->getSingleValAsEnumName(var_name);
   }

   void replayCounterExampleForOneStep(const MCTrace& ce, int& i, int& loop, const bool always_replay_from_start = false);

   inline int getStateCount() const
   {
      return states_plain_set_.size();
   }

   inline void setTypeAbstractionLayer(const std::shared_ptr<mc::TypeAbstractionLayer> new_layer)
   {
      type_abstraction_layer_ = new_layer;
      addFailableChild(type_abstraction_layer_);
   }

   inline void setStatePainter(const PainterFunction& painter
   )
   {
      painter_ = painter;
   }

   inline void addTransition(
      const int source_state,
      const int destination_state,
      const bool create_even_unnecessary_implicit_transitions = true)
   {
      addTransition(source_state, destination_state, _true(), create_even_unnecessary_implicit_transitions);
   }

   inline void addTransition(
      const int source_state,
      const int destination_state,
      const std::shared_ptr<Term> condition,
      const bool create_even_unnecessary_implicit_transitions = true)
   {
      auto trans(std::make_shared<FSMTransition>(source_state, destination_state, condition, data_));
      addTransition(trans, create_even_unnecessary_implicit_transitions);
   }

   inline void addCompletingTransitionToSelf(
      const int source_and_destination_state,
      const bool create_even_unnecessary_implicit_transitions = false)
   {
      addCompletingTransition(source_and_destination_state, source_and_destination_state, create_even_unnecessary_implicit_transitions);
   }

   inline void addCompletingTransitionToSelfFromStateName(
      const std::string& source_and_destination_state,
      const bool create_even_unnecessary_implicit_transitions = false)
   {
      int state_id = getInternalNumFromStateName(source_and_destination_state);
      addCompletingTransition(state_id, state_id, create_even_unnecessary_implicit_transitions);
   }

   inline void addCompletingTransitionToSelf(
      const std::string& source_and_destination_state,
      const bool create_even_unnecessary_implicit_transitions = false)
   {
      addCompletingTransition(
         state_name_to_state_id_[source_and_destination_state],
         state_name_to_state_id_[source_and_destination_state],
         create_even_unnecessary_implicit_transitions);
   }

   inline void completeAllStatesToSelf(const bool create_even_unnecessary_implicit_transitions = false)
   {
      for (const auto state_id : states_plain_set_) {
         addCompletingTransitionToSelf(state_id, create_even_unnecessary_implicit_transitions);
      }
   }

   inline int smallestFreeActionID() const
   {
      int i;
      for (i = 1; action_id_to_action_.count(i); ++i) {}
      return i;
   }

   inline int smallestFreeStateID() const
   {
      int i;
      for (i = INITIAL_STATE_NUM; states_plain_set_.count(i); ++i) {}
      return i;
   }

   inline void addTransition(
      const std::string& source_state,
      const std::string& destination_state,
      const std::shared_ptr<Term> condition,
      const bool create_even_unnecessary_implicit_transitions = true)
   {
      int first_id_new = 0;

      if (!state_name_to_state_id_.count(source_state)) {
         setStateName(smallestFreeStateID(), source_state);
         first_id_new = 1;
      }

      if (!state_name_to_state_id_.count(destination_state)) {
         setStateName(smallestFreeStateID() + first_id_new, destination_state);
      }

      addTransition(
         state_name_to_state_id_[source_state],
         state_name_to_state_id_[destination_state],
         condition,
         create_even_unnecessary_implicit_transitions);
   }

   inline void addTransition(
      const std::string& source_state,
      const std::string& destination_state,
      const std::string& condition,
      const bool create_even_unnecessary_implicit_transitions = true)
   {
      addTransition(
         source_state,
         destination_state,
         MathStruct::parseMathStruct(condition, parser_)->toTermIfApplicable(),
         create_even_unnecessary_implicit_transitions);
   }

   inline std::string getCurrentStateVarName() const {
      return CURRENT_STATE_VAR_NAME;
   }

   inline std::string getStepCountVarName() const {
      return STEP_COUNT_VAR_NAME;
   }

   inline int getCurrentState() const
   {
      return data_->getValFromArray(getCurrentStateVarName(), registration_number_);
   }

   inline void setVariable(const std::string& var_name, const float value, const bool make_const)
   {
      data_->addOrSetSingleVal(var_name, value, false, make_const);

      if (data_->isConst(var_name)) { // Update all instances in all formulas. TODO: Makes sense??
         applyToAllCallbacksAndConditionsAndMCItems([&var_name, &value](const MathStructPtr m) {
            auto m_var = m->toVariableIfApplicable();

            if (m_var) {
               if (m_var->getVariableName() == var_name) {
                  m_var->setConstVariable(value);
               }
            }
            }, true);
      }
   }

   static inline std::string stateNameGV(const int state_num) {
      return "State" + std::to_string(state_num - INVALID_STATE_NUM);
   }

   static inline std::vector<std::shared_ptr<fsm::FSMTransition>> extractOnlyNonFalseConditions(const std::vector<std::shared_ptr<fsm::FSMTransition>>& original)
   {
      std::vector<std::shared_ptr<fsm::FSMTransition>> result;

      for (const auto& trans : original) {
         if (!trans->condition_->isAlwaysFalse()) result.push_back(trans);
      }

      return result;
   }

   void encapsulateIfConditions(TermPtr& formula, const std::string& function_name_base = "c");

#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
   /// \brief Declares the given variable (if not existing) and connects it to the
   /// bool variable at the given address. Setting the original variable from
   /// outside will be reflected by the data_set's associated variable, and setting
   /// the data_set's variable will be reflected by the outside variable.
   /// NOTE: Be careful not to let *external_variable go out of scope 
   /// (which would result in undefined behavior)!
   void initAndAssociateVariableToExternalBool(const std::string& variable_name, bool* external_variable);

   /// \brief Declares the given variable (if not existing) and connects it to the
   /// char variable at the given address. Setting the original variable from
   /// outside will be reflected by the data_set's associated variable, and setting
   /// the data_set's variable will be reflected by the outside variable.
   /// NOTE: Be careful not to let *external_variable go out of scope 
   /// (which would result in undefined behavior)!
   void initAndAssociateVariableToExternalChar(const std::string& variable_name, char* external_variable);

   /// \brief Declares the given variable (if not existing) and connects it to the
   /// float variable at the given address. Setting the original variable from
   /// outside will be reflected by the data_set's associated variable, and setting
   /// the data_set's variable will be reflected by the outside variable.
   /// NOTE: Be careful not to let *external_variable go out of scope 
   /// (which would result in undefined behavior)!
   void initAndAssociateVariableToExternalFloat(const std::string& variable_name, float* external_variable);
#endif

   /// Declares the given variable and sets it to 0.
   inline void initVariable(const std::string& variable_name)
   {
      setVariable(variable_name, 0);
   }

   /// Declares the given variables and sets them to 0.
   inline void initVariables(const std::vector<std::string>& variables)
   {
      for (const auto& s : variables) initVariable(s);
   }

   inline void setStateName(const int state_id, const std::string state_name_raw)
   {
      std::string state_name = state_name_raw;

      if (!StaticHelper::isValidVfmVarName(state_name)) {
         state_name = "State_" + std::to_string(state_id) /*StaticHelper::safeString(state_name)*/;

         addWarning("State '"
            + std::to_string(state_id)
            + "' cannot be named '"
            + state_name_raw
            + "' because this would violate the allowed naming pattern for vfm variables "
            + "[" + VAR_AND_FUNCTION_NAME_REGEX_STRING_BASE + "]"
            + ". "
            + "I will use a name based on the state number instead: '" + state_name + "'."
            //+ "I will use the(recoverable) safe string '" + state_name + "' instead."
         );
      }

      if (!states_plain_set_.count(state_id)) {
         states_plain_set_.insert(state_id);
         outgoing_transitions_->insert({ state_id, {} });
         incoming_transitions_->insert({ state_id, {} });
      }

      state_id_to_state_name_[state_id] = state_name;
      state_name_to_state_id_[state_name] = state_id;
      steps_at_last_visit_.insert({ state_id, std::numeric_limits<int>::min() });

      addStateVariable(state_id);
   }

   inline void setDefaultStateNameIfUnspecified(const int state_id)
   {
      if (state_id_to_state_name_.find(state_id) != state_id_to_state_name_.end()) return;
      std::string state_name = std::to_string(state_id);
      setStateName(state_id, state_name);
   }

   inline void setInitialStateName(const std::string& name)
   {
      if (state_id_to_action_ids_[INITIAL_STATE_NUM].empty()) { // Adopt acions only if initial state has none so far.
         for (const auto& action : state_id_to_action_ids_[state_name_to_state_id_[name]]) {
            associateStateIdToActionId(INITIAL_STATE_NUM, action);
         }
      }

      setStateName(INITIAL_STATE_NUM, name);
   }

   inline int getInitialStateNum() const
   {
      return INITIAL_STATE_NUM;
   }

   inline FSMJitLevel getJitLevel()
   {
      return jit_level_;
   }

   inline void addImageToState(const int state, const std::shared_ptr<Image> img)
   {
      state_images_[state] = img;
   }

   inline bool hasStateImage(const int state)
   {
      return state_images_.find(state) != state_images_.end();
   }

   inline void setProhibitDoubleEdges(const bool val)
   {
      prohibit_double_edges_ = val;
   }

   inline int getNumStates() const
   {
      return states_plain_set_.size();
   }

   inline bool isReachable(const int from, const int to) const
   {
      auto vec = reachableStatesFrom(from);
      return vec.count(to);
   }

   inline bool isReachable(const int to) const
   {
      return isReachable(INVALID_STATE_NUM, to);
   }

   std::shared_ptr<FSMResolver> getResolver() const;

   float evaluateFormula(const std::string& formula) const;
   std::string getNameFromInternalStateNum(const int num);
   int getInternalNumFromStateName(const std::string& name);
   std::vector<int> getActionIDsOfStateID(const int state_num);
   int getNumOfActions() const;
   std::shared_ptr<std::vector<std::shared_ptr<FSMTransition>>> getTransitions();

   void setGraphvizStateFadeoutValue(const int value);

   void loadFromFile(const std::string& in_path, const bool reset_before_loading = true, const bool find_set_and_external_variables = false);
   void storeToFile(const std::string& out_path);
   void resetFSMTopology(const bool reset_state_images_too = false);
   void resetDataAndFSMState();
   void resetFSMState();
   void resetMathOperators();
   void resetPrivateStuff();
   void reachableStatesFrom(std::set<int>& states) const;
   std::set<int> reachableStates() const;
   std::set<int> reachableStatesFrom(const int state) const;

   void checkForSimplifiableCode(const TraverseCompoundsType include_compounds = TraverseCompoundsType::avoid_compound_structures);
   void checkForDoubleCode(const TraverseCompoundsType include_compounds = TraverseCompoundsType::avoid_compound_structures);

   int addUnconnectedStateIfNotExisting(const std::string& name);
   void addUnconnectedStateIfNotExisting(const int state_id);

   void createComplementOfTransitions(const bool ignore_invalid_state = true); /// Removes all transitions and inserts true transitions wherever there was no transition before.

   std::string toCPP(const std::string& state_var_name) const;

   std::shared_ptr<Term> createFormulaFromFSM(const std::string& state_var_name) const;

   /// \brief Exchanges each state with n>1 callbacks with n individual
   /// states with one callback each. Embeds the new structure into the FSM
   /// such that the overall behavior doesn't change other than possibly stretching
   /// out parts of the execution from one to several states.
   void fanOutMultipleCallbacksToIndividualStates();

   /// \brief Creates a sub-FSM (with no connection to the already existing states) which
   /// implements the execution of the given callback.
   ///
   /// Note: To be fanned out, a callback needs to obey the same constraints as described in the
   /// comment of the serializeCallbacksNuSMV method.
   CBInterface fanOutSingleCallbackToStates(const std::shared_ptr<Term> callback, const std::string optional_cb_name = "", const bool use_existing_callback_if_available = true);

   /// \brief The method fanOutSingleCallbackToStates creates dummy "bottleneck" states with no
   /// callback which can, in turn, be replaced by direct transitions from the incoming
   /// to the outgoing states. This is done by this method.
   ///
   /// Note: When a bottleneck state has more than two incoming transitions m > 2 and more than two outgoing
   /// transitions n > 2, it is not removed. It seems more efficient to keep it, since removing it would 
   /// create m * n > m + n new transitions. TODO: This procedure is up for debate, for example it might be
   /// better to keep the state if there are at least 2 incoming OR at least two outgoing transitions.
   void cleanupCBBottleneckStates();

   /// \brief Convenience method, expanding all non-atomic callbacks in all states to sub-graphs
   /// with zero or one atomic set function(s) per state. The method does the following:
   /// - call fanOutMultipleCallbacksToIndividualStates,
   /// - for all states s with a callback cb:
   ///   * call fanOutSingleCallbackToStates(cb, .),
   ///   * connect the returned CB interface with s and its successor states,
   /// - call cleanupCBBottleneckStates().
   ///
   /// Note: All callbacks need to follow the rules as given in the comment of
   /// the serializeCallbacksNuSMV method for the FSM to be serializable to nuSMV.
   void fanOutAllNonAtomicCallbacksToStates();

   inline void simplifyFunctions(const bool mc_mode)
   {
      parser_->simplifyFunctions(mc_mode, false);
   }

   inline void simplifyCallbacks(const bool mc_mode)
   {
      for (const auto& state_id : states_plain_set_) {
         for (const auto& cb_id : state_id_to_action_ids_[state_id]) {
            auto cb = action_id_to_action_[cb_id];
            TermPtr formula = cb->getFormula();

            if (formula) {
               formula->setChildrensFathers(true, false); // TODO: Ideally, this shoudln't be necassary. Children should be consistent, and everything valid.
               addDebug("Simplifying callback " + std::to_string(cb_id) + " for state '" + state_id_to_state_name_.at(state_id) + "'.");

               auto before = std::to_string(formula->getNodeCount(TraverseCompoundsType::go_into_compound_structures));
               formula = (mc_mode ? mc::simplification::simplifyFast(formula, parser_) : simplification::simplifyFast(formula, parser_));
               auto after = std::to_string(formula->getNodeCount(TraverseCompoundsType::go_into_compound_structures));
               cb->setFormula(formula);
               addDebug("Simplified callback. Compression " + before + " ==> " + after + " nodes.");

               // Not necessary anymore since compound structure is now automatically simplified with the formula.
               // TODO: Delete this obsoltete code...
               //std::shared_ptr<TermCompound> compound{ formula->toTermCompoundIfApplicable() };
               //if (compound && !compound->getCompoundStructure()->containsMetaCompound()) {
               //   addDebug("Simplifying compound structure of callback " + std::to_string(cb_id) + " for state '" + state_id_to_state_name_.at(state_id) + "'.");
               //   TermPtr compound_fmla{ compound->getCompoundStructure() };

               //   auto before = std::to_string(compound_fmla->getNodeCount());
               //   compound_fmla->setChildrensFathers(true, false);
               //   compound_fmla = mc_mode ? mc::simplification::simplifyFast(compound_fmla, parser_) : vfm::simplification::simplifyFast(compound_fmla, parser_);
               //   auto after = std::to_string(compound_fmla->getNodeCount());
               //   addDebug("Simplified formula. Compression " + before + " ==> " + after + " nodes.");
               //}
            }
         }
      }
   }

   inline void simplifyConditions(const bool mc_mode)
   {
      encapsulateLongConditionsIntoFunctions(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), true, mc_mode);
   }

   inline void encapsulateLongConditionsIntoFunctions(const int from_serialized_length = 100, const int from_node_count = 100, const bool simplify_first = false, const bool mc_mode = false)
   {
      int i = 1;
      int max = transitions_plain_set_->size();

      for (auto& trans : *transitions_plain_set_) {
         if (simplify_first) {
            addDebug("Simplifying condition " + std::to_string(i++) + "/" + std::to_string(max) + ".");
            trans->condition_ = mc_mode ? mc::simplification::simplifyFast(trans->condition_, parser_) : simplification::simplifyFast(trans->condition_, parser_);
         }

         if (trans->condition_->getNodeCount() >= from_node_count || trans->condition_->serialize().size() >= from_serialized_length) {
            std::string name = "_" + StaticHelper::shortenToMaxSize(state_id_to_state_name_[trans->state_source_], 10, false)
               + "_"
               + StaticHelper::shortenToMaxSize(state_id_to_state_name_[trans->state_destination_], 10, false)
               + "_"
               + encapsulation_counter_;

            encapsulation_counter_ = StaticHelper::incrementAlphabeticNum(encapsulation_counter_);

            defineNewMathOperator(
               name,
               trans->condition_->toTermIfApplicable());

            trans->condition_ = parser_->termFactory(name, std::vector<std::shared_ptr<Term>>{});
         }
      }
   }

   inline void encapsulateLongCallbacksIntoFunctions(const int from_serialized_length = 100, const int from_node_count = 100, const bool simplify_first = false)
   {
      for (const auto& state_id : states_plain_set_) {
         int counter = 0;

         for (const auto& cb_id : state_id_to_action_ids_[state_id]) {
            auto cb = action_id_to_action_[cb_id];
            auto formula = cb->getFormula();

            if (simplify_first && formula) {
               formula = MathStruct::simplify(formula);
               cb->setFormula(formula);
            }

            if (formula->getNodeCount() >= from_node_count || formula->serialize().size() >= from_serialized_length) {
               std::string name = state_id_to_state_name_[state_id] + "_" + (counter ? std::to_string(counter) : "");

               defineNewMathOperator(
                  name,
                  formula->toTermIfApplicable());

               cb->setFormula(parser_->termFactory(name, std::vector<std::shared_ptr<Term>>{}));
            }

            counter++;
         }
      }
   }

   /// Simplifies conditions and callbacks. 
   /// Then removes all transitions and states that are unreachable.
   inline void simplifyFSM(const bool include_functions, const bool mc_mode)
   {
      if (include_functions) {
         simplifyFunctions(mc_mode);
      }

      simplifyConditions(mc_mode);
      simplifyCallbacks(mc_mode);
      removeUnsatisfiableTransitionsAndUnreachableStates();

      if (mc_mode) {
         addWarning("'checkForOutdatedSimplification' has been deactivated.");
         //checkForOutdatedSimplification();
      }
   }

   static std::shared_ptr<MathStruct> collectIfConditions(const std::shared_ptr<MathStruct> m);

   /// Assumes that "0" conditions cannot be taken and removes all transitions 
   /// and states that are unreachable under that assumtion.
   /// TODO: make this more general by considering FSMResolver.
   void removeUnsatisfiableTransitionsAndUnreachableStates();

   bool testEquivalencyWith(FSM<F>& other, const int steps = 100, const bool create_pdfs = false);

   bool testEquivalencyWith(
      FSM<F>& other,
      const int steps,
      const bool store_pdfs,
      FSM<F>& var_disturber);

   bool testEquivalencyWith(
      const TermPtr formula,
      const mc::TypeAbstractionLayer& tal,
      const int steps = 100,
      const bool output_ok_too = true,
      const bool create_pdfs = false);

   /// Registers a new external data pack with this FSM or creates
   /// a new one. Note that the old one is abandoned, if existing.
   void registerDataPack(const std::shared_ptr<DataPack> data);

   void registerResolver(const std::shared_ptr<FSMResolver> resolver);

   /// Creates a deep copy of this FSM object (with minor exceptions, cf. in-code comments).
   /// Particularly, the use_same_... flags can be used to enforce pointing to the 
   /// same data/parser rather than creating a copy.
   FSM<F> copy(const bool use_same_data = false, const bool use_same_parser = true) const;
   std::shared_ptr<FSM<F>> copyToPtr(const bool use_same_data = false, const bool use_same_parser = true) const;

   bool mutate();

   bool mutateM1InsertState();
   bool mutateM2RemoveStateIfUnreachable();
   bool mutateM3RemoveIdleStateIfDeadEnd();

   bool mutateM4InsertFalseTransition();
   bool mutateM5RemoveFalseTransition();

   bool mutateM6ChangeStateCommand();

   bool mutateM7SimplifyCondition();
   bool mutateM8ComplexifyCondition();
   bool mutateM9ChangeOperatorInCondition();
   bool mutateM10ChangeConstantInCondition();
   bool mutateM11ChangeSensorInCondition();

   bool helperForRandomConditionChanges(const std::function<void(const std::shared_ptr<MathStruct>)>& func, const std::shared_ptr<bool> trigger_break) const;
   static std::vector<std::shared_ptr<Term>> getEvolutionSensors();
   static bool isEvolutionSensor(const std::shared_ptr<Term> m);

   void takeOverTopologyFrom(const std::shared_ptr<FSM<F>> other);

   int getStepCount() const;
   std::string serializeToProgram() const;

   MCTrace createTraceFromSimulation(const int steps = 10, const TermPtr variable_controller = _val(0));

   static std::deque<std::shared_ptr<MathStruct>> extractTopLevelCommandsRecursively(const std::shared_ptr<MathStruct> f);

   /// E.g., "x; 0" is in general not equal to "x", but using this helper method we can simplify it anyway.
   /// More precisely, arbitrary rules are allowed, the result does not have to be equivalent (or even "similar") to the original formula.
   /// Note that MetaRule("0; p_(0)", "p_(0)") is covered in the general simplifications, but since we have a very specific situation here,
   /// the FSM extraction ("3rd translation") is substantially faster when including this rule here, anyway.
   static void applyNonGeneralSimplifications(
      TermPtr& formula,
      const std::vector<MetaRule>& rules = { MetaRule("p_(0); 0", "p_(0)"), MetaRule("0; p_(0)", "p_(0)") },
      const bool simplify_in_between = false);

   inline void addConstantToModelChecker(const std::string& var_name)
   {
      all_constants_to_model_checker_.insert(var_name);
   }

   inline std::string getStateVarName(const int state_id, const bool include_enum_type) const
   {
      return (include_enum_type ? NAME_OF_BASE_ENUM_FOR_STATES + "::" : "") + std::string("state_") + state_id_to_state_name_.at(state_id);
   }

   inline std::set<int> getStatesPlainSet() const
   {
      return states_plain_set_;
   }

private:
   PainterFunction painter_{};
   const std::shared_ptr<Image> painter_image_{};

   int registration_number_ = 0;
   std::string encapsulation_counter_ = "A";
   FSMResolverFactory resolver_factory_{};
   const std::shared_ptr<FormulaParser> parser_{};
   std::shared_ptr<FSMResolver> resolver_{};
   NonDeterminismHandling non_determinism_option_{};
   mutable std::set<std::string> all_fsm_controlled_variables_{};
   mutable std::set<std::string> all_external_variables_{};
   mutable std::set<std::string> all_constants_to_model_checker_{};

   std::string additional_state_var_name_{};

   FSMJitLevel jit_level_{};

   std::shared_ptr<DataPack> data_{};
   std::shared_ptr<Term> closed_form_ = nullptr;
   const std::shared_ptr<std::vector<std::shared_ptr<FSMTransition>>> transitions_plain_set_{};
   std::set<int> states_plain_set_{};
   const std::shared_ptr<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>> outgoing_transitions_{};
   const std::shared_ptr<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>> incoming_transitions_{};

   mutable std::map<std::string, VarClassification> archived_sr_classifications_from_last_time_{};

   /// Returns INVALID_STATE_NUM only if there is no other state or no state at all.
   /// Both is usually not the case since state -1 and 1 are inserted automatically
   /// during construction.
   int getRandomStateIDExceptInvalidState() const;

   std::shared_ptr<FSMTransition> getRandomTransitionExceptInvalidState() const;

   std::map<int, std::vector<int>> state_id_to_action_ids_{};
   std::map<int, std::shared_ptr<TFunctor<F>>> action_id_to_action_{};
   std::map<int, std::string> state_id_to_state_name_{};
   std::map<std::string, int> state_name_to_state_id_{};
   std::map<std::string, int> action_name_to_action_id_{};

   std::map<std::string, VarClassification> var_classifications_for_sr_{};

   std::map<int, float> steps_at_last_visit_{}; // Use float to simplify formula.
   float graphviz_state_fadeout_value_ = 10;
   float step_counter_ = 0;

   std::map<std::string, std::shared_ptr<MathStruct>> unfolded_callback_interfaces_formulae_{};
   std::map<std::string, CBInterface> unfolded_callback_interfaces_{};
   std::map<std::string, int> unfolded_callback_counters_{};
   std::map<std::string, std::set<int>> cb_clusters_{};
   int fan_out_cb_counter_ = 0;
   int fan_out_cb_dummy_state_counter_{};

   std::shared_ptr<mc::TypeAbstractionLayer> type_abstraction_layer_{};

   void manageStepCounter(const int currentState);
   void transitionToNextState();
   void executeCallbacksOfCurrentState();

   void findAllSetVariables() const;

   void insertUnfoldedCallbackInterface(
      const std::string& cb_name,
      const CBInterface& cb_interface,
      const std::shared_ptr<MathStruct> formula);

   bool isImplicitCBState(const int state_id) const;

   /// To be serializable to nuSMV, all callbacks "cb" must...
   /// - be internal (external callbacks are ignored),
   /// - not use whilelim (except as part of the if/ifelse compound structures),
   /// - comply to the following form:
   ///   * cb = c1 >> c2 >> ... >> cn (all ">>" left-associative, i.e. simply don't use brackets),
   ///   * with each ci being an arbitrarily nested if statement (including "zero-level", i.e. just a set_ function),
   ///   * with no if, ifelse, >>, set_ within a set_ function or the condition part of an if/ifelse. (Technically, only set_ is prohibited since the other operators could be emulated without set_, but the implementation currently depends on all four parameters not occurring at these positions; this could fairly easily be avoided if necessary.)
   ///
   /// Example of a valid callback:
   ///    set_x(A) >>
   ///    ifelse(b,
   ///           set_x(B) >> set_y(C),
   ///           set_x(D) >> set_x(E) >> set_y(F) >> if(c, set_z(G))) >>
   ///    if(d, set_z(F))
   ///
   /// These constraints are fairly weak, only assuring that a callback is an FSM itself.
   /// Most typical use cases can be implemented with these constraints, including our "G1-demo" FCT
   /// which naturally did not use any other structures than the above. If extensions should be needed
   /// after all, please contact Lukas Koenig.
   std::string serializeCallbacksNuSMV(const MCTranslationTypeEnum translation_type, int& max_pc) const;

   static void extractTopLevelCommands(const std::shared_ptr<MathStruct> f, std::deque<std::shared_ptr<MathStruct>>& vec);
   void addSetter(
      std::map<std::string, std::vector<std::pair<int, std::pair<int, std::pair<std::shared_ptr<MathStruct>, std::shared_ptr<MathStruct>>>>>>& map,
      std::map<int, int>& pc_max_values,
      const int state,
      const int pc,
      const std::shared_ptr<MathStruct> formula) const;

   std::string last_loaded_program = "";

   std::map<int, std::shared_ptr<Image>> state_images_;

   bool prohibit_double_edges_ = false; /// CAUTION: transition evaluation order may become not what you want for not deterministic choices.

   inline void setCurrentState(const int stateNum)
   {
      data_->addArrayAndOrSetArrayVal(getCurrentStateVarName(), registration_number_, stateNum, ArrayMode::floats);

      if (!additional_state_var_name_.empty()) {
         std::string state_name = state_id_to_state_name_[stateNum];

         if (data_->isEnum(additional_state_var_name_) && StaticHelper::stringContains(state_name, "::")) {
            auto pair = StaticHelper::split(state_name, "::");
            std::string enum_name = pair[0];
            std::string value_name = pair[1];

            data_->setValueAsEnum(additional_state_var_name_, value_name);
         }
         else {
            data_->addOrSetSingleVal(additional_state_var_name_, stateNum);
         }
      }
   }

   inline void resetStepCounter()
   {
      step_counter_ = 0;
      data_->addArrayAndOrSetArrayVal(getStepCountVarName(), registration_number_, step_counter_, ArrayMode::floats);
   }

   inline void incrementStepCounter()
   {
      step_counter_++;
      data_->addArrayAndOrSetArrayVal(getStepCountVarName(), registration_number_, step_counter_, ArrayMode::floats);
   }

   inline void addStateVariable(const int state_id)
   {
      data_->addOrSetSingleVal(getStateVarName(state_id, false), state_id, true);
   }
};

template<class F>
std::shared_ptr<FSMResolver> vfm::fsm::FSM<F>::getResolver() const {
      return resolver_;
}

template<class F>
void vfm::fsm::FSM<F>::loadFromFile(const std::string& in_path, const bool reset_before_loading, const bool find_set_and_external_variables)
{
   resetAllErrors();
   std::ifstream input(in_path);

   if (input.good()) {
      std::stringstream sstr;
      while (input >> sstr.rdbuf());
      std::string program = sstr.str();

      if (last_loaded_program == program) {
         return;
      }

      last_loaded_program = program;

      if (reset_before_loading) {
         this->resetPrivateStuff();
         this->resetMathOperators();
         this->resetFSMTopology();
      }

      parseProgram(program);

      if (find_set_and_external_variables) {
         findAllSetVariables();
      }

#if defined(FSM_DEBUG)
      if (hasErrorOccurred()) {
         std::cerr << "<FSM> Parsing " << FAILED_COLOR << "FAILED" << RESET_COLOR << "." << std::endl << std::endl;
      }
      else {
         std::cout << "<FSM> Program parsed " << OK_COLOR << "SUCCESSFULLY" << RESET_COLOR << "." << std::endl << std::endl;
      }
#endif
   }
   else {
      addError("File '" + in_path + "' does not exist. No FSM loaded.");
   }

   input.close();
}

template<class F>
void vfm::fsm::FSM<F>::storeToFile(const std::string& out_path)
{
   std::string input = serializeToProgram();
   std::ofstream out(out_path);
   out << input;
   out.close();
}

template<class F>
void FSM<F>::resetFSMTopology(const bool reset_state_images_too)
{
   closed_form_ = nullptr;
   transitions_plain_set_->clear();
   states_plain_set_.clear();
   outgoing_transitions_->clear();
   incoming_transitions_->clear();
   state_id_to_action_ids_.clear();
   archived_sr_classifications_from_last_time_.clear();

   if (reset_state_images_too) {
      state_images_.clear();
   }

   // Initial transition.
   std::shared_ptr<FSMTransition> trans = std::make_shared<FSMTransition>(
      INVALID_STATE_NUM, // Invalid state before FSM start.
      INITIAL_STATE_NUM, // Actual initial state after first step.
      _true(),           // Unconditionally enter initial state from invalid state.
      data_);

   setStateName(INVALID_STATE_NUM, INVALID_STATE_NAME);

   addTransition(trans);

   setCurrentState(INVALID_STATE_NUM);
}

template<class F>
void FSM<F>::resetFSMState()
{
   resetStepCounter();
   setCurrentState(INVALID_STATE_NUM);
}

template<class F>
void FSM<F>::resetDataAndFSMState()
{
   data_->reset();
   resetFSMState();
   all_external_variables_.clear();
   all_fsm_controlled_variables_.clear();
}

template<class F>
void FSM<F>::resetPrivateStuff()
{
   data_->resetPrivateStuff();
}

template<class F>
inline void FSM<F>::checkForSimplifiableCode(const TraverseCompoundsType include_compounds)
{
   std::vector<std::shared_ptr<Term>> formulae{};

   for (const auto& t : *transitions_plain_set_) {
      formulae.push_back(t->condition_->toTermIfApplicable());
   }

   if (include_compounds == TraverseCompoundsType::go_into_compound_structures) {
      for (const auto& ts : parser_->getDynamicTermMetas()) {
         for (const auto& t : ts.second) {
            formulae.push_back(t.second);
         }
      }
   }

   for (const auto& t : formulae) {
      const auto c1{ t };
      const auto c2{ MathStruct::simplify(c1->copy()->toTermIfApplicable()) };
      int count1{ c1->getNodeCount() };
      int count2{ c2->getNodeCount() };

      if (count1 > count2) {
         addWarning("The " + std::to_string(count1) + "-node formula \t'" + c1->serialize() + "'\n"
                    + "could be simplified to " + std::to_string(count2) + " nodes: \t'" + c2->serialize() + "'.");
      }
   }
}

template<class F>
inline void FSM<F>::checkForDoubleCode(const TraverseCompoundsType include_compounds)
{
   for (const auto& t1 : *transitions_plain_set_) {
      for (const auto& t2 : *transitions_plain_set_) {
         const auto vec = t1->condition_->findDuplicateSubTerms(t2->condition_, 4, include_compounds);
         for (auto pair : vec) {
            auto m = pair.first;
            auto o = pair.second;

            std::string other_ser = m->isStructurallyEqual(o) ? "" : "'    ||SAME-FOR-SURE||    '" + o->serialize();
            addWarning("Found duplicate term '" + m->serialize() + other_ser + "':\n" + m->serializeWithinSurroundingFormula() + "\n" +  o->serializeWithinSurroundingFormula());
         }
      }
   }
}

template<class F>
int FSM<F>::addUnconnectedStateIfNotExisting(const std::string& name)
{
   if (!state_name_to_state_id_.count(name)) {
      int id = smallestFreeStateID();
      setStateName(id, name);
      state_id_to_action_ids_.insert({id, {}});
      return id;
   } else {
      return state_name_to_state_id_[name];
   }
}

template<class F>
inline void FSM<F>::addUnconnectedStateIfNotExisting(const int state_id)
{
   states_plain_set_.insert(state_id);
   outgoing_transitions_->insert({state_id, {}});
   incoming_transitions_->insert({state_id, {}});
   steps_at_last_visit_.insert({state_id, std::numeric_limits<int>::min()});
   
   if (!state_id_to_state_name_.count(state_id)) {
      setStateName(state_id, std::to_string(state_id));
   }
   addStateVariable(state_id);
   state_id_to_action_ids_.insert({state_id, {}});
}

template<class F>
inline void FSM<F>::createComplementOfTransitions(const bool ignore_invalid_state)
{
   std::set<std::pair<int, int>> complement;

   for (int i : states_plain_set_) {
      for (int j : states_plain_set_) {
         if (!ignore_invalid_state || i != INVALID_STATE_NUM && j != INVALID_STATE_NUM) {
            complement.insert({ i, j });
         }
      }
   }

   for (const auto& trans : *transitions_plain_set_) {
      std::pair<int, int> pair = {trans->state_source_, trans->state_destination_};
      complement.erase(pair);
   }

   transitions_plain_set_->clear();
   outgoing_transitions_->clear();
   incoming_transitions_->clear();

   for (const auto& pair : complement) {
      addTransition(pair.first, pair.second);
   }
}

template<class F>
inline std::shared_ptr<Term> FSM<F>::createFormulaFromFSM(const std::string& state_var_name) const
{
   //std::string cpp_code = toCPP(state_var_name);
   //auto vfm_fmla = MathStruct::parseMathStruct(cpp_code, true, true, getParser())->toTermIfApplicable();
   //return vfm_fmla;

   addFatalError("The function 'createFormulaFromFSM' is currently not implemented.");
   return nullptr;
}

template<class F>
void FSM<F>::reachableStatesFrom(std::set<int>& states) const
{
   std::set<int> new_states;

   for (const auto& state : states) {
      if (outgoing_transitions_->count(state)) {
         for (const auto& transition : outgoing_transitions_->at(state)) {
            new_states.insert(transition->state_destination_);
         }
      }
   }

   int size_before = states.size();
   states.insert(new_states.begin(), new_states.end());
   if (size_before == states.size()) {
      return;
   }
   else {
      reachableStatesFrom(states);
   }
}

template<class F>
std::set<int> FSM<F>::reachableStates() const
{
   return reachableStatesFrom(INVALID_STATE_NUM);
}

template<class F>
std::set<int> FSM<F>::reachableStatesFrom(const int state) const
{
   std::set<int> states{ state };
   reachableStatesFrom(states);
   return states;
}

template<class F>
void FSM<F>::resetMathOperators()
{
   parser_->init();
   parser_->addDefaultDynamicTerms();
   StaticHelper::addNDetDummyFunctions(parser_, data_);
}

template<class F>
void FSM<F>::setGraphvizStateFadeoutValue(const int value) {
    graphviz_state_fadeout_value_ = static_cast<float>(value);
}

template<class F>
float FSM<F>::evaluateFormula(const std::string& formula) const
{
   return MathStruct::parseMathStruct(formula, true, false, parser_)->toTermIfApplicable()->eval(data_, parser_);
}

template<class F>
FSM<F> FSM<F>::randomFSM(
   const int numStates, 
   const int prob, 
   const int cond_depth, 
   const int cb_depth, 
   const bool simplifyConditions, 
   const std::set<std::string> var_names,
   const long seed)
{
   FSM<F> m(std::make_shared<FSMResolverRemainOnNoTransitionAndObeyInsertionOrder>());
   long seeed = seed == 31415926 ? std::time(nullptr) : seed;
   std::srand(seeed);
   auto d = std::make_shared<DataPack>();
   std::vector<std::string> var_names_vec(var_names.begin(), var_names.end());

   for (int i = INITIAL_STATE_NUM; i < INITIAL_STATE_NUM + numStates; i++) {
      for (int j = INITIAL_STATE_NUM; j < INITIAL_STATE_NUM + numStates; j++) {
         if (std::rand() % 100 <= prob && m.isReachable(INITIAL_STATE_NUM, i)) {
            std::shared_ptr<Term> formula;
            int max = 3; // Try a max of this many iterations to generate a non-const condition.
            int count = 0;

            while (count <= max && (!formula || formula->isOverallConstant())) {
               count++;

               formula = MathStruct::randomTerm(
                  cond_depth,
                  { TERMS_LOGIC, TERMS_ARITH, TERMS_SPECIAL, var_names },
                  256,
                  10,
                  std::rand());
            }

            if (simplifyConditions) {
               formula = MathStruct::simplify(formula);
            }

            auto ttrans = std::make_shared<FSMTransition>(i, j, formula, d);
            m.addTransition(ttrans);
         }
      }
   }

   m.state_id_to_state_name_;
   std::string count = "A";
   std::string formula_name = "";

   for (const auto& state : m.states_plain_set_) {
      if (state != INVALID_STATE_NUM) {
         if (std::rand() % 2 || formula_name.empty()) { // Use existing or new callback.
            auto formula = MathStruct::randomCBTerm(
               cond_depth,
               cb_depth,
               { TERMS_LOGIC, TERMS_ARITH },
               var_names_vec,
               256,
               10,
               std::rand());

            if (simplifyConditions) {
               formula = MathStruct::simplify(formula);
            }

            formula_name = "cb" + count;
            m.defineNewMathOperator(formula_name, formula);
         }

         std::string state_name = "state" + count;
         m.addInternalActionToState(state, m.getParser()->termFactory(formula_name, std::vector<std::shared_ptr<Term>>{}));
         m.setStateName(state, state_name);

         count = StaticHelper::incrementAlphabeticNum(count);
      }
   }

   return m;
}

template<class F>
inline bool FSM<F>::testFanningOut(
   const int seed, 
   const int num_of_states, 
   const int num_of_vd_states, 
   const int cond_depth,
   const int cb_depth,
   const int simulate_for_steps,
   const bool output_ok_too,
   const bool create_pdfs_of_each_step)
{
   FSMs m1 = FSMs::randomFSM(num_of_states, 30, cond_depth, cb_depth, false, {"x", "y", "z"}, seed);
   FSMs v = FSMs::randomFSM(num_of_vd_states, 30, cond_depth, cb_depth, false, {"x", "y", "z"}, seed + 1);
   FSMs m2 = m1.copy();

   m1.fanOutAllNonAtomicCallbacksToStates();
   //m1.simplifyFSM();
   m1.encapsulateLongConditionsIntoFunctions();

   m1.addNote("Base states: " + std::to_string(m2.states_plain_set_.size()) + ", fanned-out states: " + std::to_string(m1.states_plain_set_.size()));

   bool result = m2.testEquivalencyWith(m1, simulate_for_steps, create_pdfs_of_each_step, v);

   if (!result || output_ok_too) {
      //std::cout << std::endl << "----------------------------------------------------" << std::endl;
      //std::cout << "Original FSM:" << std::endl << m2.getDOTCodeOfCurrentGraph() << std::endl;
      //std::cout << std::endl << "Fanned-out FSM:" << std::endl << m1.getDOTCodeOfCurrentGraph() << std::endl;
      //std::cout << std::endl << "Variable disturber:" << std::endl << v.getDOTCodeOfCurrentGraph() << std::endl;

      if (result) {
         std::cout << OK_COLOR << "==> Base FSM (" + std::to_string(m2.states_plain_set_.size()) + " states) and fanned-out FSM (" + std::to_string(m1.states_plain_set_.size()) + " states) have passed the eqivalency test for " << std::to_string(simulate_for_steps) << " simulation steps." << RESET_COLOR << std::endl << std::endl;
      }
      else {
         std::cout << FAILED_COLOR << "==> Base FSM is not eqivalent to fanned-out FSM." << RESET_COLOR << std::endl << std::endl;
      }
   }

   return result;
}

inline std::string getMappingNameForTest(const std::string& enum_val_name) {
   return enum_val_name + "_MAPPING";
}

template<class F>
FSM<F>::FSM(
   const std::shared_ptr<FSMResolver> resolver,
   const NonDeterminismHandling non_determinism_option,
   const std::shared_ptr<DataPack> data,
   const std::shared_ptr<FormulaParser> parser,
   const FSMJitLevel jit_level,
   const std::shared_ptr<mc::TypeAbstractionLayer> type_abstraction_layer
) :
   Parsable("FSM"),

   painter_image_(std::make_shared<Image>(100, 100)),
   parser_(parser ? parser : std::make_shared<FormulaParser>()),
   resolver_(resolver),
   data_(std::make_shared<DataPack>()),
   non_determinism_option_(non_determinism_option),
   jit_level_(jit_level),
   transitions_plain_set_(std::make_shared<std::vector<std::shared_ptr<FSMTransition>>>()),
   states_plain_set_(),
   outgoing_transitions_(std::make_shared<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>>()),
   incoming_transitions_(std::make_shared<std::map<int, std::vector<std::shared_ptr<FSMTransition>>>>()),
   type_abstraction_layer_(type_abstraction_layer)
{
   resetStepCounter();
   resetFSMTopology();
   registerDataPack(data);

   if (!parser) {
      resetMathOperators();
   }

   registerResolver(resolver_);

   addFailableChild(parser_, "");
   //addFailableChild(resolver_factory_, "");

   if (type_abstraction_layer_) {
      addFailableChild(type_abstraction_layer_);
   }
}

template<class F>
void FSM<F>::defineNewMathOperator(const std::string& op_name, const std::string& formula_string)
{
   std::shared_ptr<Term> formula{ MathStruct::parseMathStruct(formula_string, true, false, parser_)->toTermIfApplicable() };
   defineNewMathOperator(op_name, formula);
}

template<class F>
void FSM<F>::defineNewMathOperator(const std::string& op_name, const std::shared_ptr<Term>& formula)
{
   const OutputFormatEnum out_type = OutputFormatEnum::prefix; // For now prefix notation predefined for all operators.
   const AssociativityTypeEnum assoc = AssociativityTypeEnum::left; // Since it's prefix, don't care about associativity, ...
   const int prio = -1;                  // ...prio, ...
   const bool associative = false;       // ...if it's associative, ...
   const bool commutative = false;       // ...or if it's commutative. I.e., these values could be anything.

   const int par_num = StaticHelper::deriveParNum(formula);

   OperatorStructure op_struct(out_type, assoc, par_num, prio, op_name, associative, commutative);
   parser_->addDynamicTerm(op_struct, formula);
}

template<class F>
void FSM<F>::addTransition(
   std::shared_ptr<FSMTransition> trans,
   const bool create_even_unnecessary_implicit_transitions)
{
   if (prohibit_double_edges_) {
      auto v = outgoing_transitions_->find(trans->state_source_);

      if (v != outgoing_transitions_->end()) {
         for (auto& tra : v->second) {
            if (tra->state_destination_ == trans->state_destination_) {
               if (!tra->condition_->isStructurallyEqual(trans->condition_)) {
                  tra->condition_ = std::make_shared<TermLogicOr>(tra->condition_->toTermIfApplicable(), trans->condition_->toTermIfApplicable());
               }

               return;
            }
         }
      }
   } else { // TODO: Should be parametrizable??
      // Prohibit at least structurally equal transitions.
      auto v = outgoing_transitions_->find(trans->state_source_);

      if (v != outgoing_transitions_->end()) {
         for (auto& tra : v->second) {
            if (tra->state_destination_ == trans->state_destination_) {
               if (tra->condition_->isStructurallyEqual(trans->condition_)) {
                  return;
               }
            }
         }
      }
   }

   if (!create_even_unnecessary_implicit_transitions && resolver_->getRedundantState(trans->state_source_) == trans->state_destination_) return;
    
   (*outgoing_transitions_)[trans->state_source_].push_back(trans);
   (*incoming_transitions_)[trans->state_destination_].push_back(trans);
   transitions_plain_set_->push_back(trans);
   states_plain_set_.insert(trans->state_source_);
   states_plain_set_.insert(trans->state_destination_);
   state_id_to_action_ids_.insert({trans->state_source_, {}}); // Note that insert does nothing if element already exists.
   state_id_to_action_ids_.insert({trans->state_destination_, {}});

   if (steps_at_last_visit_.find(trans->state_source_) == steps_at_last_visit_.end()) steps_at_last_visit_[trans->state_source_] = std::numeric_limits<int>::min();
   if (steps_at_last_visit_.find(trans->state_destination_) == steps_at_last_visit_.end()) steps_at_last_visit_[trans->state_destination_] = std::numeric_limits<int>::min();

#if defined(ASMJIT_ENABLED)
   if (jit_level_ == FSMJitLevel::jit_cond)
   {
       addNote(std::string("Creating assembly for condition... ") + trans->toString());
       trans->condition_->createAssembly(data_, parser_, true);
   }
#endif

   setDefaultStateNameIfUnspecified(trans->state_source_);
   setDefaultStateNameIfUnspecified(trans->state_destination_);

   addStateVariable(trans->state_source_);
   addStateVariable(trans->state_destination_);
}

template<class F>
void FSM<F>::removeTransition(const std::shared_ptr<fsm::FSMTransition> trans)
{
   auto& plain_vec = *transitions_plain_set_;
   auto& out_vec = (*outgoing_transitions_)[trans->state_source_];
   auto& in_vec = (*incoming_transitions_)[trans->state_destination_];

   plain_vec.erase(std::remove(plain_vec.begin(), plain_vec.end(), trans), plain_vec.end());
   out_vec.erase(std::remove(out_vec.begin(), out_vec.end(), trans), out_vec.end());
   in_vec.erase(std::remove(in_vec.begin(), in_vec.end(), trans), in_vec.end());
}

template<class F>
void FSM<F>::removeState(const int id)
{
   std::vector<std::shared_ptr<FSMTransition>> to_remove;

   for (const auto trans : (*outgoing_transitions_)[id]) {
      to_remove.push_back(trans);
   }

   for (const auto trans : (*incoming_transitions_)[id]) {
      to_remove.push_back(trans);
   }

   for (const auto& trans : to_remove) {
      removeTransition(trans);
   }

   state_id_to_action_ids_[id].clear();
   state_id_to_state_name_[id] = std::to_string(id);
   states_plain_set_.erase(id);

   for (auto& cb_state : cb_clusters_) {
      cb_state.second.erase(id);
   }
}

template<class F>
inline void FSM<F>::applyToAllCallbacksAndConditionsAndMCItems(
   const std::function<void(std::shared_ptr<MathStruct>)>& f, 
   const bool include_callbacks, 
   const bool include_transitions, 
   const bool include_functions)
{
   if (include_callbacks) {
      for (const auto& state_id : states_plain_set_) {
         for (auto& action : getInternalActionsOfState(state_id)) {
            auto fmla = _id(action);
            fmla->applyToMeAndMyChildren(f, TraverseCompoundsType::go_into_compound_structures);
            action = fmla->getOperands()[0];
         }
      }
   }

   if (include_transitions) {
      for (auto& trans : *transitions_plain_set_) {
         auto fmla = _id(trans->condition_);
         fmla->applyToMeAndMyChildren(f, TraverseCompoundsType::go_into_compound_structures);
         trans->condition_ = fmla->getOperands()[0];
      }
   }

   if (include_functions) {
      for (auto& function_pairs : parser_->getDynamicTermMetas()) {
         for (auto& function_pair : function_pairs.second) {
            auto fmla = _id(function_pair.second);
            fmla->applyToMeAndMyChildren(f, TraverseCompoundsType::go_into_compound_structures);
            function_pair.second = fmla->getOperands()[0];
         }
      }
   }
}

template<class F>
inline void FSM<F>::applyConstnessToAllVariablesInCallbacksAndConditionsAndMCItems()
{
   applyToAllCallbacksAndConditionsAndMCItems([this](const MathStructPtr m) {
      auto m_var = m->toVariableIfApplicable();

      if (m_var && getData()->isConst(m_var->getVariableName())) {
         m_var->setConstVariable(getData()->getSingleVal(m_var->getVariableName()));
      }
   }, true, true, true, true);
}

template<class F>
void FSM<F>::addCompletingTransition(
   const int source_state, 
   const int destination_state,
   const bool create_even_unnecessary_implicit_transitions)
{
   auto current_transitions(extractOnlyNonFalseConditions(outgoing_transitions_->at(source_state)));

   if (current_transitions.empty())
   {
      addTransition(source_state, destination_state, _true(), create_even_unnecessary_implicit_transitions);
   }
   else {
      std::shared_ptr<Term> condition(current_transitions[0]->condition_->toTermIfApplicable()->copy()->toTermIfApplicable());
      if (condition->isAlwaysTrue()) return;

      for (int i = 1; i < current_transitions.size(); ++i)
      {
         std::shared_ptr<Term> condition2(current_transitions[i]->condition_->toTermIfApplicable()->copy()->toTermIfApplicable());
         if (condition2->isAlwaysTrue()) return;
         condition = std::make_shared<TermLogicOr>(std::vector<std::shared_ptr<Term>>({ condition, condition2 }));
      }

      condition = _not(condition);
      if (create_even_unnecessary_implicit_transitions || resolver_->getRedundantState(source_state) != destination_state) {
         // Extract condition to new math operator to decrowd visualization.
         std::string op_name = "complete_";
         op_name.append(state_id_to_state_name_[source_state]);
         defineNewMathOperator(op_name, condition);
         condition = MathStruct::parseMathStruct(op_name + "()", true, false, parser_)->toTermIfApplicable();
      }

      addTransition(source_state, destination_state, condition, create_even_unnecessary_implicit_transitions);
   }
}

template<class F>
void FSM<F>::addCompletingTransition(
   const std::string& source_state, 
   const std::string& destination_state,
   const bool create_even_unnecessary_implicit_transitions)
{
   addCompletingTransition(state_name_to_state_id_[source_state], state_name_to_state_id_[destination_state], create_even_unnecessary_implicit_transitions);
}

template<class F>
bool FSM<F>::parseProgram(const std::string& fsm_commands)
{
#if defined(FSM_DEBUG)
   std::cout << std::endl << "<FSM> Parsing program..." << std::endl;
#endif
   
   auto line_mapping = StaticHelper::getMappingFromCommandNumToLineNum(fsm_commands);
   const auto commands = StaticHelper::split(StaticHelper::removeComments(fsm_commands), PROGRAM_COMMAND_SEPARATOR);
   int command_num = 0;  
       
   for (const auto& command : commands) {
      command_num++;
       
      if (!StaticHelper::isEmptyExceptWhiteSpaces(command)) {
#if defined(FSM_DEBUG)
            std::string copy = command;
            StaticHelper::trim(copy);
            std::cout << "<FSM> Parsing: '" << copy << "'... ";
#endif
            bool error_in_current_command = false;
            const auto pair = StaticHelper::split(command, PROGRAM_DEFINITION_SEPARATOR);

            if (StaticHelper::removeWhiteSpace(command)[0] == PROGRAM_PREPROCESSOR_DENOTER) { // #
               auto tight_command = StaticHelper::removeWhiteSpace(command);

                if (resolver_factory_.isResolverCommand(command)) {
                    error_in_current_command = !resolver_factory_.parseProgram(command);

                    if (!error_in_current_command) {
                       registerResolver(resolver_factory_.getGeneratedResolver());
                    }
                } else if (StaticHelper::stringEndsWith(command, "FIND_DOUBLE_CODE")) {
                   checkForDoubleCode();
                } else if (StaticHelper::stringEndsWith(command, "FIND_SIMPLIFIABLE_CODE")) {
                   checkForSimplifiableCode();
                } else if (StaticHelper::stringEndsWith(command, "FIND_DOUBLE_CODE_ALL")) {
                   checkForDoubleCode(TraverseCompoundsType::go_into_compound_structures);
                } else if (StaticHelper::stringEndsWith(command, "FIND_SIMPLIFIABLE_CODE_ALL")) {
                   checkForSimplifiableCode(TraverseCompoundsType::go_into_compound_structures);
                } else if (StaticHelper::stringStartsWith(tight_command, PREP_ADDITIONAL_STATE_VARIABLE_NAME)) {
                   auto pair = StaticHelper::split(tight_command, "=");
                   setAdditionalStateVarName(pair[1]);
                } else {
                   addError("Not a valid preprocessor command in '" + command + "'.");
                   error_in_current_command = true;
                }
            }
            else if (command.find(FSM_PROGRAM_TRANSITION_DENOTER) != std::string::npos) { // FSM command. ->
               auto pair1 = StaticHelper::split(command, FSM_PROGRAM_TRANSITION_DENOTER);
               auto pair2 = StaticHelper::split(pair1[1], FSM_PROGRAM_TRANSITION_SEPARATOR);
               std::string state1 = StaticHelper::removeWhiteSpace(pair1[0]);
               std::string state2 = StaticHelper::removeWhiteSpace(pair2[0]);

               if (state1.empty()) {
                  setInitialStateName(state2);
               }
               else {
                  std::string formula_str = "1";
                  if (pair2.size() >= 2) {
                     formula_str = pair2[1];
                  }

                  std::shared_ptr<Term> formula = MathStruct::parseMathStruct(formula_str, true, false, parser_)->toTermIfApplicable();
                  
                  error_in_current_command = formula->serialize() == PARSING_ERROR_STRING;

                  addTransition(state1, state2, formula);
               }
            }
            else if (command.find(FSM_PROGRAM_COMMAND_DENOTER) != std::string::npos) { // Action assoc. |>
               auto pair1 = StaticHelper::split(command, FSM_PROGRAM_COMMAND_DENOTER);
               std::string state = StaticHelper::removeWhiteSpace(pair1[0]);
               std::string commds_str = pair1[1];

               auto commds = StaticHelper::split(commds_str, PROGRAM_OPENING_FORMULA_BRACKET, FUNC_ALL_STRINGS_TO_TRUE, true);
               commds = StaticHelper::split(commds, PROGRAM_CLOSING_FORMULA_BRACKET);

               for (const auto& commd : commds) {  // TODO: I hope this is equivalent to before, see change from 2023-07-04
                  std::vector<std::string> vec{};

                  if (commd[0] == PROGRAM_OPENING_FORMULA_BRACKET[0]) {
                     vec = { commd };
                  }
                  else {
                     vec = StaticHelper::split(commd, FSM_PROGRAM_COMMAND_SEPARATOR);
                  }

                  commds.insert(commds.end(), vec.begin(), vec.end());
               }

               for (std::string commd : commds) {
                  if (!StaticHelper::removeWhiteSpace(commd).empty()) {
                     if (commd[0] == PROGRAM_OPENING_FORMULA_BRACKET[0]) {
                        std::string formula_str = StaticHelper::replaceAll(commd, PROGRAM_OPENING_FORMULA_BRACKET, "");
                        std::shared_ptr<Term> formula = MathStruct::parseMathStruct(formula_str, true, false, parser_)->toTermIfApplicable();
                        addInternalActionToState(state, formula);
                     } else {
                        associateStateNameToExistingAction(state, StaticHelper::removeWhiteSpace(commd));
                     }
                  }
               }
            }
            else if (pair.size() == 1 // FormulaParser command. No :=
                     || pair[0].find(PROGRAM_OPENING_OP_STRUCT_BRACKET_LEGACY) != std::string::npos && pair[0].find(PROGRAM_OPENING_ARRAY_BRACKET) == std::string::npos) {
               error_in_current_command = !parser_->parseProgram(command);
            }
            else if (true) { // DataPack command.
               data_->setParserForProgramParsing(parser_);
               error_in_current_command = !data_->parseProgram(command);
            }
            else { // Error.
               error_in_current_command = true;
            }

            if (error_in_current_command) {
               std::string copy = command;
               StaticHelper::trim(copy);
               std::string lines_str = "line";
               auto line_nums = line_mapping[command_num];

               if (line_nums.size() > 1) {
                  lines_str.append("s ");
                  lines_str.append(std::to_string(line_nums[0]));
                  lines_str.append("-");
                  lines_str.append(std::to_string(line_nums[line_nums.size() - 1]));
               } else {
                  lines_str.append(" ");
                  lines_str.append(std::to_string(line_nums[0]));
               }

               const std::string error_command_msg = "Could not parse command '" + copy + "' in " + lines_str + ".";
               addError(error_command_msg);
            }

#if defined(FSM_DEBUG)
            if (error_in_current_command) {
               std::cout << FAILED_COLOR << "FAILED." << RESET_COLOR << std::endl;
            } else {
               std::cout << OK_COLOR << "OK." << RESET_COLOR << std::endl;
            }
#endif
         }
   }

   if (states_plain_set_.find(getCurrentState()) == states_plain_set_.end()) {
      setCurrentState(INVALID_STATE_NUM);
   }

   parser_->checkForUndeclaredVariables(data_);

   return !hasErrorOccurred();
}

template<class F>
inline void FSM<F>::replayCounterExampleForOneStep(const MCTrace& ce, int& i, int& loop, const bool always_replay_from_start)
{
   if (always_replay_from_start) {
      for (int cnt = 0; cnt <= i;) {
         const bool last = cnt == i && cnt == ce.size() - 2;

         replayCounterExampleForOneStep(ce, cnt, loop, false);

         if (last) {
            i = cnt;
            return;
         }
      }
   }
   else {
      const auto& line = ce.at(i);
      
      if (line.first == "LOOP") {
         loop = i;
         data_->addOrSetSingleVal("mcLoop", 1);
      }
      else {
         data_->addOrSetSingleVal("mcState", std::stoi(StaticHelper::split(line.first, '.')[1]));
         for (const auto& varval : line.second) {
            const std::string& var = varval.first;
            const std::string& val = varval.second;
            const float result = MathStruct::parseMathStruct(val, true, false, parser_)->eval(data_);

            if (StaticHelper::stringStartsWith(var, ".")) {
               data_->addArrayAndOrSetArrayVal(var, registration_number_, result);
            }
            else {
               data_->addOrSetSingleVal(var, result);
            }
         }
      }

      manageStepCounter(getCurrentState());
   }

   i++;
   if (i == ce.size() - 1) {
      i = loop; // TODO: Should be loop + 1, but look if always_replay_from_start then still works.
   }
}

#ifndef DISALLOW_EXTERNAL_ASSOCIATIONS
template<class F>
void FSM<F>::initAndAssociateVariableToExternalBool(const std::string & variable_name, bool* external_variable)
{
   data_->associateSingleValWithExternalBool(variable_name, external_variable);
}

template<class F>
void FSM<F>::initAndAssociateVariableToExternalChar(const std::string & variable_name, char* external_variable)
{
   data_->associateSingleValWithExternalChar(variable_name, external_variable);
}

template<class F>
void FSM<F>::initAndAssociateVariableToExternalFloat(const std::string & variable_name, float* external_variable)
{
   data_->associateSingleValWithExternalFloat(variable_name, external_variable);
}
#endif

template<class F>
void FSM<F>::step(const SteppingOrderEnum ordering)
{
   bool stop_at_implicit_cb_states = 
      SteppingOrderEnum::transition_then_execute_callbacks_and_stop_at_cb_states == ordering
      || SteppingOrderEnum::execute_callbacks_then_transition_and_stop_at_cb_states == ordering
      || SteppingOrderEnum::transition_only_and_stop_at_cb_states == ordering;

   if (SteppingOrderEnum::transition_then_execute_callbacks == ordering || SteppingOrderEnum::transition_then_execute_callbacks_and_stop_at_cb_states == ordering) {
      transitionToNextState();
      executeCallbacksOfCurrentState();
   }
   else if (SteppingOrderEnum::execute_callbacks_then_transition == ordering || SteppingOrderEnum::execute_callbacks_then_transition_and_stop_at_cb_states == ordering) {
      executeCallbacksOfCurrentState();
      transitionToNextState();
   }
   else if (SteppingOrderEnum::transition_only == ordering || SteppingOrderEnum::transition_only_and_stop_at_cb_states == ordering) {
      transitionToNextState();
   }
   else if (SteppingOrderEnum::execute_callbacks_only == ordering) {
      executeCallbacksOfCurrentState();
   }

   manageStepCounter(getCurrentState());

   if (!stop_at_implicit_cb_states && isImplicitCBState(getCurrentState())) {
      step(ordering);
   }
}

template<class F>
inline void FSM<F>::manageStepCounter(const int currentState)
{
   steps_at_last_visit_[currentState] = step_counter_;
   incrementStepCounter();
}

template<class F>
void FSM<F>::transitionToNextState()
{
   if (jit_level_ == FSMJitLevel::jit_full)
   {
      setCurrentState(closed_form_->eval(data_, parser_));
   } else {
      std::vector<int> next_states = resolver_->retrieveNextStates(
          non_determinism_option_ == NonDeterminismHandling::ignore // Break after first found if non-det ok.
             || non_determinism_option_ == NonDeterminismHandling::note_error_on_deadlock,
          non_determinism_option_ == NonDeterminismHandling::ignore // Fill with redundant state if deadlock ok.
             || non_determinism_option_ == NonDeterminismHandling::note_error_on_non_determinism
      );

      if ((non_determinism_option_ == NonDeterminismHandling::note_error_on_deadlock
        || non_determinism_option_ == NonDeterminismHandling::note_error_on_non_determinism_or_deadlock)
        && next_states.empty()) {
         next_states = resolver_->retrieveNextStates(true, true);
         addError("Ended up in deadlock, no active outgoing transition from state '" + state_id_to_state_name_[getCurrentState()] + "'. Using redundant transition to '" + state_id_to_state_name_[next_states[0]] + "'.");
      }
      
      if ((non_determinism_option_ == NonDeterminismHandling::note_error_on_non_determinism
        || non_determinism_option_ == NonDeterminismHandling::note_error_on_non_determinism_or_deadlock)
        && next_states.size() > 1) {
         std::string nondet_options = "\n->";

         for (const auto& state : next_states) {
            nondet_options += state_id_to_state_name_[state] + "\n";
         }

         addError("Ended up with non-deterministic choice in state '" + state_id_to_state_name_[getCurrentState()] + "'. "
            "The possible next states are (of which I will follow the first): " + nondet_options);
      }

      setCurrentState(next_states[0]);
   }
}

template<class F>
void FSM<F>::executeCallbacksOfCurrentState()
{
   auto actions = state_id_to_action_ids_[getCurrentState()];
   for (const auto& action : actions)
   {
       action_id_to_action_[action]->call(data_, parser_, getCurrentStateVarName());
   }
}

template<class F>
std::shared_ptr<Term> FSM<F>::generateClosedForm()
{
   closed_form_ = resolver_->generateClosedForm();

#if defined(ASMJIT_ENABLED)
   closed_form_->createAssembly(data_, parser_, true);
#endif

#if defined(FSM_DEBUG)
   std::cout << "<FSM> Generated closed-form representation: " << *closed_form_ << std::endl;
#endif

   return closed_form_;
}

template<class F>
void FSM<F>::findAllSetVariables() const
{
   //for (const auto& f : parser_->getDynamicTermMetas()) { // TODO: Correct to comment this?
   //   findSetVariables(f.second);                         // Dynamic terms should only be considered if actually used in an action(?)
   //}                                                      // Therefore, we go into compounds in the next step.

   for (const auto& f : action_id_to_action_) {
      auto formula = f.second->getFormula();
      if (formula) {
         auto fsm_controlled{ formula->findSetVariables(data_) };
         auto inputs{ fsm_controlled.getInputVariables() };
         auto outputs{ fsm_controlled.getOutputVariables() };

         all_fsm_controlled_variables_.insert(inputs.begin(), inputs.end());
         all_fsm_controlled_variables_.insert(outputs.begin(), outputs.end());
      }
   }

   all_fsm_controlled_variables_.insert(getCurrentStateVarName());
   //all_constants_to_model_checker_.insert("expectedFctCycleTime"); // TODO: Make this more general by introducing vfm constants.
   //all_constants_to_model_checker_.insert("numberOfToleratedActuatorStandbyCycles");

   //for (std::string s : data_->getAllSingleValVarNames()) {
   //   if (!all_fsm_controlled_variables_.count(s)
   //        && !all_constants_to_model_checker_.count(s)
   //        && !StaticHelper::stringStartsWith(s, "enum_")
   //        && !StaticHelper::stringStartsWith(s, "state_")
   //        && s != "const_approx"
   //        && s != "lastState"
   //        && s != "false"
   //        && s != "true"
   //        && s != getStepCountVarName()) {
   //      all_external_variables_.insert(s);
   //   }
   //}
}

template<class F>
inline bool FSM<F>::isImplicitCBState(const int state_id) const
{
   for (const auto& cluster : cb_clusters_) {
      if (cluster.second.count(state_id)) {
         return true;
      }
   }

   return false;
}

template<class F>
void FSM<F>::addInternalActionToState(const int state_id, const std::shared_ptr<Term> formula)
{
   addUnconnectedStateIfNotExisting(state_id);

   int action_id = smallestFreeActionID();
   action_id_to_action_[action_id] = std::make_shared<TInternalFunctor<F>>();
   action_id_to_action_[action_id]->setFormula(formula);
   setActionName(action_id, action_id_to_action_[action_id]->getName());
   associateStateIdToActionId(state_id, action_id);
}

template<class F>
void FSM<F>::addInternalActionToState(const std::string& state_name, const std::shared_ptr<Term> formula)
{
   addInternalActionToState(state_name_to_state_id_[state_name], formula);
}

template<class F>
inline void FSM<F>::addInternalActionToState(const std::string& state_name, const std::string& formula)
{
   addInternalActionToState(state_name, MathStruct::parseMathStruct(formula, true, false, parser_)->toTermIfApplicable());
}

template<class F>
inline std::vector<TermPtr> FSM<F>::getInternalActionsOfState(const int state_num) const
{
   std::vector<TermPtr> vec;

   for (const auto action_id : state_id_to_action_ids_.at(state_num)) {
      vec.push_back(action_id_to_action_.at(action_id)->getFormula());
   }

   return vec;
}

template<class F>
inline std::vector<TermPtr> FSM<F>::getInternalActionsOfState(const std::string& state) const
{
   return getInternalActionsOfState(state_name_to_state_id_.at(state));
}

template<class F>
int FSM<F>::addAction(
   F* _pt2Object, 
   void(F::*_fpt)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName),
   const std::string& specify_name,
   const int specifyID)
{
   int action_id = specifyID;

   if (action_id < 0) { // No valid ID specified.
      for (const auto& action_pair : action_id_to_action_) {
         if (action_pair.second->getpt2Object() == _pt2Object && action_pair.second->getFpt() == _fpt) {
            setActionName(action_pair.first, specify_name);
            return action_pair.first; // Already in map.
         }
      }

      action_id = smallestFreeActionID(); // Not yet in map, create new entry.
   }

   TSpecificFunctor<F> specFunc(_pt2Object, _fpt, specify_name);
   action_id_to_action_[action_id] = std::make_shared<TSpecificFunctor<F>>(specFunc);

   setActionName(action_id, specify_name);

   return action_id;
}

template<class F>
void FSM<F>::associateStateIdToActionId(const int state_id, const int action_id)
{
   state_id_to_action_ids_[state_id].push_back(action_id);
}

template<class F>
inline void FSM<F>::associateStateNameToActionId(const std::string& state_name, const int action_id)
{
   associateStateIdToActionId(state_name_to_state_id_[state_name], action_id);
}

template<class F>
int FSM<F>::associateStateIdToAction(
   const int state_id, 
   F* _pt2Object, 
   void(F::*_fpt)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName),
   const std::string& action_name)
{
   int action_id = addAction(_pt2Object, _fpt, action_name);
   associateStateIdToActionId(state_id, action_id);

   return action_id;
}

template<class F>
void FSM<F>::associateStateIdToExistingAction(const int state_id, const std::string& action_name)
{
   associateStateIdToActionId(state_id, action_name_to_action_id_[action_name]);
}

template<class F>
void FSM<F>::associateStateNameToExistingAction(const std::string& state_name, const std::string& action_name)
{
   associateStateIdToExistingAction(state_name_to_state_id_[state_name], action_name);
}

template<class F>
void FSM<F>::setActionName(const int action_id, const std::string& action_name)
{
   std::string an = action_name;

   if (an.empty()) {
      an = std::to_string(action_id);
   }

   action_name_to_action_id_[an] = action_id;
}

template<class F>
inline void FSM<F>::removeActionsFromState(const int state_id)
{
   state_id_to_action_ids_[state_id].clear();
}

template<class F>
inline void FSM<F>::setAdditionalStateVarName(const std::string& name)
{
   additional_state_var_name_ = name;
}

template<class F>
int FSM<F>::associateStateToAction(
   const std::string& state_name, 
   F* _pt2Object, 
   void(F::*_fpt)(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName),
   const std::string& action_name)
{
   addUnconnectedStateIfNotExisting(state_name);
   return associateStateIdToAction(state_name_to_state_id_[state_name], _pt2Object, _fpt, action_name);
}

template<class F>
std::shared_ptr<DataPack> FSM<F>::getData() const
{
   return data_;
}

template<class F>
inline std::shared_ptr<FormulaParser> FSM<F>::getParser() const
{
   return parser_;
}

template<class F>
int FSM<F>::getInternalNumFromStateName(const std::string& name)
{
   return state_name_to_state_id_[name];
}

template<class F>
std::vector<int> FSM<F>::getActionIDsOfStateID(const int state_num)
{
   return state_id_to_action_ids_[state_num];
}

template<class F>
inline int FSM<F>::getNumOfActions() const
{
   return action_id_to_action_.size();
}

template<class F>
inline std::shared_ptr<std::vector<std::shared_ptr<FSMTransition>>> FSM<F>::getTransitions()
{
   return transitions_plain_set_;
}

template<class F>
std::string FSM<F>::getNameFromInternalStateNum(const int num)
{
   return state_id_to_state_name_[num];
}

template<class F>
void FSM<F>::fanOutMultipleCallbacksToIndividualStates()
{
   completeAllStatesToSelf(true);

   for (const auto& state_id : states_plain_set_) {
      auto& callbacks = state_id_to_action_ids_[state_id];

      if (callbacks.size() > 1) {
         std::string state_name = state_id_to_state_name_[state_id];
         int last_state_id = state_id;
         int first_callback_state_id = INVALID_STATE_NUM;
         std::string count = "A";

         for (int i = 1; i < callbacks.size(); i++) {
            std::string state_callback_name = state_name + "_" + count;
            int new_state_id = addUnconnectedStateIfNotExisting(state_callback_name);
            addTransition(last_state_id, new_state_id, "1");

            associateStateIdToActionId(new_state_id, callbacks[i]);

            last_state_id = new_state_id;

            if (i == 1) {
               first_callback_state_id = new_state_id;
            }

            count = StaticHelper::incrementAlphabeticNum(count);
         }

         callbacks.erase(callbacks.begin() + 1, callbacks.end());

         std::vector<std::shared_ptr<FSMTransition>> to_erase;
         for (std::shared_ptr<FSMTransition> trans : outgoing_transitions_->at(state_id)) {
            if (trans->state_destination_ != first_callback_state_id) {
               to_erase.push_back(trans);
               addTransition(last_state_id, trans->state_destination_, trans->condition_->copy(), true);
            }
         }

         for (const auto trans : to_erase) {
            removeTransition(trans);
         }
      }
   }
}

template<class F>
void FSM<F>::insertUnfoldedCallbackInterface(const std::string& cb_name, const CBInterface& cb_interface, const std::shared_ptr<MathStruct> formula) {
   unfolded_callback_interfaces_.insert({ cb_name, cb_interface });
   unfolded_callback_interfaces_formulae_.insert({ cb_name, formula });
}

template<class F>
CBInterface FSM<F>::fanOutSingleCallbackToStates(const std::shared_ptr<Term> callback, const std::string optional_cb_name, const bool use_existing_callback_if_available)
{
   std::shared_ptr<Term> plain_cb = MathStruct::flattenFormula(callback);
   std::string cb_serialized = plain_cb->serialize();

   std::string cb_name = (optional_cb_name.empty() ? std::string("cb_") + std::to_string(fan_out_cb_counter_++) + "_" : optional_cb_name);

   if (!unfolded_callback_counters_.count(cb_name)) {
      unfolded_callback_counters_.insert({cb_name, 0});
      cb_clusters_.insert({cb_name, {}});
   }

   CBInterface result;
   bool has_been_fanned_out_already = unfolded_callback_interfaces_.count(cb_serialized);

   // Avoids cutting into the middle of an existing cb. TODO: Is this the best way / even correct in all cases?
   bool has_same_structure_as_former = true;// has_been_fanned_out_already && unfolded_callback_interfaces_formulae_[cb_serialized]->isStructurallyEqual(callback);

   if (!use_existing_callback_if_available || !has_been_fanned_out_already || !has_same_structure_as_former) {
      std::shared_ptr<Term> cond_true = std::make_shared<TermVal>(1);

      if (plain_cb->isTermSetVarOrAssignment()) {
         int id = addUnconnectedStateIfNotExisting(cb_name + std::to_string(unfolded_callback_counters_[cb_name]++));
         cb_clusters_.at(cb_name).insert(id);
         addInternalActionToState(id, plain_cb);
         result = {{{id, cond_true}}, {id}};
         insertUnfoldedCallbackInterface(cb_serialized, result, callback);
         return result;
      } else if (plain_cb->isTermSequence()) {
         auto res1 = fanOutSingleCallbackToStates(plain_cb->getOperands()[0], cb_name, false);
         auto res2 = fanOutSingleCallbackToStates(plain_cb->getOperands()[1], cb_name, false);

         CBInterface cb_interface;
         int dummy_state = addUnconnectedStateIfNotExisting(cb_name + std::to_string(unfolded_callback_counters_[cb_name]++));
         cb_clusters_.at(cb_name).insert(dummy_state);

         for (CBOutgoingState co : res1.second) {
            addTransition(co, dummy_state, cond_true);
         }

         for (CBIncomingState ci : res2.first) {
            auto cond = ci.second;
            addTransition(dummy_state, ci.first, cond);
         }

         result = {res1.first, res2.second};
         insertUnfoldedCallbackInterface(cb_serialized, result, callback);
         return result;
     } else if (plain_cb->getOptor() == SYMB_IF_ELSE || plain_cb->getOptor() == SYMB_IF) { // TODO: Use isTermIf()/isTermIfelse().
         auto res1 = fanOutSingleCallbackToStates(plain_cb->getOperands()[1], cb_name, false);
         auto res2 = fanOutSingleCallbackToStates(plain_cb->getOptor() == SYMB_IF_ELSE 
                                                  ? plain_cb->getOperands()[2]
                                                  : std::make_shared<TermVal>(fan_out_cb_dummy_state_counter_++), cb_name, false);
         CBInterface new_interface;

         for (CBIncomingState ci : res1.first) {
            auto cond = plain_cb->getOperands()[0]->copy();
            if (!ci.second->isAlwaysTrue()) {
               cond = std::make_shared<TermLogicAnd>(cond, ci.second);
            }

            new_interface.first.push_back({ ci.first, cond });
         }

         for (CBIncomingState ci : res2.first) {
            std::shared_ptr<Term> cond = _not(plain_cb->getOperands()[0]->copy());

            if (!ci.second->isAlwaysTrue()) {
               cond = std::make_shared<TermLogicAnd>(cond, ci.second);
            }

            new_interface.first.push_back({ ci.first, cond });
         }

         new_interface.second = res1.second;

         for (CBOutgoingState co : res2.second) {
            new_interface.second.push_back(co);
         }

         result = new_interface;
         insertUnfoldedCallbackInterface(cb_serialized, result, callback);
         return result;
      } else {
         int id = addUnconnectedStateIfNotExisting(cb_name + std::to_string(unfolded_callback_counters_[cb_name]++));
         cb_clusters_.at(cb_name).insert(id);
         result = {{{id, cond_true}}, {id}};
         insertUnfoldedCallbackInterface(cb_serialized, result, callback);
         return result;
      }
   }
#ifdef FSM_DEBUG
   else {
      std::cout << "Using existing CB '" << cb_name << "'" << std::endl;
      std::cout << "For formula:" << unfolded_callback_interfaces_formulae_[cb_serialized]->serializeWithinSurroundingFormula() << std::endl;
      std::cout << "New Formula:" << callback->serializeWithinSurroundingFormula() << std::endl << std::endl;
   }
#endif

   return unfolded_callback_interfaces_.at(cb_serialized);
}

template<class F>
void FSM<F>::cleanupCBBottleneckStates()
{
   for (const auto& cb : cb_clusters_) {
      bool keep_going = true;

      while (keep_going) { 
         std::vector<int> to_remove;
         
         for (int id : cb_clusters_[cb.first]) {
            auto trans_ins = (*incoming_transitions_)[id];
            auto trans_outs = (*outgoing_transitions_)[id];

            if ((trans_ins.size() <= 2 || trans_outs.size() <= 2) 
                 && !trans_ins.empty() && !trans_outs.empty() // If ins or outs is empty, it's an interface state.
                 && state_id_to_action_ids_[id].empty()) {
               to_remove.push_back(id);

               for (const auto trans_in : trans_ins) {
                  for (const auto trans_out : trans_outs) {
                     auto cond1 = trans_in->condition_->toTermIfApplicable();
                     auto cond2 = trans_out->condition_->toTermIfApplicable();
                     auto cond = cond1->isAlwaysTrue() ? cond2 : (cond2->isAlwaysTrue() ? cond1 : std::make_shared<TermLogicAnd>(cond1, cond2));
                     addTransition(trans_in->state_source_, trans_out->state_destination_, cond);
                  }
               }
            }
         }

         keep_going = !to_remove.empty();
         for (const auto& id : to_remove) {
            removeState(id);
         }
      }
   }
}

inline std::string getCBName(const std::shared_ptr<MathStruct> cb_formula) {
   if (!cb_formula) {
      return "";
   }

   std::string name = cb_formula->serialize();
   name.erase(std::remove_if(name.begin(), name.end(), [](int c) {return !std::isalnum(c); }), name.end()); // Removes special characters.
   return name;
}

template<class F>
void FSM<F>::fanOutAllNonAtomicCallbacksToStates()
{
   setProhibitDoubleEdges(false);
   fanOutMultipleCallbacksToIndividualStates();

   std::vector<std::shared_ptr<FSMTransition>> to_remove;

   std::map<int, bool> single_cbs; // Find double callbacks.
   for (const auto state_id1 : states_plain_set_) {
      single_cbs[state_id1] = true;

      for (const auto state_id2 : states_plain_set_) {
         if (state_id2 != state_id1
             && !state_id_to_action_ids_[state_id1].empty()
             && !state_id_to_action_ids_[state_id2].empty()) {
            int action_id1 = state_id_to_action_ids_[state_id1][0];
            int action_id2 = state_id_to_action_ids_[state_id2][0];
            auto cb_formula1 = action_id_to_action_[action_id1]->getFormula();
            auto cb_formula2 = action_id_to_action_[action_id2]->getFormula();
            auto name1 = getCBName(cb_formula1);
            auto name2 = getCBName(cb_formula2);
            if (name2 == name1) {
               single_cbs[state_id1] = false;
            }
         }
      }
   }

   for (const auto state_id : states_plain_set_) {
      assert(state_id_to_action_ids_[state_id].size() <= 1); // This is certainly true after fanOutMultipleCallbacksToIndividualStates.

      for (const auto action_id : state_id_to_action_ids_[state_id]) { // This loop will be visited at most once.
         auto cb_formula = action_id_to_action_[action_id]->getFormula();

         if (cb_formula) { // Internal callback (ignore if not).
            auto cb_formula_term = cb_formula->toTermIfApplicable();
            auto cb_flattened_formula = MathStruct::flattenFormula(cb_formula_term);

            if (!cb_flattened_formula->isTermSetVarOrAssignment()) {
               auto name = getCBName(cb_formula);
               bool is_single_occurrence_of_cb = single_cbs[state_id];

               auto cb_interface = fanOutSingleCallbackToStates(cb_formula_term, name);

               int dummy_state_id = INVALID_STATE_NUM;
               if (cb_interface.second.size() > 1) {
                  dummy_state_id = addUnconnectedStateIfNotExisting(name + "_CB_out");

                  int some_cb_state = cb_interface.second[0];

                  for (const auto& pair : cb_clusters_) { // Add "out" state to cluster.
                     if (pair.second.count(some_cb_state)) {
                        cb_clusters_[pair.first].insert(dummy_state_id);
                        break;
                     }
                  }
               }

               for (const auto cbout : cb_interface.second) { // Transitions from CB_OUT to destination states.
                  auto id = cbout;
                  
                  if (dummy_state_id >= 0) {
                     addTransition(id, dummy_state_id, _true());
                  }

                  for (const auto& trans : (*outgoing_transitions_)[state_id]) {
                     auto formula_last = is_single_occurrence_of_cb 
                        ? _true() 
                        : _eq(_var("last"), _val(state_id));

                     addTransition(
                        dummy_state_id >= 0 ? dummy_state_id : id,
                        trans->state_destination_, 
                        _and(trans->condition_->copy(), formula_last));
                     to_remove.push_back(trans);
                  }
               }

               for (const auto& cbin : cb_interface.first) { // Transitions from original state to CB_IN.
                  auto id = cbin.first;
                  auto cond = cbin.second->copy();

                  addTransition(state_id, id, cond);
               }

               state_id_to_action_ids_[state_id].clear();

               if (!is_single_occurrence_of_cb) {
                  addInternalActionToState(state_id, _set("last", _val(state_id)));
               }
            }
         } else {
            addError("Action " + std::to_string(action_id) + " in state " + std::to_string(state_id) + " cannot be fanned out since it is an external callback.");
         }
      }
   }

   for (const auto& trans : to_remove) {
      removeTransition(trans);
   }

   cleanupCBBottleneckStates();

   for (auto& trans : *transitions_plain_set_) {
      trans->condition_ = trans->condition_->copy();
      trans->condition_->setChildrensFathers(true);
   }
}

template<class F>
void FSM<F>::extractTopLevelCommands(const std::shared_ptr<MathStruct> f, std::deque<std::shared_ptr<MathStruct>>& vec)
{
   auto sub_formula{ f };
   while (sub_formula->isTermSequence()) {
      vec.push_front(sub_formula->getTermsJumpIntoCompounds()[1]);
      sub_formula = sub_formula->getTermsJumpIntoCompounds()[0];
   }
   vec.push_front(sub_formula);
}

template<class F>
std::deque<std::shared_ptr<MathStruct>> FSM<F>::extractTopLevelCommandsRecursively(const std::shared_ptr<MathStruct> f)
{
   int count{ 0 };
   std::deque<std::shared_ptr<MathStruct>> vec{};
   extractTopLevelCommands(f, vec);
   std::shared_ptr<bool> trigger_abandon_children_once{ std::make_shared<bool>(false) };

   for (int i = 0; i < vec.size(); i++) {
      auto formula{ vec.at(i) };

      if (formula->isTermIf() || formula->isTermIfelse()) {
         vec.erase(vec.begin() + i);
         
         formula->applyToMeAndMyChildren([trigger_abandon_children_once, &vec, i](const std::shared_ptr<MathStruct> m) {
            auto father{ m->getFatherJumpOverCompound() };
            if (m->isTermSequence() && !father->isTermSequence()) { // TODO: Probably unnecessary check "!father->isTermSequence()".
               std::deque<std::shared_ptr<MathStruct>> vec2{};
               extractTopLevelCommands(m, vec2);
               vec.insert(vec.begin() + i, vec2.begin(), vec2.end());
               *trigger_abandon_children_once = true;
            } else if (m->isTermSetVarOrAssignment() && !father->isTermSequence()) {
               vec.insert(vec.begin() + i, m);
            }
         }, TraverseCompoundsType::avoid_compound_structures, trigger_abandon_children_once);

         i--;
      }
   }

   return vec;
}

template<class F>
void FSM<F>::addSetter(
   std::map<std::string, std::vector<std::pair<int, std::pair<int, std::pair<std::shared_ptr<MathStruct>, std::shared_ptr<MathStruct>>>>>>& map,
   std::map<int, int>& pc_max_values,
   const int state,
   const int pc,
   const std::shared_ptr<MathStruct> formula) const
{
   if (pc_max_values.count(state)) {
      pc_max_values[state] = std::max(pc_max_values[state], pc);
   } else {
      pc_max_values.insert({state, pc});
   }

   auto set_var = formula->toSetVarIfApplicable();

   if (set_var) {
      std::string setter_var_name = set_var->getVarName(data_);

      if (map.count(setter_var_name)) {
         map[setter_var_name].push_back({ state, {pc, {formula, collectIfConditions(formula)}} });
      }
      else {
         map.insert({ setter_var_name, {{state, {pc, {formula, collectIfConditions(formula)}}}} });
      }
   }
   else {
      addWarning("Atomic line '" + formula->serializeWithinSurroundingFormula(100, 100) + "' is not a set var operation.");
   }
}

template<class F>
std::shared_ptr<MathStruct> FSM<F>::collectIfConditions(const std::shared_ptr<MathStruct> m)
{
   auto condition{ _id(_val(1)) };
   auto trace{ std::make_shared<std::shared_ptr<MathStruct>>(m) };

   m->applyToMeAndMyParents([condition, trace](const std::shared_ptr<MathStruct> m2) {
      if (m2->getOptor() == SYMB_IF || m2->getOptor() == SYMB_IF_ELSE) { // TODO: Use isTermIf()/isTermIfelse().
         auto cond{ m2->getOperands()[0] };

         if (*trace != m2->getOperands()[1]) {
            cond = _not(cond->copy());
         }

         std::vector<std::shared_ptr<Term>> vec{};
         vec.push_back(condition->getOperands()[0]);
         vec.push_back(cond);
         condition->getOperands()[0] = std::make_shared<TermLogicAnd>(vec);
      }

      *trace = m2;
      });

   condition = condition->getOperands()[0];

   return condition;
}

template<class F>
inline void FSM<F>::removeUnsatisfiableTransitionsAndUnreachableStates()
{
   auto states = states_plain_set_;
   std::vector<std::shared_ptr<FSMTransition>> to_remove_trans;

   for (const auto& trans : *transitions_plain_set_) {
      if (trans->condition_->isAlwaysFalse()) {
         to_remove_trans.push_back(trans);
      }
   }

   for (const auto& trans : to_remove_trans) {
      removeTransition(trans);
   }

   auto reachable = reachableStates();

   for (int state : states) {
      if (!reachable.count(state)) {
         removeState(state);
      }
   }
}

template<class F>
std::string FSM<F>::serializeCallbacksNuSMV(const MCTranslationTypeEnum translation_type, int& max_pc) const
{
   std::string current_state_var_name = StaticHelper::makeVarNameNuSMVReady(getCurrentStateVarName());
   std::map<std::string, std::vector<std::pair<int, std::pair<int, std::pair<std::shared_ptr<MathStruct>, std::shared_ptr<MathStruct>>>>>> map;
   std::map<int, int> pc_max_values;
   std::string s;

   for (const auto& id : states_plain_set_) {
      pc_max_values.insert({ id, 0 });
   }

   for (const auto& state_id : states_plain_set_) {
      if (state_id != INVALID_STATE_NUM && state_id_to_action_ids_.count(state_id)) {
         int pc = 1;

         for (const auto& callback_id : state_id_to_action_ids_.at(state_id)) {
            auto formula = action_id_to_action_.at(callback_id)->getFormula();

            if (formula) {
               auto flat_formula = _id(MathStruct::flattenFormula(formula->toTermIfApplicable()));
               flat_formula = StaticHelper::makeAssociative(flat_formula, SYMB_SEQUENCE, AssociativityTypeEnum::left);
               auto top_level_commands = extractTopLevelCommandsRecursively(flat_formula->getOperands()[0]);

               for (const auto& v : top_level_commands) {
                  addSetter(map, pc_max_values, state_id, pc, v);
                  pc++;
               }
            }
         }
      }
   }

   for (const auto& p : map) {
      s += "   next(" + p.first + ") :=\n";
      s += "   case\n";

      for (const auto& pp : p.second) {
         auto formula = pp.second.second.first->getTermsJumpIntoCompounds()[1];
         auto condition = pp.second.second.second;
         std::string formula_str{};

         if (data_->isExternalBool(p.first) && formula->isAlwaysTrue()) {
            formula_str = "TRUE";
         }
         else if (data_->isExternalBool(p.first) && formula->isAlwaysFalse()) {
            formula_str = "FALSE";
         }
         else {
            formula_str = formula->serializeNuSMV(data_, parser_);
         }

         s += "      " + current_state_var_name + " = " + StaticHelper::makeVarNameNuSMVReady(getStateVarName(pp.first, false))
            + (translation_type == MCTranslationTypeEnum::emulate_program_counter ? " & pc = " + std::to_string(pp.second.first) : "")
            + (condition->isAlwaysTrue() ? "" : " & (" + condition->serializeNuSMV(data_, parser_) + ")")
            + " : "
            + formula_str + ";\n";
      }

      s += "      TRUE : " + p.first + ";\n";
      s += "   esac;\n\n";
   }

   if (translation_type == MCTranslationTypeEnum::emulate_program_counter) {
      s += "   next(pc) := case FALSE";

      int global_max = 0;
      for (const auto& p : pc_max_values) {
         s += " | (" + current_state_var_name + " = " + StaticHelper::makeVarNameNuSMVReady(getStateVarName(p.first, false)) + " & pc >= " + std::to_string(p.second) + ")";
         global_max = std::max(global_max, p.second);
      }

      s += " : 0; TRUE : min(" + std::to_string(global_max) + ", pc + 1); esac;";
      max_pc = global_max;
   }

   return s;
}

template<class F>
inline bool FSM<F>::testEquivalencyWith(
   const TermPtr formula, 
   const mc::TypeAbstractionLayer& tal, // TODO: Why not use fsm's TAL?
   const int steps,
   const bool output_ok_too,
   const bool create_pdfs)
{
   static const std::string TEST_NAME = "TEST_FSM-Creation-From-Formula";
   static const auto LOGGER = Failable::getSingleton(TEST_NAME);
   std::set<std::string> relevant_variables;
   std::string relevant_vars_str;
   std::string broken_since_undeclared;
   auto fsm_copy = copy();

   // Collect relevant variables.
   formula->applyToMeAndMyChildren([&relevant_variables, &relevant_vars_str](const MathStructPtr m) {
      if (m->isTermVar()) {
         std::string final_var_name = StaticHelper::cleanVarNameOfPossibleRefSymbol(m->toVariableIfApplicable()->getVariableName());
         relevant_vars_str += relevant_variables.count(final_var_name) ? "" : " " + final_var_name;
         relevant_variables.insert(final_var_name);
      }
   });

   fsm_copy.resetFSMState();
   fsm_copy.step();
   auto formula_data = std::make_shared<DataPack>();
   auto fsm_data = fsm_copy.getData();

   fsm_data->addOrSetSingleVal(MC_INTERACTIVE_MODE_CONSTANT_NAME, 0);

   formula_data->initializeValuesBy(fsm_data);

   bool ok = true;
   float val_formula = 0, val_fsm = 0;
   std::string broken_var;
   int i = 0;

   for (const auto& var_type : tal.getVariablesWithTypes()) {
      fsm_data->addOrSetSingleVal(var_type.first, MathStruct::parseMathStruct(tal.getInitValueFor(var_type.first), getParser())->constEval());
      formula_data->addOrSetSingleVal(var_type.first, MathStruct::parseMathStruct(tal.getInitValueFor(var_type.first), getParser())->constEval());
   }

   for (; i < steps; i++) {
      for (const auto& var_pair : var_classifications_for_sr_) {
         if (var_pair.second != VarClassification::state && !fsm_data->isConst(var_pair.first) && relevant_variables.count(var_pair.first)) {
            constexpr int MAX = 10;
            int value = rand() % MAX - MAX / 2;

            if (output_ok_too) {
               LOGGER->addNote("Setting non-state variable '" + var_pair.first + "' to \"random\" value '" + std::to_string(value) + "'.");
            }

            formula_data->addOrSetSingleVal(var_pair.first, value);
            fsm_data->addOrSetSingleVal(var_pair.first, value);
         }
      }

      formula->eval(formula_data, fsm_copy.getParser());
      fsm_copy.step();
      if (!output_ok_too) {
         LOGGER->addNotePlain(".", "");
      }

      for (const auto& var : relevant_variables) {
         val_formula = formula_data->getSingleVal(var);
         val_fsm = fsm_data->getSingleVal(var);

         if (formula_data->isVarDeclared(var) != fsm_data->isVarDeclared(var)) {
            ok = false;
            std::string decl1 = formula_data->isVarDeclared(var) ? "declared" : "undeclared";
            std::string decl2 = fsm_data->isVarDeclared(var) ? "declared" : "undeclared";
            broken_since_undeclared = " " + decl1 + " in formula, but " + decl2 + " in FSM.";
         }
         else if (val_formula != val_fsm && (!std::isnan(val_formula) || !std::isnan(val_fsm))) {
            ok = false;
         }

         if (!ok) {
            broken_var = var;
            break;
         }
      }

      if (!ok) break;

      if (output_ok_too) {
         LOGGER->addNote("Relevant variables still equal after step " + std::to_string(i) + ".");
      }
   }

   if (!output_ok_too) {
      LOGGER->addNotePlain("");
   }

   if (ok) {
      if (output_ok_too) {
         std::string fsm_file_name = "fsm_ok.dot";
         LOGGER->addNote("Relevant variables (" + relevant_vars_str + " ) still equal after all steps.");
         LOGGER->addNote("<Please observe the stored PDF '" + fsm_file_name + "'.>");
         fsm_copy.createGraficOfCurrentGraph(fsm_file_name, true, "pdf");
      }
   }
   else {
      std::string fsm_file_name = "fsm_failed.dot";
      fsm_copy.createGraficOfCurrentGraph(fsm_file_name, true, "pdf");
      LOGGER->addNote("Test " + FAILED_COLOR + "FAILED" + RESET_COLOR + " in step " + std::to_string(i) + " with relevant variable '" + broken_var + "'"
         + (broken_since_undeclared.empty() 
            ? " = " + std::to_string(val_formula) + " (formula) != " + std::to_string(val_fsm) + " (fsm)."
            : broken_since_undeclared)
      );
      LOGGER->addNote("This is the formula: '" + formula->serialize() + "'.");
      LOGGER->addNote("This is the FSM:");
      LOGGER->addNote("<Please observe the stored PDF '" + fsm_file_name + "'.>");
   }

   if (!ok || output_ok_too) {
      LOGGER->addNote("This is the data (formula):");
      LOGGER->addNotePlain(formula_data->toStringHeap());
      LOGGER->addNote("This is the data (FSM):");
      LOGGER->addNotePlain(fsm_data->toStringHeap());
   }

   return ok;
}

template<class F>
inline bool FSM<F>::testEquivalencyWith(FSM<F>& other, const int steps, const bool create_pdfs)
{
   FSM<F> vd = randomFSM(15, 30, 4, 4, false, data_->getAllSingleValVarNames(false));
   return testEquivalencyWith(other, steps, create_pdfs, vd);
}

template<class F>
inline bool FSM<F>::testEquivalencyWith(
   FSM<F>& other,
   const int steps, 
   const bool store_pdfs,
   FSM<F>& vd_other)
{
   auto vd_this = vd_other.copy();
   vd_other.registerDataPack(other.data_);
   vd_this.registerDataPack(this->data_);

   this->resetFSMState();
   other.resetFSMState();

   for (int i = 0; i < steps; i++) {
      if (store_pdfs) {
         this->createGraficOfCurrentGraph("this" + std::to_string(i), true, "pdf");
         other.createGraficOfCurrentGraph("other" + std::to_string(i), true, "pdf");
         vd_this.createGraficOfCurrentGraph("vd_this" + std::to_string(i), true, "pdf");
         vd_other.createGraficOfCurrentGraph("vd_other" + std::to_string(i), true, "pdf");
      }

      std::string current_state_var_name = getCurrentStateVarName();
      std::string step_counter_var_name = getStepCountVarName();

      if (!this->data_->hasEqualData(*other.data_, [current_state_var_name, step_counter_var_name](const std::string& name, const DataPack& data_this, const DataPack& data_other) {
               return name != current_state_var_name
                  && name != "last" // TODO: Avoid magic string.
                  && name != step_counter_var_name
                  && !data_this.isHidden(name) 
                  && !data_other.isHidden(name);
            })) {
         addNote("Other FSM is not equal to this. DataPacks differed after " + std::to_string(i) + " steps.");
         addNote("This is the variable disturber:");
         addNote(vd_this.getDOTCodeOfCurrentGraph());

         return false;
      }

      vd_other.step();
      vd_this.step();
      other.step();
      this->step();
   }

   addNote("FSMs passed the equivalency test up to " + std::to_string(steps) + " simulation steps.");
   return true;
}

template<class F>
inline void FSM<F>::registerDataPack(const std::shared_ptr<DataPack> data)
{
   auto cs = getCurrentState(); // Use old data pack to retrieve current state...

   if (data) {
      registration_number_ = data->getArraySize(getCurrentStateVarName());
      data_ = data;
   }
   else {
      registration_number_ = 0;
      data_ = std::make_shared<DataPack>();

      data_->addOrSetSingleVal("true", 1, false, true);
      data_->addOrSetSingleVal("false", 0, false, true);
   }

   addFailableChild(data_, "");
   setCurrentState(cs); // ...and set it in new data pack.
}

template<class F>
inline void FSM<F>::registerResolver(const std::shared_ptr<FSMResolver> resolver)
{
   if (!resolver_) {
      resolver_ = std::make_shared<FSMResolverDefault>();
   }

   addFailableChild(resolver_, "");

   resolver_->setFSMData(getInitialStateNum(),
      getCurrentStateVarName(),
      registration_number_,
      transitions_plain_set_,
      outgoing_transitions_,
      incoming_transitions_,
      data_,
      parser_,
      getJitLevel());
}

template<class F>
inline FSM<F> FSM<F>::copy(const bool use_same_data, const bool use_same_parser) const
{
   return *copyToPtr(use_same_data, use_same_parser);
}

template<class F>
inline std::shared_ptr<FSM<F>> FSM<F>::copyToPtr(const bool use_same_data, const bool use_same_parser) const
{
   auto m = std::make_shared<FSM<F>>(
      FSMResolverFactory::createNewResolverFromExisting(resolver_), 
      non_determinism_option_,
      use_same_data ? data_ : nullptr,     // Either same as or copy of old data.
      use_same_parser ? parser_ : nullptr, // Either same as or copy of old parser.
      jit_level_);

   if (!use_same_data) { // Create copy of old data.
      m->data_->initializeValuesBy(data_);
   }

   if (!use_same_parser) { // Create copy of old parser.
      m->parser_->initializeValuesBy(parser_);
   }

   m->action_id_to_action_.clear();
   for (const auto& e : action_id_to_action_) {
      if (e.second->getFormula()) {
         auto functor = std::make_shared<TInternalFunctor<F>>(e.second->getFormula()->copy(), e.second->getName());
         m->action_id_to_action_.insert({ e.first, functor });
      }
      else {
         m->action_id_to_action_.insert({ e.first, e.second });
      }
   }

   m->action_name_to_action_id_ = action_name_to_action_id_;
   m->all_constants_to_model_checker_ = all_constants_to_model_checker_;
   m->all_external_variables_ = all_external_variables_;
   m->all_fsm_controlled_variables_ = all_fsm_controlled_variables_;
   m->cb_clusters_ = cb_clusters_;
   m->closed_form_ = closed_form_ ? closed_form_->copy() : nullptr;
   m->encapsulation_counter_ = encapsulation_counter_;
   m->fan_out_cb_counter_ = fan_out_cb_counter_;
   m->fan_out_cb_dummy_state_counter_ = fan_out_cb_dummy_state_counter_;
   m->graphviz_state_fadeout_value_ = graphviz_state_fadeout_value_;

   if (type_abstraction_layer_) {
      m->type_abstraction_layer_ = type_abstraction_layer_->copy();
   }

   m->incoming_transitions_->clear();
   for (const auto& e : *incoming_transitions_) {
      std::vector<std::shared_ptr<FSMTransition>> vec;
      for (const auto& f : e.second) {
         vec.push_back(std::make_shared<FSMTransition>(f->state_source_, f->state_destination_, f->condition_->copy(), m->data_));
      }

      m->incoming_transitions_->insert({ e.first, vec });
   }

   m->outgoing_transitions_->clear();
   for (const auto& e : *outgoing_transitions_) {
      std::vector<std::shared_ptr<FSMTransition>> vec;
      for (const auto& f : e.second) {
         vec.push_back(std::make_shared<FSMTransition>(f->state_source_, f->state_destination_, f->condition_->copy(), m->data_));
      }

      m->outgoing_transitions_->insert({ e.first, vec });
   }

   m->transitions_plain_set_->clear();
   for (const auto& f : *transitions_plain_set_) {
      m->transitions_plain_set_->push_back(std::make_shared<FSMTransition>(f->state_source_, f->state_destination_, f->condition_->copy(), m->data_));
   }

   m->last_loaded_program = last_loaded_program;
   m->prohibit_double_edges_ = prohibit_double_edges_;
   m->registration_number_ = registration_number_;
   // m->resolver_factory_ = resolver_factory_;                                        // Doesn't copy generated resolver, but sets to nullptr.
   if (m->type_abstraction_layer_) {
      m->type_abstraction_layer_ = std::make_shared<mc::TypeAbstractionLayer>(*type_abstraction_layer_);
      m->addFailableChild(m->type_abstraction_layer_);
   }
   m->states_plain_set_ = states_plain_set_;
   m->state_id_to_action_ids_ = state_id_to_action_ids_;
   m->state_id_to_state_name_ = state_id_to_state_name_;
   m->state_images_ = state_images_;                                                   // Doesn't copy images, only pointers.
   m->state_name_to_state_id_ = state_name_to_state_id_;
   m->steps_at_last_visit_ = steps_at_last_visit_;
   m->step_counter_ = step_counter_;
   m->unfolded_callback_counters_ = unfolded_callback_counters_;
   m->unfolded_callback_interfaces_ = unfolded_callback_interfaces_;                   // Doesn't copy formulae of incoming conditions, only pointers.
   m->unfolded_callback_interfaces_formulae_ = unfolded_callback_interfaces_formulae_; // Doesn't copy formulae of incoming conditions, only pointers.

   m->resolver_->setFSMData(m->getInitialStateNum(),
                           m->getCurrentStateVarName(),
                           m->registration_number_,
                           m->transitions_plain_set_,
                           m->outgoing_transitions_,
                           m->incoming_transitions_,
                           m->data_,
                           m->parser_,
                           m->getJitLevel());
   m->painter_ = painter_;
   m->painter_image_->insertImage(0, 0, *painter_image_, false);

   return m;
}

template<class F>
inline bool FSM<F>::mutate()
{
   //std::random_device rd;
   //std::mt19937 gen(rd());
   //std::uniform_int_distribution<> distr(0, 48);

   //                        1  2  3  4  5  6  7  8   9   10  11
   constexpr int probs[] = { 0, 0, 0, 0, 0, 0, 300, 600, 601, 602, 603 };
   //constexpr int probs[] = { 3, 7, 10, 16, 24, 25, 28, 35, 45, 48, 49 };
   const int rand_num = std::rand() % probs[10]; // distr(gen); (For now, the classic way, more sophisticated randomness in future.)

   if (rand_num < probs[0])       return mutateM1InsertState();   
   else if (rand_num < probs[1])  return mutateM2RemoveStateIfUnreachable();
   else if (rand_num < probs[2])  return mutateM3RemoveIdleStateIfDeadEnd();
   else if (rand_num < probs[3])  return mutateM4InsertFalseTransition();
   else if (rand_num < probs[4])  return mutateM5RemoveFalseTransition();
   else if (rand_num < probs[5])  return mutateM6ChangeStateCommand();
   else if (rand_num < probs[6])  return mutateM7SimplifyCondition();
   else if (rand_num < probs[7])  return mutateM8ComplexifyCondition();
   else if (rand_num < probs[8])  return mutateM9ChangeOperatorInCondition();
   else if (rand_num < probs[9])  return mutateM10ChangeConstantInCondition();
   else if (rand_num < probs[10]) return mutateM11ChangeSensorInCondition();
   return false;
}

template<class F>
inline bool FSM<F>::mutateM1InsertState()
{
   const int state_id = smallestFreeStateID();
   const int action_id = rand() % action_id_to_action_.size();
   addUnconnectedStateIfNotExisting(state_id);
   associateStateIdToActionId(state_id, action_id);
   return true;
}

template<class F>
inline bool FSM<F>::mutateM2RemoveStateIfUnreachable()
{
   const int rand_state = getRandomStateIDExceptInvalidState();

   if (!isReachable(rand_state)) {
      removeState(rand_state);
      return true;
   }

   return false;
}

template<class F>
inline bool FSM<F>::mutateM3RemoveIdleStateIfDeadEnd()
{
   const int rand_state = getRandomStateIDExceptInvalidState();
   const auto& actions = state_id_to_action_ids_[rand_state];

   if (rand_state != INITIAL_STATE_NUM && actions.size() == 1 && action_name_to_action_id_.at("IDLE") == actions.at(0)) {
      removeState(rand_state);
      return true;
   }

   return false;
}

template<class F>
inline bool FSM<F>::mutateM4InsertFalseTransition()
{
   const int rand_state1 = getRandomStateIDExceptInvalidState();
   const int rand_state2 = getRandomStateIDExceptInvalidState();
   addTransition(rand_state1, rand_state2, _val(0));
   return true;
}

template<class F>
inline bool FSM<F>::mutateM5RemoveFalseTransition()
{
   const auto rand_trans = getRandomTransitionExceptInvalidState();

   if (rand_trans && rand_trans->condition_->isAlwaysFalse()) {
      removeTransition(rand_trans);
      return true;
   }

   return false;
}

template<class F>
inline bool FSM<F>::mutateM6ChangeStateCommand()
{
   const int rand_state = getRandomStateIDExceptInvalidState();
   const int action_id = rand() % action_id_to_action_.size();
   removeActionsFromState(rand_state);
   associateStateIdToActionId(rand_state, action_id);
   return true;
}

template<class F>
inline bool FSM<F>::mutateM7SimplifyCondition()
{
   const auto trigger_break = std::make_shared<bool>(false);

   return helperForRandomConditionChanges([trigger_break](const std::shared_ptr<MathStruct> m) {
         if (m->getOptor() == SYMB_AND) {
            if (m->getOperands()[0]->isAlwaysFalse() || m->getOperands()[1]->isAlwaysFalse()) { // (c AND false) -> false,
               m->getFather()->replaceOperand(m->toTermIfApplicable(), _false());
               *trigger_break = true;
            }
            else if (m->getOperands()[0]->isAlwaysTrue()) { // (true AND c) -> c
               m->getFather()->replaceOperand(m->toTermIfApplicable(), m->getOperands()[1]);
               *trigger_break = true;
            }
            else if (m->getOperands()[1]->isAlwaysTrue()) { // (c AND true) -> c
               m->getFather()->replaceOperand(m->toTermIfApplicable(), m->getOperands()[0]);
               *trigger_break = true;
            }
         }
         else if (m->getOptor() == SYMB_OR) {
            if (m->getOperands()[0]->isAlwaysFalse()) { // (false OR c) -> c
               m->getFather()->replaceOperand(m->toTermIfApplicable(), m->getOperands()[1]);
               *trigger_break = true;
            }
            else if (m->getOperands()[1]->isAlwaysFalse()) { // (c OR false) -> c
               m->getFather()->replaceOperand(m->toTermIfApplicable(), m->getOperands()[0]);
               *trigger_break = true;
            }
            else if (m->getOperands()[0]->isAlwaysTrue() || m->getOperands()[1]->isAlwaysTrue()) { // (c OR true) -> true
               m->getFather()->replaceOperand(m->toTermIfApplicable(), _true());
               *trigger_break = true;
            }
         }
      }, trigger_break);
}

template<class F>
inline bool FSM<F>::mutateM8ComplexifyCondition()
{
   const auto rand_trans = getRandomTransitionExceptInvalidState();

   if (rand_trans->condition_->getNodeCount() > 7) {
      return false;
   }

   if (rand_trans) {
      rand_trans->condition_ = std::rand() % 2 
         ? _and(_true(), rand_trans->condition_)
         : _or(_false(), rand_trans->condition_);

#if defined(ASMJIT_ENABLED)
      if (USE_ASMJIT_FOR_MUTATION) rand_trans->condition_->createAssembly(data_, parser_, true);
#endif

      return true;
   }

   return false;
}

template<class F>
inline bool FSM<F>::mutateM9ChangeOperatorInCondition()
{
   const auto rand_trans = getRandomTransitionExceptInvalidState();

   if (rand_trans) {
      const auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
      const auto rand_cond = _id(rand_trans->condition_->copy());
      auto sub_formula = rand_cond->getUniformlyRandomSubFormula(time.count(), true, true);
      auto sub_formula_father = sub_formula->getFather();

      while (sub_formula_father->getOptor() != SYMB_AND
         && sub_formula_father->getOptor() != SYMB_OR
         && sub_formula_father->getOptor() != SYMB_ID) {
         sub_formula = sub_formula_father;
         sub_formula_father = sub_formula_father->getFather();
      }

      const bool towards_true = std::rand() % 2;
      const bool towards_false = !towards_true;
      const bool upper_branch = std::rand() % 2;
      const bool lower_branch = !upper_branch;
      const std::string optor = sub_formula->getOptor();
      auto opnd0 = sub_formula->getOperands().empty() ? nullptr : sub_formula->getOperands()[0];
      auto opnd1 = sub_formula->getOperands().empty() ? nullptr : sub_formula->getOperands()[1];

      std::shared_ptr<Term> new_formula;

      if (towards_false && optor == SYMB_EQ) {
         // FALSE
         new_formula = _false();
      }
      else if (towards_true && optor == SYMB_NEQ) {
         // TRUE
         new_formula = _true();
      }
      else if (towards_true && optor == SYMB_EQ || towards_false && (optor == SYMB_SMEQ || optor == SYMB_GREQ)) {
         // APPROX
         new_formula = _approx(opnd0, opnd1);
      }
      else if (towards_false && optor == SYMB_NEQ || towards_true && (optor == SYMB_SM || optor == SYMB_GR)) {
         // NAPPROX
         new_formula = _napprox(opnd0, opnd1);
      }
      else if (towards_true && upper_branch && optor == SYMB_APPROX || towards_false && optor == SYMB_SM) {
         // SMEQ
         new_formula = _smeq(opnd0, opnd1);
      }
      else if (towards_true && lower_branch && optor == SYMB_APPROX || towards_false && optor == SYMB_GR) {
         // GREQ
         new_formula = _greq(opnd0, opnd1);
      }
      else if (towards_false && upper_branch && optor == SYMB_NAPPROX || towards_true && optor == SYMB_SMEQ) {
         // SM
         new_formula = _sm(opnd0, opnd1);
      }
      else if (towards_false && lower_branch && optor == SYMB_NAPPROX || towards_true && optor == SYMB_GREQ) {
         // GR
         new_formula = _gr(opnd0, opnd1);
      } else { // Has to be EQ or NEQ now.
         if (!opnd0) {
            const auto& sensors = getEvolutionSensors();
            const float some_val = sensors.at(std::rand() % sensors.size())->eval(data_, parser_);
            const bool sensor0 = std::rand() % 2;
            const bool sensor1 = !sensor0 || std::rand() % 2;

            opnd0 = sensor0
               ? sensors.at(std::rand() % sensors.size())
               : _val(some_val);
               //: _val(MIN_FLOAT + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (MAX_FLOAT - MIN_FLOAT))));

            opnd1 = sensor1
               ? sensors.at(std::rand() % sensors.size())
               : _val(some_val);
               //: _val(MIN_FLOAT + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (MAX_FLOAT - MIN_FLOAT))));
         }

         if (towards_false && optor == SYMB_APPROX || sub_formula->isAlwaysFalse()) {
            // EQ         
            new_formula = _eq(opnd0, opnd1);
         }
         else /* (towards_true && optor == SYMB_NAPPROX || sub_formula->isAlwaysTrue()) */ {
            // NEQ
            new_formula = _neq(opnd0, opnd1);
         }
      }

      sub_formula_father->replaceOperand(sub_formula->toTermIfApplicable(), new_formula);
      sub_formula_father->setChildrensFathers(true, false);
      rand_trans->condition_ = rand_cond->getOperands()[0];

#if defined(ASMJIT_ENABLED)
      if (USE_ASMJIT_FOR_MUTATION) rand_trans->condition_->createAssembly(data_, parser_, true);
#endif
      return true;
   }

   return false;
}

template<class F>
inline bool FSM<F>::mutateM10ChangeConstantInCondition()
{
   const auto rand_trans = getRandomTransitionExceptInvalidState();

   if (rand_trans) {
      const auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
      const auto rand_cond = _id(rand_trans->condition_->copy());
      auto sub_formula = rand_cond->getUniformlyRandomSubFormula(
         time.count(), 
         true, 
         true, 
         [](const std::shared_ptr<MathStruct> m) {
            const auto father = m->getFather();

            return father && father->getOptor() != SYMB_AND && father->getOptor() != SYMB_OR && !father->isRootTerm() && m->isOverallConstant(); 
      });
      
      if (sub_formula) {
         const float rand_val = -STEP_FLOAT + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * STEP_FLOAT)));
         const float new_val = std::min(std::max(sub_formula->constEval() + rand_val, MIN_FLOAT), MAX_FLOAT);

         sub_formula->getFather()->replaceOperand(sub_formula->toTermIfApplicable(), _val(new_val));
         rand_trans->condition_ = rand_cond->getOperands()[0];

#if defined(ASMJIT_ENABLED)
         if (USE_ASMJIT_FOR_MUTATION) rand_trans->condition_->createAssembly(data_, parser_, true);
#endif

         return true;
      }
   }

   return false;
}

template<class F>
inline bool FSM<F>::mutateM11ChangeSensorInCondition()
{
   const auto rand_trans = getRandomTransitionExceptInvalidState();

   if (rand_trans) {
      const auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
      const auto rand_cond = _id(rand_trans->condition_->copy());
      auto sub_formula = rand_cond->getUniformlyRandomSubFormula(
         time.count(), 
         true, 
         true,
         [](const std::shared_ptr<MathStruct> m) {
         const auto father = m->getFather();

         return (m->getOperands().empty() && m->getFather()->getOptor() != SYMB_GET_ARR || m->getOptor() == SYMB_GET_ARR) && !m->isOverallConstant(); // Has father due to construction of other mutations.
      });

      if (sub_formula) {
         const auto& sensors = getEvolutionSensors();
         const auto new_sens = sensors.at(std::rand() % sensors.size());

         sub_formula->getFather()->replaceOperand(sub_formula->toTermIfApplicable(), new_sens);
         rand_trans->condition_ = rand_cond->getOperands()[0];

#if defined(ASMJIT_ENABLED)
         if (USE_ASMJIT_FOR_MUTATION) rand_trans->condition_->createAssembly(data_, parser_, true);
#endif

         return true;
      }
   }

   return false;
}

template<class F>
inline bool FSM<F>::helperForRandomConditionChanges(const std::function<void(const std::shared_ptr<MathStruct>)>& func, const std::shared_ptr<bool> trigger_break) const
{
   const auto rand_trans = getRandomTransitionExceptInvalidState();

   if (rand_trans) {
      const auto rand_cond = _id(rand_trans->condition_->copy());
      rand_cond->applyToMeAndMyChildren(func, false, nullptr, trigger_break);

      if (*trigger_break) {
         rand_trans->condition_ = rand_cond->getOperands()[0];

#if defined(ASMJIT_ENABLED)
         if (USE_ASMJIT_FOR_MUTATION) rand_trans->condition_->createAssembly(data_, parser_, true);
#endif
         
         return true;
      }
   }

   return false;
}

template<class F>
inline std::vector<std::shared_ptr<Term>> FSM<F>::getEvolutionSensors()
{
   std::vector<std::shared_ptr<Term>> vec;

   //vec.push_back(_Get(_var("agents_vy"), _var("ego_lead_car")));
   //vec.push_back(_Get(_var("agents_vx_rel"), _var("ego_lead_car")));
   //vec.push_back(_Get(_var("agents_pos_y"), _var("ego_lead_car")));
   vec.push_back(_Get(_var("agents_pos_x"), _var("ego_lead_car")));
   //vec.push_back(_var("ego_vx"));
   //vec.push_back(_var("ego_pos_y"));
   //vec.push_back(_var("ego_lead_car"));
   //vec.push_back(_var("ego_ax"));
   //vec.push_back(_var("ego_vy"));

   return vec;
}

template<class F>
inline bool FSM<F>::isEvolutionSensor(const std::shared_ptr<Term> m)
{
   return m->isTermVar() || true;
}

template<class F>
inline void FSM<F>::takeOverTopologyFrom(const std::shared_ptr<FSM<F>> other)
{
   if (!other || other.get() == this) {
      return;
   }

   resetFSMTopology();

   for (const auto& trans : *other->transitions_plain_set_) {
      auto new_cond = trans->condition_->copy();

#if defined(ASMJIT_ENABLED)
      if (trans->condition_->isAssemblyCreated()) {
         new_cond->createAssembly(data_, parser_);
      }
#endif

      auto new_trans = std::make_shared<FSMTransition>(trans->state_source_, trans->state_destination_, new_cond, this->data_);
      addTransition(new_trans);
   }

   for (const auto& state : other->states_plain_set_) {
      for (const auto& action : other->state_id_to_action_ids_[state]) {
         associateStateIdToActionId(state, action);
      }
   }
}

template<class F>
inline void FSM<F>::encapsulateIfConditions(TermPtr& formula, const std::string& function_name_base)
{
   int counter = 0;

   formula->applyToMeAndMyChildren([&counter, this, &function_name_base](const MathStructPtr m) {
      if (m->isTermIf() || m->isTermIfelse()) {
         auto& condition = m->getOperands()[0];

         if (condition->getNodeCount() > 1) {
            std::string op_name = "__" + function_name_base + std::to_string(counter) + "__";
            defineNewMathOperator(op_name, condition);
            condition = parser_->termFactory(op_name);
            counter++;
         }
      }
   });
}

template<class F>
inline MCTrace FSM<F>::createTraceFromSimulation(const int steps, const TermPtr variable_controller)
{
   MCTrace trace;
   vfm::VarVals varvals_last;

   for (size_t i = 1; i < steps + 1; i++) {
      vfm::VarVals varvals;

      for (const auto& var : data_->getAllSingleValVarNames()) {
         if (!data_->isConst(var) && !data_->isHidden(var) && !StaticHelper::isPrivateVar(var)) {
            std::string val = MathStruct::parseMathStruct(var, parser_, data_)->serializeNuSMV(data_, parser_);
            bool is_already_there_and_same = false;

            for (auto& varval_last : varvals_last) {
               if (varval_last.first == var) {
                  if (varval_last.second == val) {
                     is_already_there_and_same = true;
                     break;
                  }
                  else {
                     varval_last.second = val;
                  }
               }
            }

            if (!is_already_there_and_same) {
               varvals.insert({ var, val });
               varvals_last.insert({ var, val });
            }
         }
      }

      trace.addTraceStep({ std::to_string(i), varvals });

      step();
      variable_controller->eval(data_, parser_);
   }

   return trace;
}

template<class F>
inline int FSM<F>::getStepCount() const
{
   return (int) step_counter_;
}

template<class F>
inline std::string FSM<F>::serializeToProgram() const
{
   std::string str;

   str += std::string(1, PROGRAM_PREPROCESSOR_DENOTER) + FSMResolverFactory::RESOLVER_DENOTER + PROGRAM_ARGUMENT_ASSIGNMENT + resolver_->getName() + PROGRAM_COMMAND_SEPARATOR + "\n";
   str += (additional_state_var_name_.empty() ? "" : PREP_ADDITIONAL_STATE_VARIABLE_NAME + "=" + additional_state_var_name_) + PROGRAM_COMMAND_SEPARATOR + "\n\n";
   str += data_->serializeEnumsFull() + "\n";
   str += parser_->serializeDynamicTerms();
   str += data_->serializeVariableValues() + "\n";

   str += FSM_PROGRAM_TRANSITION_DENOTER + state_id_to_state_name_.at(getInitialStateNum()) + PROGRAM_COMMAND_SEPARATOR + "\n";

   for (const auto& trans : *transitions_plain_set_) {
      str += trans->toString(state_id_to_state_name_) + PROGRAM_COMMAND_SEPARATOR + "\n";
   }

   str += "\n";

   for (int state_id : states_plain_set_) {
      std::string callbacks;

      for (const auto& callback_id : state_id_to_action_ids_.at(state_id)) {
         auto formula = action_id_to_action_.at(callback_id)->getFormula();
         callbacks += PROGRAM_OPENING_FORMULA_BRACKET + formula->serialize(nullptr, MathStruct::SerializationStyle::cpp, MathStruct::SerializationSpecial::none, 0, 0) + PROGRAM_CLOSING_FORMULA_BRACKET + FSM_PROGRAM_COMMAND_SEPARATOR + " ";
      }

      str += state_id_to_state_name_.at(state_id) + " " + FSM_PROGRAM_COMMAND_DENOTER + " " 
         + callbacks
         + PROGRAM_COMMAND_SEPARATOR + "\n";
   }

   return str;
}

template<class F>
inline int FSM<F>::getRandomStateIDExceptInvalidState() const
{
   const int state_count = getStateCount();
   
   if (state_count > 1) {
      const int state_num = std::rand() % state_count;
      std::set<int>::const_iterator it(states_plain_set_.begin());
      advance(it, state_num);
      const int state_id = *it;
      return state_id == INVALID_STATE_NUM ? getRandomStateIDExceptInvalidState() : state_id; // TODO: This is of course a bit shitty... But for now whatever works...
   }
   else {
      return INVALID_STATE_NUM;
   }
}

template<class F>
inline std::shared_ptr<FSMTransition> FSM<F>::getRandomTransitionExceptInvalidState() const
{
   if (transitions_plain_set_->size() >= 2) {
      const int trans_num = std::rand() % (transitions_plain_set_->size() - 1);
      return transitions_plain_set_->at(trans_num + 1); // Zeroth entry is "invalid" transition.
   }

   return nullptr;
}

///////////////// XWizard/Graphviz output ////////////////

constexpr float NUM_TRIALS_FOR_ESTIMATING_TRANSITION_STRENGTH = 100;
constexpr float MAX_EDGE_THICKNESS = 1;
constexpr int MAX_SENSOR_VALUE_FOR_RANDOMIZING = 255;
const std::string NORMAL_TRANSITION_COLOR = "black";
const std::string ALWAYS_FALSE_TRANSITION_COLOR = "red";
const std::string ALWAYS_TRUE_TRANSITION_COLOR = "darkgreen";
const std::string REDUNDANT_TRANSITION_COLOR = "snow4";
const std::string UNREACHABLE_STATE_COLOR = REDUNDANT_TRANSITION_COLOR;
const std::string DATA_NODE_COLOR = "blue";
const std::string DATA_NODE_BACKGROUND_COLOR = "white";
const std::string FUNCTION_NODE_COLOR = "blue";
const std::string FUNCTION_NODE_BACKGROUND_COLOR = "lightgrey";
const std::string RECENT_STATE_BACKGROUND_BASE_COLOR = "#ffff55";
const std::string CURRENT_STATE_BACKGROUND_COLOR = "orange";
const std::string CALLBACK_SUBGRAPH_BORDER_COLOR = "lightgrey";
constexpr bool USE_HTML_NODES = true;

template<class F>
std::string FSM<F>::getDOTCodeOfCurrentGraph(
   const bool mark_current_state, 
   const bool mc_mode, 
   const bool point_mark_dead_ends, 
   const GraphvizOutputSelector what) const
{
   auto reachable_states = reachableStates();
   std::string result = std::string("digraph {\n") + (mc_mode ? "rankdir=\"LR\";\n" : "");
    
    if (what == GraphvizOutputSelector::all || what == GraphvizOutputSelector::graph_only) {
      for (const auto& state : states_plain_set_) {
         if (mc_mode && state == INVALID_STATE_NUM) {
            continue;
         }

         std::string assoc_actions;
         for (const auto& act : state_id_to_action_ids_.at(state)) {
            std::string action_name = action_id_to_action_.at(act)->getName();
            assoc_actions.append(StaticHelper::shortenToMaxSize(action_name, 52) + " ");
         }

         StaticHelper::trim(assoc_actions);
         assoc_actions = StaticHelper::replaceAll(assoc_actions, " ", ",");

         std::string color = CURRENT_STATE_BACKGROUND_COLOR;
         if (getCurrentState() != state) {
            int opacity = static_cast<int>(255.0 - (step_counter_ - steps_at_last_visit_.at(state)) * 255.0 / graphviz_state_fadeout_value_);
            opacity = std::max(std::min(opacity, 255), 0);
            std::stringstream stream;
            stream << std::hex << opacity;
            std::string opacity_str(stream.str());
            color = RECENT_STATE_BACKGROUND_BASE_COLOR + opacity_str;
         }

         std::string addons = reachable_states.count(state) ? "" : "color=" + UNREACHABLE_STATE_COLOR + ",fontcolor=" + UNREACHABLE_STATE_COLOR + ",";
         addons.append(state == INVALID_STATE_NUM ? "shape=diamond," : "shape=record,");
         addons.append(mark_current_state ? "style=filled,fillcolor=\"" + color + "\"," : "");
         std::string state_name = StaticHelper::replaceAll(StaticHelper::replaceAll(StaticHelper::replaceAll(state_id_to_state_name_.at(state), "::::", "\\n"), ":::", " = "), "..:..", "-");
         std::string label = assoc_actions.empty() 
               ? "label=\"" + state_name + "\"]"
            : "label=\"" + state_name + "\\n&lt;" + assoc_actions + "&gt;\"]";

         label = StaticHelper::replaceAll(label, "<", "&lt;");
         label = StaticHelper::replaceAll(label, ">", "&gt;");
         label = StaticHelper::replaceAll(label, "|", "&#124;");

         auto it = state_images_.find(state);
         if (it != state_images_.end()) {
            std::string img_name = "state" + std::to_string(state) + "_image.png";
            it->second->store(img_name, OutputType::png);
            
            if (point_mark_dead_ends && (!outgoing_transitions_->count(state) || outgoing_transitions_->at(state).empty())) {
               addons = "shape=point,";
            } else {
               addons = "image=\"" + img_name + "\",shape=none,";
            }

            label = "label=\"\"]";
         }

         result.append(stateNameGV(state))
               .append("[")
               .append(addons)
               .append(label)
               .append(";\n");
      }

      for (const auto& cb : cb_clusters_) {
         result.append("subgraph cluster_").append(cb.first).append("{\n").append("color=" + CALLBACK_SUBGRAPH_BORDER_COLOR + ";\n").append("penwidth=8;\n").append("label=\"").append(cb.first).append("\";\n");

         for (const auto& id : cb.second) {
            result.append(stateNameGV(id)).append(";\n");
         }

         result.append("}\n");
      }
   }

    if (!mc_mode // Data, heap and functions nodes only if not in MC mode.
       && (what == GraphvizOutputSelector::all 
       || what == GraphvizOutputSelector::data_only 
       || what == GraphvizOutputSelector::data_with_functions 
       || what == GraphvizOutputSelector::functions_only)) {

       if (what == GraphvizOutputSelector::all || what == GraphvizOutputSelector::data_with_functions || what == GraphvizOutputSelector::data_only) {
          std::string heap_string("HEAP\n\n" + data_->toStringHeap());

          if (USE_HTML_NODES) {
             heap_string = StaticHelper::replaceAll(heap_string, ">", "&gt;");
             heap_string = StaticHelper::replaceAll(heap_string, "<", "&lt;");
             heap_string = StaticHelper::replaceAll(heap_string, "\t\tlabel", "\tlabel");
             heap_string = StaticHelper::replaceAll(heap_string, "\t", "</TD><TD align=\"left\">");
             heap_string = "<TABLE border=\"0\">\n<TR><TD align=\"left\">"
                + StaticHelper::replaceAll(heap_string, "\n", "</TD></TR>\n<TR><TD align=\"left\">")
                + "</TD></TR></TABLE>";
             heap_string = "<" + heap_string + ">";
          }
          else {
             // TODO
          }

          std::string heap_addons = "color=" + DATA_NODE_COLOR + ",fontcolor=" + DATA_NODE_COLOR + ",";
          heap_addons.append("shape=record,");
          heap_addons.append("style=filled,fillcolor=" + DATA_NODE_BACKGROUND_COLOR + ",");
          std::string heap_label = "label=" + heap_string + "]";
          result.append("HEAP_NODE ")
             .append("[")
             .append(heap_addons)
             .append(heap_label)
             .append(";\n");

          std::string data_string("DATA\n\n" + data_->toString()
             + std::string("\nnondet strategy=*") + resolver_->getNdetMechanismName() + "*"
             + "\ndeadlock strategy=*" + resolver_->getDeadlockMechanismName() + "*");
          if (USE_HTML_NODES) {
             data_string = StaticHelper::replaceAll(data_string, "=", "</TD><TD align=\"left\">");
             data_string = StaticHelper::replaceAll(data_string, " -- ", "</TD><TD align=\"left\">");
             data_string = "<TABLE border=\"0\">\n<TR><TD align=\"left\">"
                + StaticHelper::replaceAll(data_string, "\n", "</TD></TR>\n<TR><TD align=\"left\">")
                + "</TD></TR></TABLE>";
             data_string = "<" + data_string + ">";
          }
          else {
             data_string = StaticHelper::replaceAll(data_string, "\"", "'");
             data_string = StaticHelper::replaceAll(data_string, "[", "\\[");
             data_string = StaticHelper::replaceAll(data_string, "]", "\\]");
             data_string = StaticHelper::replaceAll(data_string, "\n", "\\l\n");
             data_string = "\"" + data_string + "\"";
          }

          std::string addons = "color=" + DATA_NODE_COLOR + ",fontcolor=" + DATA_NODE_COLOR + ",";
          addons.append("shape=record,");
          addons.append("style=filled,fillcolor=" + DATA_NODE_BACKGROUND_COLOR + ",");
          std::string label = "label=" + data_string + "]";
          result.append("DATA_NODE ")
             .append("[")
             .append(addons)
             .append(label)
             .append(";\n");
       }

       if (what == GraphvizOutputSelector::all || what == GraphvizOutputSelector::data_with_functions || what == GraphvizOutputSelector::functions_only) {
          std::string funcs;
          for (const auto& ls : parser_->getDynamicTermMetas()) {
             for (const auto& l : ls.second) {
                if (l.second && !parser_->isTermNative(ls.first, l.first)) {
                   std::string func_string = l.second->serialize(nullptr, MathStruct::SerializationStyle::cpp, MathStruct::SerializationSpecial::none, 0, 3, 150);

                   std::string dots = "";
                   for (int i = 0; i < l.first; i++) {
                      dots.append(".");
                   }

                   std::string op_name = ls.first;

                   funcs += "\\l\n" + op_name + "(" + dots + ") " + PROGRAM_DEFINITION_SEPARATOR + " \\l\n\\ \\ \\ \\ \\ \\ "
                      + StaticHelper::replaceAll(
                         StaticHelper::replaceAll(
                            StaticHelper::replaceAll(
                               StaticHelper::replaceAll(
                                  StaticHelper::replaceAll(
                                     StaticHelper::replaceAll(
                                        func_string, "\n", "\\l\n\\ \\ \\ \\ \\ \\ "),
                                     ">", "&gt;"),
                                  "<", "&lt;"),
                               "|", "\\|"),
                            "}", "\\}"),
                         "{", "\\{")
                      + std::string("\\l\n");
                }
             }
          }

          result.append("FUNC_NODE [color=" + FUNCTION_NODE_COLOR + ",fontcolor=" + FUNCTION_NODE_COLOR + ", shape=record, style=filled, fillcolor=" + FUNCTION_NODE_BACKGROUND_COLOR + ", \
            label=\" \
            FUNCTIONS\\l\\l\n"
             + funcs + "\"];\n");
       }

       if (what == GraphvizOutputSelector::all || what == GraphvizOutputSelector::data_with_functions) {
          result.append("DATA_NODE -> FUNC_NODE;\n");
       }
   }

    if (what == GraphvizOutputSelector::all || what == GraphvizOutputSelector::graph_only) {
       auto destination_states_from_current = resolver_->retrieveNextStates(false);

       for (const auto& pair : *outgoing_transitions_) {
          auto transitions = pair.second;
          auto active_transitions = resolver_->sortTransitions(transitions, false);
          int count = 0;

          for (const auto& trans : transitions) {
             if (mc_mode && trans->state_source_ == INVALID_STATE_NUM) {
                continue;
             }

             int cond_strength = 1;

             if (mark_current_state
                && getCurrentState() == trans->state_source_
                && std::count(destination_states_from_current.begin(), destination_states_from_current.end(), trans->state_destination_)
                && std::count(active_transitions.begin(), active_transitions.end(), trans))
             {
                int fac = !active_transitions.empty() && active_transitions[0] == trans ? 3 : 0;
                cond_strength = 3 + fac * (active_transitions.size() > 1);
             }

             std::string addons = "penwidth=" + std::to_string(cond_strength) + ",";
             std::string trans_color_if_reachable = NORMAL_TRANSITION_COLOR;

             if (resolver_->getRedundantState(trans->state_source_) == trans->state_destination_) {
                trans_color_if_reachable = REDUNDANT_TRANSITION_COLOR;
             }
             else if (trans->condition_->isAlwaysTrue()) {
                trans_color_if_reachable = ALWAYS_TRUE_TRANSITION_COLOR;
             }
             else if (trans->condition_->isAlwaysFalse()) {
                trans_color_if_reachable = ALWAYS_FALSE_TRANSITION_COLOR;
             }

             std::string cond_serialized = trans->condition_->serialize();

             std::string actual_trans_label = mc_mode
                && (trans_color_if_reachable == ALWAYS_TRUE_TRANSITION_COLOR
                   || trans_color_if_reachable == REDUNDANT_TRANSITION_COLOR)
                ? ""
                : cond_serialized;

             if (mc_mode) {
                trans_color_if_reachable = NORMAL_TRANSITION_COLOR;
             }

             std::string prio = (!mc_mode && transitions.size() > 1) ? std::string(" <") + std::to_string(count++) + ">" : "";

             addons += mc_mode || reachable_states.count(trans->state_source_)
                ? "color=" + trans_color_if_reachable + ",fontcolor=" + trans_color_if_reachable + ","
                : "color=" + UNREACHABLE_STATE_COLOR + ",fontcolor=" + UNREACHABLE_STATE_COLOR + ",";
             result.append(stateNameGV(trans->state_source_))
                .append("->")
                .append(stateNameGV(trans->state_destination_))
                .append("[")
                .append(addons)
                .append("label=\"")
                .append(StaticHelper::shortenToMaxSize(actual_trans_label, 1000, 30))
                .append(prio)
                .append("\"")
                .append("];\n");
          }
       }
    }

   result.append("}");

   return result;
}

template<class F>
std::string FSM<F>::getXWizardAnimationOfSeveralSteps(const int steps, const SteppingOrderEnum ordering) const
{
    std::string result = std::string("latex:%varm|gra%\n");
    int animation_num = 0;
    std::shared_ptr<DataPack> data_copy = std::make_shared<DataPack>();
   data_copy->initializeValuesBy(data_);

    result.append(std::string("x") + std::to_string(animation_num) + std::string("=@{\n"))
    .append("dot:\n")
   .append(getDOTCodeOfCurrentGraph(false))
   .append("}@\n");

    for (animation_num = 1; animation_num <= steps; ++animation_num) {
      result.append(std::string("x") + std::to_string(animation_num) + std::string("=@{\n"))
        .append("dot:\n")
      .append(getDOTCodeOfCurrentGraph())
      .append("}@\n");
        step(ordering);
    }

    result.append("--declarations--\n").append("animate=");

    for (animation_num = 0; animation_num <= steps; ++animation_num) {
        result.append(std::string("x") + std::to_string(animation_num) + std::string("->"));
    }

    result.append(";\n").append("--declarations-end--");

    data_->initializeValuesBy(data_copy);

    return result;
}

template<class F>
int FSM<F>::createGraficOfCurrentGraph(const std::string& base_filename, const bool mark_current_state, const std::string& format, const bool mc_mode, const GraphvizOutputSelector which) const
{
   std::string command = "";
   std::ofstream file;
   std::string error_msg = "digraph G {A [label\"Malformed FSM\"];}";

   file.open(base_filename);
   
//    try {
      std::string graph = getDOTCodeOfCurrentGraph(mark_current_state, mc_mode, false, which);
      if (!graph.empty()) {
         file << graph;
      }
      else {
         file << error_msg;
      }
//    }
//    catch (...) {
//       file << error_msg;
//    }

   file.close();

   if (format.empty()) {
      return 0;
   } else {
      return StaticHelper::createImageFromGraphvizDotFile(base_filename, format);
   }
}

///////////////// EO XWizard/Graphviz output ////////////////

template<class F>
inline void FSM<F>::addEnumType(const std::string & enum_name, const std::map<int, std::string>& mapping)
{
   data_->addEnumMapping(enum_name, mapping);
}

} // fsm
} // vfm
