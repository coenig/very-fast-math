//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "fsm.h"
#include "failable.h"
#include "simulation/highway_image.h"
#include <vector>

namespace vfm {

typedef std::pair<CarPars, CarParsVec> SystemState;
typedef std::vector<SystemState> SystemStateVec;
typedef std::function<SystemStateVec(const SystemState& state)> TransFuncType;
typedef std::pair<SystemStateVec, TransFuncType> ModelConfiguration;
typedef std::pair<ModelConfiguration, std::vector<float>> ModelDesign;

class MC : public Failable {
public:
   static const ModelConfiguration CONFIG_HELLO_WORLD;
   static const ModelConfiguration CONFIG_SIMPLE_ACC;
   static const ModelConfiguration CONFIG_OVERTAKE;

   static const ModelDesign DESIGN_SIMPLE_ACC;

   // This constructor is called eventually.
   MC(const SystemStateVec& seed, const TransFuncType& trans_func, const int image_width = 200, const int image_height = 100, const float ego_offset_x = 0) : Failable("MC_State_Space_Generator"),
      seed_(seed), trans_func_(trans_func), img_width_(image_width), img_height_(image_height), ego_offset_x_(ego_offset_x)
   {
      ascii_table_ = Image::retrieveAsciiTable();
      model_.setProhibitDoubleEdges(true);
   }

   MC(const ModelConfiguration& config) : MC(config.first, config.second) {}
   MC(const ModelDesign& design) : MC(design.first.first, design.first.second, design.second[0], design.second[1], design.second[2]) {}

   void generateOnePath(const int num_to_take_on_nondeterminism = -1, const std::shared_ptr<std::function<bool(const SystemState& state)>> fct = nullptr, const int max_states = 100000);

   void generateModel(
      const bool try_reinserting_immediate_next_state = false, // Makes smaller graphs more connected, but can lead to infinite loops. => Choose larger insert_only_first_n_for_every_state.
      const int abort_after_n_transitions = 100000,
      const int abort_after_n_states = 100000,
      const int insert_only_first_n_for_every_state = 100000,
      const bool complement_of_transitions = false, 
      const int insert_every_nth_transition_only = 1,
      const bool insert_unconnected_system_states = true);

   inline void generateFullModel() 
   {
      // Full model (except that unconnected states are omitted; set last parameter to true if desired).
      generateModel();
   }
   
   static SystemStateVec generateFullSystemSeedACC();
   
   fsm::FSMs getModel();

   static float getSpeed(const CarPars& cp);
   static float getPos(const CarPars& cp);
   static float getLane(const CarPars& cp);
   static float getEgoSpeed(const SystemState& state);
   static float getEgoLane(const SystemState& state);
   static float getOtherSpeed(const SystemState& state, const int other_num);
   static float getOtherPos(const SystemState& state, const int other_num);
   static float getOtherLane(const SystemState& state, const int other_num);
   static void setEgoSpeed(SystemState& state, const float val);
   static void setEgoLane(SystemState& state, const float val);
   static void setOtherSpeed(SystemState& state, const int other_num, const float val);
   static void setOtherPos(SystemState& state, const int other_num, const float val);
   static std::string toString(const CarPars& s, const std::string& name = "");
   static std::string toString(const CarParsVec& s, const std::string& name = "");
   static std::string toString(const SystemState& s);
   static std::string toString(const SystemStateVec& sv);

   static SystemStateVec performCarShiftIfNecessary(SystemState& state);

   static CarParsVec getCarsWithProperty(const SystemState& state, const std::function<bool(const CarPars& pars)>& func);
   static CarParsVec getCarsInFrontOfOtherSorted(const SystemState& state, const int other_num, const int lane_offset = 0, const bool reverse = false);
   static CarParsVec getCarsBehindOfOtherSorted(const SystemState& state, const int other_num, const int lane_offset = 0);
   static CarParsVec getCarsInFrontOfEgoSorted(const SystemState& state, const int lane_offset = 0);
   static CarParsVec getCarsBehindOfEgoSorted(const SystemState& state, const int lane_offset = 0);

   static SystemStateVec allNondetChoicesOfOthers(
      const SystemState& state, 
      const float plus_minus, 
      const float step, 
      const bool speed, /* or else distance */
      const std::function<bool(const SystemState& state)> is_ok = [](const SystemState& state) { return true; });

private:
   int addStateToSystem(
      const SystemState& state, 
      std::map<SystemState, int>& states, 
      std::map<int, SystemState>& states_rev, 
      std::vector<int>& unprocessed,
      int& max_state_num);
   
   void addPainting(const int state_num, const SystemState& state);

   SystemStateVec seed_;
   TransFuncType trans_func_;
   int img_width_;
   int img_height_;
   float ego_offset_x_ = 0;
   fsm::FSMs model_;
   std::vector<Image> ascii_table_;
};

} // vfm
