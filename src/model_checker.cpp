//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/model_checker.h"
#include "simulation/highway_image.h"

using namespace vfm;

const ModelConfiguration MC::CONFIG_HELLO_WORLD = {
   { { { 0, 0, 0 }, { {0, 9, 0 } } } },
   [](const SystemState& state) {
      std::vector<SystemState> vec;
      SystemState s = state;
      setEgoSpeed(s, ((int) (getEgoSpeed(s) + 1)) % 5);
      vec.push_back(s);
      return vec;
   } };

// ACC
const int SPEED_MIN_ACC = 0;                             // 0
const int SPEED_MAX_ACC = 2;                   // 10     // 2
const int SPEED_STEP_ACC = 1;                            // 1
const int DISTANCE_MIN_ACC = 9;  // "Close"    // 10     // 9
const int DISTANCE_MAX_ACC = 27; // "Infinite" // 25     // 27
const int DISTANCE_STEP_ACC = 9;               // 3      // 9

const ModelConfiguration MC::CONFIG_SIMPLE_ACC = {
   MC::generateFullSystemSeedACC(), // SEED
   [](const SystemState& state) {   // TRANSITION FUNCTION.
      SystemStateVec vec;
      SystemState s1 = state, s2 = state, s3 = state;
      int egoSpeed = getEgoSpeed(state);
      int otherSpeed = getOtherSpeed(state, 0);
      int otherPos = getOtherPos(state, 0);

      // if (otherPos < DISTANCE_MAX_ACC) {
         if (egoSpeed > otherSpeed) {
            otherPos -= DISTANCE_STEP_ACC * (egoSpeed - otherSpeed);
            egoSpeed -= SPEED_STEP_ACC;
         } else if (egoSpeed < otherSpeed) {
            otherPos += DISTANCE_STEP_ACC * (otherSpeed - egoSpeed);
            egoSpeed += SPEED_STEP_ACC;
         }
      // } else {
         // for (int i = SPEED_MIN_ACC; i < egoSpeed; i++) {
         //    SystemState s = state;
         //    setOtherPos(s, 0, DISTANCE_MAX_ACC - DISTANCE_STEP_ACC);
         //    setOtherSpeed(s, 0, i);
         //    vec.push_back(s);
         // }

         // return vec;
      // }

      // if (otherPos >= DISTANCE_MAX_ACC) { // Only one state in case of "infinite" distance.
      //    otherPos = DISTANCE_MAX_ACC;
      //    otherSpeed = SPEED_MIN_ACC - SPEED_STEP_ACC;
      // }

      if (otherPos < DISTANCE_MIN_ACC) { // No valid following states available.
         return vec;
      }

      setEgoSpeed(s1, egoSpeed);
      setOtherSpeed(s1, 0, std::min(std::max(otherSpeed, SPEED_MIN_ACC), SPEED_MAX_ACC));
      setOtherPos(s1, 0, otherPos);
      setEgoSpeed(s2, egoSpeed);
      setOtherSpeed(s2, 0, std::min(SPEED_MAX_ACC, otherSpeed + 1));
      setOtherPos(s2, 0, otherPos);
      setEgoSpeed(s3, egoSpeed);
      setOtherSpeed(s3, 0, std::max(SPEED_MIN_ACC, otherSpeed - 1));
      setOtherPos(s3, 0, otherPos);

      vec.push_back(s1);
      vec.push_back(s2);
      vec.push_back(s3);

      return vec;
   } };

const std::vector<float> DRAW_CONFIG_ACC{ 230, 40, -9 };

const ModelDesign MC::DESIGN_SIMPLE_ACC{ MC::CONFIG_SIMPLE_ACC, DRAW_CONFIG_ACC };
// EO ACC

// OVERTAKE
const int LANES = 3;
const int SPEED_MIN_LOWEST_LANE = 70; // 80, 90, ...
const int SPEED_MAX_LOWEST_LANE = 90; // 100, 110, ...
const int SPEED_INCREASE_TO_NEXT_LANE = 20;
const int SPEED_STEP_OVERTAKE = 10;
const float DISTANCE_FACTOR_OVERTAKE = 0.1;
const float DISTANCE_MAX_OVERTAKE = 80;
const float DISTANCE_STEP_OVERTAKE = 1;
const int TARGET_SPEED_OVERTAKE = 120;

const float MAX_DIST_TO_ADJUST_SPEED_EGO = 15;
const float MAX_DIST_TO_ADJUST_SPEED_OTHERS = 100000;
const float MIN_GAP_TO_MERGE = 11;

const auto lane_speed_max = [](const int lane_num) { return SPEED_MAX_LOWEST_LANE + SPEED_INCREASE_TO_NEXT_LANE * (LANES - lane_num - 1); };
const auto lane_speed_min = [](const int lane_num) { return SPEED_MIN_LOWEST_LANE + SPEED_INCREASE_TO_NEXT_LANE * (LANES - lane_num - 1); };

void doEgoBehaviorOvertake(SystemState& s)
{
   CarParsVec front_vec = MC::getCarsInFrontOfEgoSorted(s);
   CarParsVec front_l_vec = MC::getCarsInFrontOfEgoSorted(s, -1);
   CarParsVec behind_l_vec = MC::getCarsBehindOfEgoSorted(s, -1);
   CarParsVec front_r_vec = MC::getCarsInFrontOfEgoSorted(s, 1);
   CarParsVec behind_r_vec = MC::getCarsBehindOfEgoSorted(s, 1);
   float ego_speed = MC::getEgoSpeed(s);

   if (ego_speed < TARGET_SPEED_OVERTAKE) { // So far, ego speed cannot be greater than target speed.
      MC::setEgoSpeed(s, ego_speed + SPEED_STEP_OVERTAKE);
   }

   auto front = front_vec[0];

   float ego_pos = 0;
   float front_speed = MC::getSpeed(front);

   if (MC::getPos(front) - ego_pos < MAX_DIST_TO_ADJUST_SPEED_EGO && ego_speed > front_speed) {
      bool can_overtake = false;
      if (MC::getEgoLane(s) > 0) {
         auto front_l = front_l_vec[0];
         auto behind_l = behind_l_vec[0];
         can_overtake = -MC::getPos(behind_l) > MIN_GAP_TO_MERGE && MC::getPos(front_l) > MIN_GAP_TO_MERGE;
      }

      if (can_overtake) {
         MC::setEgoLane(s, MC::getEgoLane(s) - 1);
         return;
      }
      else if (MC::getPos(front) - ego_pos < MIN_GAP_TO_MERGE) {
         MC::setEgoSpeed(s, front_speed - SPEED_STEP_OVERTAKE);
      }
      else {
         MC::setEgoSpeed(s, front_speed);
      }
   }

   if (MC::getEgoLane(s) < LANES - 1) {
      auto front_r = front_r_vec[0];
      auto behind_r = behind_r_vec[0];
      bool can_merge_right = -MC::getPos(behind_r) > CAR_LENGTH + 1 && MC::getPos(front_r) > MIN_GAP_TO_MERGE;;

      if (can_merge_right) {
         MC::setEgoLane(s, MC::getEgoLane(s) + 1);
      }
   }
}

const ModelConfiguration MC::CONFIG_OVERTAKE = {
   { { CarPars{ 1, 0, lane_speed_min(1) },
    { {0, -DISTANCE_STEP_OVERTAKE * 14, lane_speed_min(0) },
      {0, DISTANCE_STEP_OVERTAKE, lane_speed_max(0) },
      {1, -DISTANCE_MAX_OVERTAKE, lane_speed_min(1) },
      {1, DISTANCE_STEP_OVERTAKE * 14, lane_speed_min(1) },
      {2, -DISTANCE_MAX_OVERTAKE, lane_speed_min(2) },
      {2, DISTANCE_STEP_OVERTAKE * 7, lane_speed_min(2) } } } },
   [](const SystemState& state) {
      SystemState st = state;
      float ego_speed = getEgoSpeed(st);

      doEgoBehaviorOvertake(st);

      for (auto& other : st.second) {
         float other_speed = getSpeed(other);
         float diff = (ego_speed - other_speed) * DISTANCE_FACTOR_OVERTAKE;
         float real_dist = getPos(other) - diff;

         if (real_dist > DISTANCE_MAX_OVERTAKE) {
            real_dist = DISTANCE_MAX_OVERTAKE;
         }

         if (real_dist < -DISTANCE_MAX_OVERTAKE) {
            real_dist = -DISTANCE_MAX_OVERTAKE;
         }

         float given_dist = real_dist;
         float min_diff = 100000000000;
         for (float i = -DISTANCE_MAX_OVERTAKE; i <= DISTANCE_MAX_OVERTAKE; i += DISTANCE_STEP_OVERTAKE) {
            float diff = std::abs(i - real_dist);
            if (diff < min_diff) {
               min_diff = diff;
               given_dist = i;
            }
         }

         other.car_rel_pos_ = given_dist;
      }

      auto nondet_vec = allNondetChoicesOfOthers(st, 10, 10, true, [](const SystemState s) {
         for (int i = 0; i < s.second.size(); i++) {
            float other_lane = getOtherLane(s, i);
            float other_speed = getOtherSpeed(s, i);
            float other_pos = getOtherPos(s, i);

            if (lane_speed_max(other_lane) < other_speed || lane_speed_min(other_lane) > other_speed) {
               return false;
            }

            auto vec = getCarsInFrontOfOtherSorted(s, i);
            if (!vec.empty()) {
               CarPars cp = vec[0];

               if (cp.car_rel_pos_ - other_pos < MAX_DIST_TO_ADJUST_SPEED_OTHERS && other_speed > cp.car_velocity_) {
                  return false;
               }
            }
         }

         return true; 
      });

      SystemStateVec vec;

      for (auto& item : nondet_vec) {
         SystemStateVec v = performCarShiftIfNecessary(item);
         vec.insert(vec.end(), v.begin(), v.end());
      }

      return nondet_vec;
   } };
// EO OVERTAKE

void vfm::MC::generateOnePath(const int num_to_take_on_nondeterminism, const std::shared_ptr<std::function<bool(const SystemState& state)>> fct, const int max_states)
{
   srand((unsigned) time(NULL));
   std::vector<SystemState> chain;
   chain.push_back(seed_[(num_to_take_on_nondeterminism >= 0 ? num_to_take_on_nondeterminism : rand()) % seed_.size()]);
   SystemState state = chain[0];
   int loop = -1;

   for (int i = 0; i < max_states; i++) {
      auto states = trans_func_(chain[chain.size() - 1]);

      if (states.empty()) {
         break;
      }

      state = states[(num_to_take_on_nondeterminism >= 0 ? num_to_take_on_nondeterminism : rand()) % states.size()];

      if (fct) {
         for (const auto& s : states) {
            if ((*fct)(s)) {
               state = s;
            }
         }
      }

      for (int i = 0; i < chain.size(); i++) {
         if (chain[i] == state) {
            loop = i;
         }
      }

      if (loop > 0) {
         break;
      }

      chain.push_back(state);
   }

   for (int i = 0; i < chain.size() - 1; i++) {
      model_.addTransition(i + 1, i + 2);
      addPainting(i + 1, chain[i]);
   }

   addPainting(chain.size(), chain[chain.size() - 1]);

   if (loop >= 0) {
      model_.addTransition(chain.size(), loop + 1);
   }
}

void vfm::MC::generateModel(
   const bool try_reinserting_immediate_next_state,
   const int abort_after_n_transitions,
   const int abort_after_n_states,
   const int insert_only_first_n_for_every_state,
   const bool complement_of_transitions, 
   const int insert_every_nth_transition_only, 
   const bool insert_unconnected_system_states)
{
   int state_counter = 1;
   std::map<SystemState, int> all_states;
   std::map<int, SystemState> all_states_rev;
   std::vector<int> unprocessed;
   int i = 0;
   int num_old_trans = -1;
   int num_old_states = -1;
   int num_old_unprocessed = -1;

   for (const auto& s : seed_) {
      addStateToSystem(s, all_states, all_states_rev, unprocessed, state_counter);
   }

   while (!unprocessed.empty()) {
      int current_state_num = unprocessed[0];
      SystemState current_state = all_states_rev[current_state_num];
      auto next_states_vec = trans_func_(current_state);
      std::set<SystemState> next_states(next_states_vec.begin(), next_states_vec.end());
      
      int next_state_counter = 0;

      addPainting(current_state_num, current_state);

      for (const auto& next_state : next_states) {
         i++;
         int next_state_num = addStateToSystem(next_state, all_states, all_states_rev, unprocessed, state_counter);
         if (i % insert_every_nth_transition_only == 0 && next_state_counter++ < insert_only_first_n_for_every_state) {
            model_.addTransition(current_state_num, next_state_num);

            if (try_reinserting_immediate_next_state) {
               unprocessed.insert(unprocessed.begin(), next_state_num);
            }
         }
         else if (insert_unconnected_system_states) {
            model_.addUnconnectedStateIfNotExisting(current_state_num);
            model_.addUnconnectedStateIfNotExisting(next_state_num);
         }
         addPainting(next_state_num, next_state);
      }

      unprocessed.erase(unprocessed.begin());
      addNote("System states: " + std::to_string(model_.getNumStates()) + ", transitions: " + std::to_string(model_.getTransitions()->size()) 
              + " (" + std::to_string(i) + ")" + ", unprocessed: " + std::to_string(unprocessed.size()));

      if (model_.getTransitions()->size() >= abort_after_n_transitions) {
         break;
      }

      if (model_.getNumStates() >= abort_after_n_states) {
         break;
      }

      if (num_old_states == model_.getNumStates()
            && num_old_trans == model_.getTransitions()->size()
            && num_old_unprocessed == unprocessed.size()) {
         break;
      }

      num_old_states = model_.getNumStates();
      num_old_trans = model_.getTransitions()->size();
      num_old_unprocessed = unprocessed.size();
   }

   if (complement_of_transitions) {
      addNote("Creating complement of model... Transforming from " + std::to_string(model_.getTransitions()->size()) + " to ", true, "");
      model_.createComplementOfTransitions();
      addNotePlain(std::to_string(model_.getTransitions()->size()) + " transitions.");
   }

   addNote("Generation complete.");
}

SystemStateVec vfm::MC::generateFullSystemSeedACC()
{
   SystemStateVec vec;

   std::vector<float> vals{ (float) SPEED_MIN_ACC, (float) SPEED_MIN_ACC, (float) DISTANCE_MIN_ACC};

   for (;;) {
      if (vals[2] != DISTANCE_MAX_ACC || vals[1] == 0) {
         SystemState state = { { 0, 0, (int) vals[0] }, { { 0, vals[2], (int) vals[1] } } };
         vec.push_back(state);
      }

      if (vals[2] < DISTANCE_MAX_ACC) {
         vals[2] += DISTANCE_STEP_ACC;
      }
      else {
         vals[2] = DISTANCE_MIN_ACC;
         if (vals[1] < SPEED_MAX_ACC) {
            vals[1] += SPEED_STEP_ACC;
         }
         else {
            vals[1] = SPEED_MIN_ACC;
            if (vals[0] < SPEED_MAX_ACC) {
               vals[0] += SPEED_STEP_ACC;
            }
            else {
               vals[0] = SPEED_MIN_ACC;
            }
         }
      }

      if (vals[0] == SPEED_MIN_ACC && vals[1] == SPEED_MIN_ACC && vals[2] == DISTANCE_MIN_ACC) {
         break;
      }
   }

   return vec;
}

fsm::FSMs vfm::MC::getModel()
{
   return model_;
}

void vfm::MC::addPainting(const int state_num, const SystemState& state)
{
   if (!model_.hasStateImage(state_num)) {
      std::shared_ptr<HighwayImage> small = std::make_shared<HighwayImage>(img_width_, img_height_, std::make_shared<Plain2DTranslator>(), 3); // TODO: magic lane number.
      small->paintHighwayScene(state.first, state.second, {}, ego_offset_x_);
      model_.addImageToState(state_num, small);
   }
}

int vfm::MC::addStateToSystem(
   const SystemState& state,
   std::map<SystemState, int>& states,
   std::map<int, SystemState>& states_rev,
   std::vector<int>& unprocessed,
   int& state_counter)
{
   for (const auto& pair : states) {
      SystemState s = pair.first;

      if (s == state) {
         return states[s];
      }
   }

   states[state] = state_counter;
   states_rev[state_counter] = state;
   unprocessed.push_back(state_counter);
   state_counter++;

   return state_counter - 1;
}

float vfm::MC::getSpeed(const CarPars& cp)
{
   return cp.car_velocity_;
}

float vfm::MC::getPos(const CarPars& cp)
{
   return cp.car_rel_pos_;
}

float vfm::MC::getLane(const CarPars& cp)
{
   return cp.car_lane_;
}

// EGO is usually at pos = 0, therefore no method for that.
float MC::getEgoSpeed(const SystemState& state)
{
   return getSpeed(state.first);
}

float vfm::MC::getEgoLane(const SystemState& state)
{
   return getLane(state.first);
}

float MC::getOtherSpeed(const SystemState& state, const int other_num)
{
   return getSpeed(state.second[other_num]);
}

float MC::getOtherPos(const SystemState& state, const int other_num)
{
   return getPos(state.second[other_num]);
}

float vfm::MC::getOtherLane(const SystemState &state, const int other_num)
{
   return getLane(state.second[other_num]);
}

void MC::setEgoSpeed(SystemState& state, const float val)
{
   state.first.car_velocity_ = val;
}

void vfm::MC::setEgoLane(SystemState& state, const float val)
{
   state.first.car_lane_ = val;
}

void MC::setOtherSpeed(SystemState& state, const int other_num, const float val)
{
   state.second[other_num].car_velocity_ = val;
}

void MC::setOtherPos(SystemState& state, const int other_num, const float val)
{
   state.second[other_num].car_rel_pos_ = val;
}

SystemStateVec vfm::MC::performCarShiftIfNecessary(SystemState& state)
{
   SystemStateVec vec;

   for (int i = 0; i < LANES; i++) {
      int ind1 = 2 * i;
      int ind2 = 2 * i + 1;
      CarPars car1 = state.second[ind1];
      CarPars car2 = state.second[ind2];

      if (getPos(car1) > 0) {
         state.second[ind2] = car1;
         setOtherPos(state, ind1, -DISTANCE_MAX_OVERTAKE);
         setOtherSpeed(state, ind1, lane_speed_max(i));
      }

      if (getPos(car2) <= 0) {
         state.second[ind1] = car2;
         setOtherPos(state, ind2, DISTANCE_MAX_OVERTAKE);
         setOtherSpeed(state, ind2, lane_speed_min(i));
      }
   }

   vec.push_back(state);

   return vec;
}

CarParsVec vfm::MC::getCarsWithProperty(
   const SystemState& state, const std::function<bool(const CarPars& pars)>& func)
{
   CarParsVec vec;

   if (func(state.first)) {
      vec.push_back(state.first); // EGO
   }

   for (const auto& cp : state.second) { // OTHERS
      if (func(cp)) {
         vec.push_back(cp);
      }
   }

   return vec;
}

CarParsVec vfm::MC::getCarsInFrontOfOtherSorted(const SystemState& state, const int other_num, const int lane_offset, const bool reverse)
{
   int real_lane = other_num < 0 ? getLane(state.first) : getLane(state.second[other_num]);
   int lane = real_lane + lane_offset;
   float pos = other_num < 0 ? getPos(state.first) : getPos(state.second[other_num]);
      
   CarParsVec vec = getCarsWithProperty(state, [lane, pos, reverse, lane_offset](const CarPars& pars) {
      return getLane(pars) == lane && (reverse ? pos > getPos(pars) || lane_offset != 0 && pos == getPos(pars) : pos < getPos(pars));
   });

   std::sort(vec.begin(), vec.end(), [reverse](const CarPars& c1, const CarPars& c2) {
      return reverse ? getPos(c1) >= getPos(c2) : getPos(c1) < getPos(c2);
   });

   return vec;
}

CarParsVec vfm::MC::getCarsBehindOfOtherSorted(const SystemState& state, const int other_num, const int lane_offset)
{
   return getCarsInFrontOfOtherSorted(state, other_num, lane_offset, true);
}

CarParsVec vfm::MC::getCarsInFrontOfEgoSorted(const SystemState& state, const int lane_offset)
{
   return getCarsInFrontOfOtherSorted(state, -1, lane_offset);
}

CarParsVec vfm::MC::getCarsBehindOfEgoSorted(const SystemState& state, const int lane_offset)
{
   return getCarsBehindOfOtherSorted(state, -1, lane_offset);
}

typedef std::pair<float, std::pair<float, int>> CarPars;

std::string vfm::MC::toString(const CarPars& s, const std::string& name)
{
   std::string str;

   str += std::string("[") + name + "_vel  = " + std::to_string(getSpeed(s)) + ", ";
   str += std::string("") + name + "_lane = " + std::to_string(getLane(s)) + ", ";
   str += std::string("") + name + "_pos = " + std::to_string(getPos(s)) + "]";

   return str;
}

std::string vfm::MC::toString(const CarParsVec& s, const std::string& name)
{
   std::string str = "[";

   int cnt = 0;
   for (const auto& car_pars : s) {
      str += toString(car_pars, name + std::to_string(cnt));
      cnt++;
   }

   return str + "]";
}

std::string vfm::MC::toString(const SystemState& s)
{
   std::string str = "[";

   str += toString(s.first, "ego");
   str += ", ";
   str += toString(s.second, "other");

   return str + "]";
}

std::string vfm::MC::toString(const SystemStateVec& sv)
{
   std::string str = "[";

   for (const auto& itm : sv) {
      str += toString(itm) + "," + "\r\n ";
   }

   return str + "]";
}

SystemStateVec vfm::MC::allNondetChoicesOfOthers(
   const SystemState& state,
   const float plus_minus,
   const float step,
   const bool speed,
   const std::function<bool(const SystemState& state)> is_ok)
{
   SystemStateVec vec;
   int num = 0;
   int num_others = state.second.size();
   std::vector<float> curr;
   float max = 0;
   float min = 0;
   float pm = std::abs(plus_minus);
   bool goon = true;

   for (max = 0; max < pm; max += step) {
      num++;
   }
   min -= max;

   for (const auto& i : state.second) {
      curr.push_back(min);
   }

   while (goon) {
      SystemState s = state;

      for (int i = 0; i < s.second.size(); i++) {
         if (speed) {
            s.second[i].car_velocity_ += curr[i];
         }
         else {
            s.second[i].car_lane_ += curr[i];
         }
      }

      if (is_ok(s)) {
         vec.push_back(s);
      }

      for (int i = 0; i < curr.size(); i++) {
         if (curr[i] + step <= max) {
            curr[i] += step;
            break;
         }
         else {
            if (i == curr.size() - 1) {
               goon = false;
            }
            curr[i] = min;
         }
      }
   }

   return vec;
}