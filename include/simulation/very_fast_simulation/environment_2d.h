//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "simulation/env2d_simple.h"
#include "parser.h"
#include "fsm.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <memory>


using namespace vfm::fsm;

namespace vfm {

const std::string EGO_POS_Y_NAME = "ego_pos_y";
const std::string EGO_LEAD_CAR_NAME = "ego_lead_car";
const std::string EGO_VX_NAME = "ego_vx";
const std::string EGO_AX_NAME = "ego_ax";
const std::string EGO_VY_NAME = "ego_vy";
const std::string AGENTS_POS_X_NAME = "agents_pos_x";
const std::string AGENTS_POS_Y_NAME = "agents_pos_y";
const std::string AGENTS_VX_REL_NAME = "agents_vx_rel";
const std::string AGENTS_VY_NAME = "agents_vy";
//const std::vector<std::string> SENSORS_TO_CONTROLLER = { EGO_POS_Y_NAME, EGO_LEAD_CAR_NAME, EGO_VX_NAME, AGENTS_POS_X_NAME, AGENTS_POS_Y_NAME, AGENTS_VX_REL_NAME, AGENTS_VY_NAME };
//const std::vector<std::string> ACTUATORS_TO_CONTROLLER = { EGO_AX_NAME, EGO_VY_NAME };

constexpr int IDLE_ID = 0;
constexpr int LCL_ID = 1;
constexpr int LCR_ID = 2;
constexpr int ACC0_ID = 3;
constexpr int ACCPLUS_ID = 4;
constexpr int ACCMINUS_ID = 5;

inline static int MIDDLE_NUM_CARS = -1;
static constexpr int NUM_LANES = 3;
static constexpr int SPEED_DIVISOR_FOR_STEP_SMOOTHNESS_INT = (int)SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;
static constexpr float MAX_SPEED_PER_LANE[5] = {
   150 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   130 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   110 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   90 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   80 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS };

static constexpr float MIN_SPEED_PER_LANE[5] = {
   120 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   110 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   80 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   70 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS,
   70 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS };

static constexpr float AVG_SPEED_PER_LANE[5] = {
   (MAX_SPEED_PER_LANE[0] + MIN_SPEED_PER_LANE[0]) / 2,
   (MAX_SPEED_PER_LANE[1] + MIN_SPEED_PER_LANE[1]) / 2,
   (MAX_SPEED_PER_LANE[2] + MIN_SPEED_PER_LANE[2]) / 2,
   (MAX_SPEED_PER_LANE[3] + MIN_SPEED_PER_LANE[3]) / 2,
   (MAX_SPEED_PER_LANE[4] + MIN_SPEED_PER_LANE[4]) / 2 };

inline static float RANGE_SPEED_PER_LANE_PER_CAR[5] = {};

static constexpr float SPEED_STEP = 1 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;
static constexpr float ACCELERATION_STEP = 0.05 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;
static constexpr float LANE_CHANGE_SPEED = 2.4 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;
static constexpr float EPSILON = LANE_CHANGE_SPEED / 2;
static constexpr int HALF_SMOOTH_DIVISOR = SPEED_DIVISOR_FOR_STEP_SMOOTHNESS_INT > 2 ? (SPEED_DIVISOR_FOR_STEP_SMOOTHNESS_INT / 2) : 1;

static constexpr float EGO_ACCELERATION_STEP = 0.5 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;
static constexpr float EGO_ACCELERATION_MAX = 5 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;
static constexpr float EGO_SPEED_MAX = 200 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;
static constexpr float EGO_SPEED_MIN = 0 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;

template<size_t MAX_CARS>
class Environment2D : public Env2D {
public:
   const Polygon2D<float> CAR_POLYGON_SHAPE{ {0, 0}, {0, CAR_WIDTH}, {CAR_LENGTH, CAR_WIDTH}, {CAR_LENGTH, 0} };

   Environment2D(const int num_cars);

   /// \brief One sim step. automatic_re_sort needed for calculation of lead vehicle etc. But only if set
   /// to false, agent IDs will remain constant. (TODO: get best from both worlds.)
   void step(const bool do_random_lanechanges, const bool automatic_re_sort, const bool calculate_fitness, const bool auto_correct_speed, const bool torus);

   std::string getAsciiView() const;
   std::shared_ptr<FSM<Environment2D>> getEgoController() const;
   void setEgoController(const std::shared_ptr<FSM<Environment2D>> controller);
   void registerEgoController(const std::shared_ptr<FSM<Environment2D>> controller);
   void resetAndRandomizeTraffic(const bool reset_fitness);

   float getEgoFitness() const;

   void resetEgoAcceleration(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName);
   void incrementEgoAcceleration(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName);
   void decrementEgoAcceleration(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName);
   void laneChangeEgoLeft(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName);
   void laneChangeEgoRight(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName);
   void idleCommand(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName);

   std::shared_ptr<DataPack> getData() const;

public: // TODO: private:
   std::shared_ptr<FSM<Environment2D>> ego_controller_;
   float ego_fitness_;
   bool collided_in_last_step_;

   // Sensors/Actuators to FSM controller.
   float agents_pos_x_[MAX_CARS];
   float agents_pos_y_[MAX_CARS];
   float agents_vx_rel_[MAX_CARS];
   float agents_vy_[MAX_CARS];
   float agents_ax_[MAX_CARS]; // Could be a sensor, not sure yet.
   char agents_lead_cars_[MAX_CARS + 1]; // Could be a sensor, not sure yet.

   float ego_vx_ = 0;
   float ego_pos_y_ = 0;

   float ego_ax_ = 0; // Actuator.
   float ego_vy_ = 0; // Actuator.

   // EO Sensors/Actuators to FSM controller.

   bool needs_recompute_lead_cars_;
   long count_;

   std::shared_ptr<DataPack> data_ = nullptr;

   int lane(const int i) const;
   int lanePlain(const float val) const;
   int egoLane() const;
   bool collides(const float x1, const float y1, const float x2, const float y2) const;
   int collidesWithAny(const float x1, const float y1, const int ignore) const; /// Does NOT check collision with ego, use collidesWithAnyEgo.
   int collidesWithAny(const int id) const; /// Does NOT work with id = MAX_CARS for ego, use collidesWithAnyEgo.
   int collidesWithAnyEgo() const;
   template<class T> void swap(T* array, const int index1, const int index2);
   void swapAll(const int index1, const int index2);
   void restrictLaneChange(float& y_pos, float& y_vel, const int lan);
   float distanceInDrivingDirection(const float x1, const float x2) const;
   void calculateFitness();

   /// Assumed to be most efficient since on avg. only 0 to 1 swaps per step (with 10 cars).
   /// Could get worse with more dense traffic, but 20 cars still yield only 3 times greater
   /// overall runtime.
   void bubbleSort();
   
   /// Recomputes only if necessary.
   int getLeadCar(const int i);

   /// Assumes cars are sorted, recomputes always.
   int recomputeLeadCarFor(const int i) const;
};

template<size_t MAX_CARS>
inline Environment2D<MAX_CARS>::Environment2D(const int num_cars): Env2D(num_cars, "Environment2D")
{
   MIDDLE_NUM_CARS = num_cars_ / 2;

   for (int i = 0; i < 5; i++)
      RANGE_SPEED_PER_LANE_PER_CAR[i] = (MAX_SPEED_PER_LANE[i] - MIN_SPEED_PER_LANE[i]) / (float)num_cars_;
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::lane(const int i) const
{
   return lanePlain(agents_pos_y_[i]);
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::lanePlain(const float val) const
{
   return (int) std::round(val);
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::egoLane() const
{
   return (int) std::round(ego_pos_y_);
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::restrictLaneChange(float& y_pos, float& y_vel, const int lan)
{
   const bool close_to_lane_mid = std::abs(y_pos - lan) < EPSILON;

   if (y_vel > 0) {
      if (y_pos >= NUM_LANES - 1 || close_to_lane_mid) {
         y_vel = 0;
         y_pos = lan;
      }
   }
   else if (y_vel < 0) {
      if (y_pos <= 0 || close_to_lane_mid) {
         y_vel = 0;
         y_pos = lan;
      }
   }
}

template<size_t MAX_CARS>
inline float Environment2D<MAX_CARS>::distanceInDrivingDirection(const float x1, const float x2) const
{
   if (x1 <= x2) {
      return x2 - x1;
   }
   else {
      return RIGHT_MARGIN - LEFT_MARGIN - (x1 - x2);
   }
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::calculateFitness()
{
   if (collidesWithAnyEgo() >= 0) {
      ego_fitness_ -= 1;
   }

   //ego_fitness_ += ego_vx_;
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::step(const bool do_random_lanechanges, const bool automatic_re_sort, const bool calculate_fitness, const bool auto_correct_speed, const bool torus)
{
   float a = agents_lead_cars_[num_cars_];
   float b = getEgoController()->getData()->getSingleVal(EGO_LEAD_CAR_NAME);
   std::string s = std::to_string(a) + "  " + std::to_string(b) + "\r\n";
   if (a != b) {
      std::cout << s;
      std::cin.get();
   }

   ego_controller_->step();

   // Ego.
   const float ego_vx_old = ego_vx_;
   ego_vx_ = std::max(std::min(ego_vx_ + ego_ax_, EGO_SPEED_MAX), EGO_SPEED_MIN);
   const float ego_vx_diff = ego_vx_old - ego_vx_;

   ego_pos_y_ += ego_vy_;
   restrictLaneChange(ego_pos_y_, ego_vy_, egoLane());

   for (int i = 0; i < num_cars_; i++) {
      const float speed = agents_vx_rel_[i] + ego_vx_;
      const int lan = lane(i);
      const float error = 0 * ((float) i - ((float)num_cars_) / 2) * RANGE_SPEED_PER_LANE_PER_CAR[lan];
      const float avg_speed = AVG_SPEED_PER_LANE[lan] + error;
      const float my_pos = agents_pos_x_[i];

      if (auto_correct_speed) {
         agents_ax_[i] = agents_ax_[i] / 1.1 + ACCELERATION_STEP * (avg_speed - speed);
      }

      float temp_x = my_pos + agents_vx_rel_[i];

      if (torus) {
         if (temp_x < LEFT_MARGIN) {
            temp_x = RIGHT_MARGIN;
         }
         else if (my_pos > RIGHT_MARGIN) {
            temp_x = LEFT_MARGIN;
         }
      }

      if (do_random_lanechanges && static_cast<float>(rand()) / static_cast<float>(RAND_MAX) < 2.8 * SPEED_DIVISOR_FOR_STEP_SMOOTHNESS) {
         if (std::rand() % 2) {
            agents_vy_[i] = -LANE_CHANGE_SPEED;
         }
         else {
            agents_vy_[i] = LANE_CHANGE_SPEED;
         }
      }

      float temp_y = std::min(std::max(agents_pos_y_[i] + agents_vy_[i], 0.0f), (float) NUM_LANES - 1);

      agents_pos_x_[i] = temp_x;

      if (lanePlain(temp_y) != lane(i)) {
         needs_recompute_lead_cars_ = true; // Lane change occurred.
      }

      agents_pos_y_[i] = temp_y;
      agents_vx_rel_[i] += agents_ax_[i] + ego_vx_diff;
      
      const float DIST_THRESH = 33;
      const float SPEED_CORRECTOR = 2.0f / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS;

      if (auto_correct_speed && count_ % HALF_SMOOTH_DIVISOR == 0) {
         const int leading_car = getLeadCar(i);
         const float corr_restriction = 1;

         if (leading_car >= 0) {
            if (leading_car == num_cars_) { // Leading car is ego.
               const float dist = distanceInDrivingDirection(my_pos, 0);
               const float corr = std::max(((dist + 1) / 7), corr_restriction);
               if (dist < DIST_THRESH) {
                  agents_vx_rel_[i] = std::min(-SPEED_CORRECTOR / corr, agents_vx_rel_[i]);
               }
            }
            else {
               const float dist = distanceInDrivingDirection(my_pos, agents_pos_x_[leading_car]);
               const float corr = std::max(((dist + 1) / 7), corr_restriction);
               if (dist >= 0 && dist < DIST_THRESH) { // Includes special case if both are leading cars of each other.
                  agents_vx_rel_[i] = std::min(agents_vx_rel_[leading_car] - SPEED_CORRECTOR / corr, agents_vx_rel_[i]);
               }
            }
         }
      }

      //agents_vx_rel_[i] = std::max(std::min(agents_vx_rel_[i] + ego_vx_, MAX_SPEED_PER_LANE[lan]), MIN_SPEED_PER_LANE[lan] - 10) - ego_vx_;

      restrictLaneChange(agents_pos_y_[i], agents_vy_[i], lan);
   }

   if (automatic_re_sort) {
      bubbleSort();
   }

   if (calculate_fitness) {
      calculateFitness();
   }
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::resetAndRandomizeTraffic(const bool reset_fitness)
{
   if (reset_fitness) {
      ego_fitness_ = 0;
   }

   collided_in_last_step_ = false;
   ego_ax_ = 0;
   ego_vy_ = 0;
   needs_recompute_lead_cars_ = true;
   count_ = 0;

   ego_pos_y_ = NUM_LANES / 2; // std::rand() % NUM_LANES;
   ego_vx_ = 200 / SPEED_DIVISOR_FOR_STEP_SMOOTHNESS; // MAX_SPEED_PER_LANE[egoLane()]; // MIN_SPEED_PER_LANE[(int) ego_pos_y_] + static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX / (MAX_SPEED_PER_LANE[(int) ego_pos_y_] - MIN_SPEED_PER_LANE[(int) ego_pos_y_])));

   for (int i = 0; i < num_cars_; i++) {
      int lane = std::rand() % NUM_LANES;
      float pos_x = LEFT_MARGIN + static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX / (RIGHT_MARGIN - LEFT_MARGIN)));
      float speed = MIN_SPEED_PER_LANE[lane] + static_cast <float> (std::rand()) /( static_cast <float> (RAND_MAX / (MAX_SPEED_PER_LANE[lane] - MIN_SPEED_PER_LANE[lane])));

      while (collidesWithAny(pos_x, lane, -1) > 0) {
         lane = std::rand() % NUM_LANES;
         pos_x = LEFT_MARGIN + static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX / (RIGHT_MARGIN - LEFT_MARGIN)));
      }

      agents_pos_x_[i] = pos_x;
      agents_pos_y_[i] = lane;
      agents_ax_[i] = 0;
      agents_vx_rel_[i] = speed - ego_vx_;
      agents_vy_[i] = 0;
   }

   bubbleSort();
   getLeadCar(0);
}

template<size_t MAX_CARS>
inline std::string Environment2D<MAX_CARS>::getAsciiView() const
{
   float img_left = agents_pos_x_[0];
   float img_top = agents_pos_y_[0];
   float img_right = agents_pos_x_[0] + CAR_LENGTH;
   float img_bottom = agents_pos_y_[0] + CAR_WIDTH;

   for (int i = 1; i < num_cars_; i++) {
      img_left = std::min(img_left, agents_pos_x_[i]);
      img_top = std::min(img_top, agents_pos_y_[i]);
      img_right = std::max(img_right, agents_pos_x_[i] + CAR_LENGTH);
      img_bottom = std::max(img_bottom, agents_pos_y_[i] + CAR_WIDTH);
   }

   return std::string();
}

template<size_t MAX_CARS>
inline std::shared_ptr<FSM<Environment2D<MAX_CARS>>> Environment2D<MAX_CARS>::getEgoController() const
{
   return ego_controller_;
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::setEgoController(const std::shared_ptr<FSM<Environment2D<MAX_CARS>>> controller)
{
   ego_controller_ = controller;
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::registerEgoController(const std::shared_ptr<FSM<Environment2D<MAX_CARS>>> controller)
{
   bool is_already_registered = controller->getNumOfActions();

   if (!is_already_registered) {
      if (data_) {
         controller->registerDataPack(data_);
      } else {
         data_ = controller->getData();

         data_->associateExternalFloatArray(AGENTS_POS_X_NAME, agents_pos_x_, num_cars_);
         data_->associateExternalFloatArray(AGENTS_POS_Y_NAME, agents_pos_y_, num_cars_);
         data_->associateExternalFloatArray(AGENTS_VX_REL_NAME, agents_vx_rel_, num_cars_);
         data_->associateExternalFloatArray(AGENTS_VY_NAME, agents_vy_, num_cars_);

         data_->associateSingleValWithExternalFloat(EGO_VX_NAME, &ego_vx_);
         data_->associateSingleValWithExternalFloat(EGO_POS_Y_NAME, &ego_pos_y_);
         data_->associateSingleValWithExternalChar(EGO_LEAD_CAR_NAME, &(agents_lead_cars_[num_cars_]));

         data_->associateSingleValWithExternalFloat(EGO_AX_NAME, &ego_ax_); // Actuator.
         data_->associateSingleValWithExternalFloat(EGO_VY_NAME, &ego_vy_); // Actuator.
      }

      controller->addAction(this, &Environment2D::idleCommand, "IDLE", IDLE_ID);
      controller->addAction(this, &Environment2D::laneChangeEgoLeft, "LC_L", LCL_ID);
      controller->addAction(this, &Environment2D::laneChangeEgoRight, "LC_R", LCR_ID);
      //controller->addAction(this, &Environment2D::resetEgoAcceleration, "ACC0", ACC0_ID);
      //controller->addAction(this, &Environment2D::incrementEgoAcceleration, "ACC+", ACCPLUS_ID);
      //controller->addAction(this, &Environment2D::decrementEgoAcceleration, "ACC-", ACCMINUS_ID);

   }
      
   setEgoController(controller);

   assert(ego_controller_->getData() == data_); // Make sure that the controller has not been registered for a different environment.
}

template<size_t MAX_CARS>
inline float Environment2D<MAX_CARS>::getEgoFitness() const
{
   return ego_fitness_;
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::resetEgoAcceleration(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName)
{
   ego_ax_ = 0;
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::incrementEgoAcceleration(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName)
{
   ego_ax_ = std::min(ego_ax_ + EGO_ACCELERATION_STEP, EGO_ACCELERATION_MAX);
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::decrementEgoAcceleration(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName)
{
   ego_ax_ = std::max(ego_ax_ - EGO_ACCELERATION_STEP, -EGO_ACCELERATION_MAX);
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::laneChangeEgoLeft(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName)
{
   ego_vy_ = -LANE_CHANGE_SPEED;
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::laneChangeEgoRight(const std::shared_ptr<DataPack> data, const std::string& currentStateVarName)
{
   ego_vy_ = LANE_CHANGE_SPEED;
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::idleCommand(const std::shared_ptr<DataPack> data, const std::string & currentStateVarName)
{
   // This callback function is intended to be empty, or, at least, to effectively do nothing in terms of ego behavior.
}

template<size_t MAX_CARS>
inline std::shared_ptr<DataPack> Environment2D<MAX_CARS>::getData() const
{
   return data_;
}

template<size_t MAX_CARS>
inline bool Environment2D<MAX_CARS>::collides(const float l1, const float t1, const float l2, const float t2) const
{
   const float r1 = l1 + CAR_LENGTH;
   const float r2 = l2 + CAR_LENGTH;
   const float b1 = t1 + CAR_WIDTH / LANE_WIDTH;
   const float b2 = t2 + CAR_WIDTH / LANE_WIDTH;

   return l1 >= l2 && l1 <= r2 && t1 >= t2 && t1 <= b2 || l2 >= l1 && l2 <= r1 && t2 >= t1 && t2 <= b1;
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::collidesWithAny(const float x1, const float y1, const int ignore) const
{
   for (int i = 0; i < num_cars_; i++) {
      if ((i != ignore) && collides(x1, y1, agents_pos_x_[i], agents_pos_y_[i])) {
         return i;
      }
   }

   return -1;
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::collidesWithAny(const int id) const
{
   return collidesWithAny(agents_pos_x_[id], agents_pos_y_[id], id);
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::collidesWithAnyEgo() const
{
   int i = 0;
   const int ego_lane = egoLane();

   for (; i < num_cars_ && agents_pos_x_[i] >= 0 && agents_pos_x_[i] <= CAR_LENGTH; i++) {
      if (lane(i) == ego_lane) {
         return i;
      }
   }

   for (int j = num_cars_ - 1; j >= i && agents_pos_x_[j] < 0 && -agents_pos_x_[j] <= CAR_LENGTH; j--) {
      if (lane(j) == ego_lane) {
         return j;
      }
   }

   return -1;
}

template<size_t MAX_CARS> 
template<class T>
inline void Environment2D<MAX_CARS>::swap(T* arr, const int index1, const int index2)
{
   assert(index1 < num_cars_);
   assert(index2 < num_cars_);

   arr[index1] = arr[index1] + arr[index2];
   arr[index2] = arr[index1] - arr[index2];
   arr[index1] = arr[index1] - arr[index2];
   
   //float temp = arr[index1];
   //arr[index1] = arr[index2];
   //arr[index2] = temp;

   //std::swap(arr[index1], arr[index2]);
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::swapAll(const int index1, const int index2)
{
   swap(agents_pos_x_, index1, index2);
   swap(agents_pos_y_, index1, index2);
   swap(agents_ax_, index1, index2);
   swap(agents_vx_rel_, index1, index2);
   swap(agents_vy_, index1, index2);
}

template<size_t MAX_CARS>
inline void Environment2D<MAX_CARS>::bubbleSort()
{
   bool swap_occurred = true;
   bool swapped_any = false;

   while (swap_occurred) {
      swap_occurred = false;

      for (int i = 1; i < num_cars_; i++) {
         const float p1 = agents_pos_x_[i - 1];
         const float p2 = agents_pos_x_[i];

         if ((p1 < 0 && p2 > 0) || ((p1 > 0 == p2 > 0) && p1 > p2)) {
            // Swaps: (4, 3) ==> (3, 4); (-4, 3) ==> (3, -4); (-5, -10) ==> (-10, -5) [NEG > POS]

            swapAll(i - 1, i);
            swap_occurred = true;
            swapped_any = true;
         }
      }
   }

   if (swapped_any) {
      needs_recompute_lead_cars_ = true;
   }
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::getLeadCar(const int i)
{
   if (needs_recompute_lead_cars_) {
      for (int j = 0; j < num_cars_ + 1; j++) {
         agents_lead_cars_[j] = recomputeLeadCarFor(j);
      }

      needs_recompute_lead_cars_ = false;
   }

   return agents_lead_cars_[i];
}

template<size_t MAX_CARS>
inline int Environment2D<MAX_CARS>::recomputeLeadCarFor(const int i) const
{
   if (i == num_cars_) { // Compute lead car for EGO.
      const int ego_lane = egoLane();

      for (int j = 0; j < num_cars_; j++) {
         if (lane(j) == ego_lane) {
            return j;
         }
      }
   }
   else { // Compute lead car for agent.
      const int lan = lane(i);
      const bool on_ego_lan = egoLane() == lan;
      const float pos = agents_pos_x_[i];
      const float dist_to_ego = pos > 0 ? std::numeric_limits<float>::infinity() : -pos;

      for (int j = (i + 1) % num_cars_; j != i; j = (j + 1) % num_cars_) {
         if (lane(j) == lan) {
            if (on_ego_lan && dist_to_ego < agents_pos_x_[j] - pos) {
               return num_cars_; // EGO is lead car.
            }
            else {
               return j; // j is lead car.
            }
         }
      }

      if (on_ego_lan) {
         return num_cars_;
      }
   }

   return -1; // No lead car.
}

} // vfm
