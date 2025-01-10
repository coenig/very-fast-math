//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "geometry/vector_2d.h"
#include "geometry/polygon_2d.h"
#include "failable.h"
#include <string>

namespace vfm {

class LaneSegment {
public:
   LaneSegment(const float begin, const int min_lane, const int max_lane);

   float getBegin() const;
   int getMinLane() const;
   int getMaxLane() const;

   int getActualDrivableMinLane() const;
   int getActualDrivableMaxLane() const;

   bool isMinLaneShoulder() const;
   bool isMaxLaneShoulder() const;

   void screwUpBegin();
   bool isScrewedUp();

   std::string toString() const;

private:
   float begin_{};
   const int min_lane_{}; // 0: right-most, 1: right-most as emergency, 2: right-most - 1 etc.
   const int max_lane_{};
};

static constexpr int MIN_DISTANCE_BETWEEN_SEGMENTS{ 20 };

class StraightRoadSection : public Failable {
public:
   StraightRoadSection(); // Constructs an invalid lane structure.
   StraightRoadSection(const int lane_num);
   StraightRoadSection(const int lane_num, const std::vector<LaneSegment>& segments);

   void addLaneSegment(const LaneSegment& segment);
   void cleanUp();
   std::string toString() const;
   void setNumLanes(const int num_lanes) const;
   int getNumLanes() const;
   bool isValid() const;
   std::map<float, LaneSegment> getSegments() const;

private:
   std::map<float, LaneSegment> segments_{};
   mutable int num_lanes_{ -1 }; // We have lane ids: 0 .. (num_lanes_ - 1) * 2
};

class RoadGraph : public Failable {
   RoadGraph();

private:
   StraightRoadSection my_road_{};
   Vec2Df point_origin_{ 0.0F, 0.0F };
   float angle_{ 0 };

   std::vector<std::unique_ptr<StraightRoadSection>> successors_{};
   std::vector<std::unique_ptr<StraightRoadSection>> predecessors_{};
};

} // vfm