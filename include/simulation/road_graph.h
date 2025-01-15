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
struct CarPars {
   float car_lane_{};
   float car_rel_pos_{};
   int car_velocity_{};

   inline bool operator<(const CarPars& other) const
   {
      std::cout << "Something strange was requested. Look into CarPars struct. (Probably you put CarPars into a map, but after refactoring there is no default < operation anymore. Please fix, future Lukas!)" << std::endl;
      std::exit(1);
      return false;
   }

   inline bool operator==(const CarPars& other) const
   {
      return car_lane_ == other.car_lane_
         && car_rel_pos_ == other.car_rel_pos_
         && car_velocity_ == other.car_velocity_;
   }
};

using CarParsVec = std::vector<CarPars>;

/// <summary>
/// Represents one straight road segment with a minimum lane id and a maximum lane id.
/// It is embedded into a StraightRoadSection which defines the number of lanes available.
/// Also, the begin_ member implicitly defines a length when within a sequence of LaneSegment's.
/// </summary>
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

/// <summary>
/// Sequence of LaneSegment's.
/// </summary>
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

   void setEgo(const std::shared_ptr<CarPars> ego);
   void setOthers(const CarParsVec& others);
   void setFuturePositionsOfOthers(const std::map<int, std::pair<float, float>>& future_positions_of_others);

private:
   std::map<float, LaneSegment> segments_{};
   mutable int num_lanes_{ -1 }; // We have lane ids: 0 .. (num_lanes_ - 1) * 2
   std::shared_ptr<CarPars> ego_{}; // Ego may or may not (nullptr) be within this road section.
   CarParsVec others_{};
   std::map<int, std::pair<float, float>> future_positions_of_others_{};
};

/// <summary>
/// Connected, possibly circular graph of StraightRoadSection's.
/// </summary>
class RoadGraph : public Failable, public std::enable_shared_from_this<RoadGraph>{
public:
   RoadGraph(const int id);
   std::shared_ptr<RoadGraph> findSectionWithID(const int id);

   StraightRoadSection getMyRoad() const;
   Vec2D getOriginPoint() const;
   float getAngle() const;
   int getID() const;

   void setMyRoad(const StraightRoadSection& section);
   void setOriginPoint(const Vec2D& point);
   void setAngle(const float angle);

   void addSuccessor(const std::shared_ptr<RoadGraph> subgraph);
   void addPredecessor(const std::shared_ptr<RoadGraph> subgraph);

private:
   StraightRoadSection my_road_{};
   Vec2D origin_point_{ 0.0F, 0.0F };
   float angle_{ 0 };
   int id_{};

   std::set<std::shared_ptr<RoadGraph>> successors_{};
   std::set<std::shared_ptr<RoadGraph>> predecessors_{};

   std::shared_ptr<RoadGraph> findSectionWithID(const int id, std::set<std::shared_ptr<RoadGraph>>& visited);
};

} // vfm