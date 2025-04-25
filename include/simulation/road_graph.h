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

struct Color;
struct HighwayTranslator;

/// <summary>
/// Provides information on how to visually connect roads coming together at a junction.
/// 
///       __________|     |__________
///            
///  source -> drain   ?   drain <- source
///       __________       __________
///                 |     |
/// 
/// Idea: On two connected sections A, B with a junction in between, draw for each matching id_ a polygon colored col_, consisting of:
///    * a bezier line from A.outgoing_.base_point_ in A.outgoing_.direction 
///                      to B.incoming_.base_point_ in B.incoming_.direction,
///    * a line from B.incoming_.base_point_ to B.outgoing_.base_point_,
///    * a bezier line from B.outgoing_.base_point_ in B.outgoing_.direction 
///                      to A.incoming_.base_point_ in A.incoming_.direction, and
///    * a line from A.incoming_.base_point_ to A.outgoing_.base_point_.
/// 
/// For example, the lane marker lines, ending at the border of the section, could be continued
/// to connect smoothly with the respective markers on the other side.
/// 
/// For this, HighwayImage::paintStraightRoadScene needs to provide a ConnectorPolygonEnding
/// for each side of the road section painted.
/// </summary>
struct ConnectorPolygonEnding
{
   enum class Side { source, drain, invalid };

   Side side_{ Side::invalid };
   vfm::Lin2D connector_{ {0, 0}, {0, 0} };
   float thick_{};
   std::shared_ptr<Color> col_{ nullptr };
   int id_{ -1 };
   std::shared_ptr<HighwayTranslator> my_trans_{};
};

struct CarPars {
   inline CarPars() : CarPars(0.0f, 0.0f, 0, -1) {}

   inline CarPars(const float car_lane, const float car_rel_pos, const int car_velocity, const int car_id)
      : car_lane_{ car_lane }, car_rel_pos_{ car_rel_pos }, car_velocity_{ car_velocity }, car_id_{ car_id }
   {}

   float car_lane_{};
   float car_rel_pos_{};
   int car_velocity_{};
   int car_id_{};

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
   int getNumLanes() const;

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
/// begin1 = 0   begin2            begin_n    section_end_  
/// ------------------------------------------
///   SEG1       |  SEG2  | ... |  SEG n     |
/// ------------------------------------------
/// </summary>
class StraightRoadSection : public Failable {
public:
   StraightRoadSection(); // Constructs an invalid lane structure.
   StraightRoadSection(const int lane_num, const float section_end);
   StraightRoadSection(const int lane_num, const float section_end, const std::vector<LaneSegment>& segments);

   void addLaneSegment(const LaneSegment& segment);
   void cleanUp(bool add_note);
   std::string toString() const;
   void setNumLanes(const int num_lanes) const;
   int getNumLanes() const;
   bool isValid() const;
   std::map<float, LaneSegment> getSegments() const;

   void setEgo(const std::shared_ptr<CarPars> ego);
   void setOthers(const CarParsVec& others);
   void setFuturePositionsOfOthers(const std::map<int, std::pair<float, float>>& future_positions_of_others);
   void setSectionEnd(const float end);
   std::shared_ptr<CarPars> getEgo() const;
   CarParsVec getOthers() const;
   std::map<int, std::pair<float, float>> getFuturePositionsOfOthers() const;
   float getLength() const;

private:
   std::map<float, LaneSegment> segments_{};
   mutable int num_lanes_{ -1 }; // We have lane ids: 0 .. (num_lanes_ - 1) * 2
   std::shared_ptr<CarPars> ego_{}; // Ego may or may not (nullptr) be within this road section.
   CarParsVec others_{};
   std::map<int, std::pair<float, float>> future_positions_of_others_{};

public: // TODO
   float section_end_{ -1 };
};

/// <summary>
/// Connected, possibly circular graph of StraightRoadSection's.
/// </summary>
class RoadGraph : public Failable, public std::enable_shared_from_this<RoadGraph>{
public:
   RoadGraph(const int id);

   std::shared_ptr<RoadGraph> findFirstSectionWithProperty(const std::function<bool(const std::shared_ptr<RoadGraph>)> property);
   std::shared_ptr<RoadGraph> findSectionWithID(const int id);
   std::shared_ptr<RoadGraph> findSectionWithCar(const int car_id);
   std::shared_ptr<RoadGraph> findSectionWithEgo();
   void applyToMeAndAllMySuccessorsAndPredecessors(const std::function<void(const std::shared_ptr<RoadGraph>)> action);

   StraightRoadSection& getMyRoad();
   Vec2D getOriginPoint() const;
   Vec2D getDrainPoint() const;
   float getAngle() const;

   /// <summary>
   /// Checks if THIS SECTION is rooted in the origin (0/0) and has a zero angle (s.t. small epsilon).
   /// </summary>
   /// <returns></returns>
   bool isRootedInZeroAndUnturned() const;
   
   /// <summary>
   /// Translates and rotates THE WHOLE GRAPH such that the section with ego on it "isRootedInZeroAndUnturned()".
   /// </summary>
   void normalizeRoadGraphToEgoSection();

   int getID() const;
   int getNumberOfNodes() const;

   void setMyRoad(const StraightRoadSection& section);
   void setOriginPoint(const Vec2D& point);
   void setAngle(const float angle);

   void addSuccessor(const std::shared_ptr<RoadGraph> subgraph);
   void addPredecessor(const std::shared_ptr<RoadGraph> subgraph);

   Rec2D getBoundingBox() const;

   std::vector<std::shared_ptr<RoadGraph>> getSuccessors() const;
   std::vector<std::shared_ptr<RoadGraph>> getPredecessors() const;

   std::vector<std::shared_ptr<RoadGraph>> getAllNodes() const;

   std::vector<ConnectorPolygonEnding> connectors_{};

   StraightRoadSection my_road_{}; // TODO: Make private.
private:
   Vec2D origin_point_{ 0.0F, 0.0F };
   float angle_{ 0 }; // In RAD
   int id_{};

   std::vector<std::shared_ptr<RoadGraph>> successors_{};
   std::vector<std::shared_ptr<RoadGraph>> predecessors_{};

   std::shared_ptr<RoadGraph> findFirstSectionWithProperty(
      const std::function<bool(std::shared_ptr<RoadGraph>)> property,
      std::set<std::shared_ptr<RoadGraph>>& visited);

   bool isOriginAtZero() const;
   bool isAngleZero() const;
};

} // vfm