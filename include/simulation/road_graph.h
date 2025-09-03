//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "geometry/vector_2d.h"
#include "geometry/polygon_2d.h"
#include "xml/xml_generator.h"
#include "failable.h"
#include "static_helper.h"
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


static int global_id_first_free{ 0 };

class RoadGraph;

struct WayIDs {
   inline WayIDs() : 
      relation_id_{ global_id_first_free++ },
      road_link_id_{ global_id_first_free++ },
      left_border_id_{ global_id_first_free++ },
      right_border_id_{ global_id_first_free++ }
   {}

   int relation_id_{};
   int road_link_id_{};
   int left_border_id_{};
   int right_border_id_{};
};

class Way : public Failable {
public:
   inline Way(const std::shared_ptr<RoadGraph> my_father_rg);
   std::pair<std::shared_ptr<xml::CodeXML>, std::shared_ptr<xml::CodeXML>> getXML() const;

private:
   WayIDs way_ids_{};
   int origin_node_left_id_{};
   int target_node_left_id_{};
   int origin_node_right_id_{};
   int target_node_right_id_{};
   mutable std::vector<std::vector<int>> ids_to_connector_successor_nodes_left_{};
   mutable std::vector<std::vector<int>> ids_to_connector_successor_nodes_right_{};
   mutable std::vector<WayIDs> ids_for_connector_successor_ways_{};
   std::shared_ptr<RoadGraph> my_road_graph_{};

   static std::shared_ptr<xml::CodeXML> getWayXMLGeneric(
      WayIDs way_ids,
      const std::vector<int> left_ids,
      const std::vector<int> right_ids
   );

   std::shared_ptr<xml::CodeXML> getNodesXML() const;
   std::shared_ptr<xml::CodeXML> getWayXML() const;

   inline static std::shared_ptr<xml::CodeXML> getNodeXML(const int id, const double lat, const double lon)
   {
      return xml::CodeXML::retrieveParsOnlyElement("node", {
         { "id", std::to_string(id) },
         { "visible", "true" },
         { "version", "1" },
         { "lat", std::to_string(lat) },
         { "lon", std::to_string(lon) }
         });
   }
};

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
/// begin1 = 0   begin2            begin_n                       section_end_  
/// ------------------------------------------
///   SEG1       |  SEG2  | ... |  SEG n     |
/// -------------------------------------------------------------
///   SEG1       |  SEG2  | ... |  SEG n     | ... |  SEG m     |
/// -------------------------------------------------------------
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
   std::map<float, LaneSegment>& getSegmentsRef();

   void setEgo(const std::shared_ptr<CarPars> ego);
   void setOthers(const CarParsVec& others);
   void addOther(const CarPars& other);
   void setFuturePositionsOfOthers(const std::map<int, std::pair<float, float>>& future_positions_of_others);
   void setSectionEnd(const float end);
   std::shared_ptr<CarPars> getEgo() const;
   CarParsVec getOthers() const;
   std::map<int, std::pair<float, float>> getFuturePositionsOfOthers() const;
   float getLength() const;
   std::map<int, Way> addWaysToSegment(const float segment_id, const std::shared_ptr<RoadGraph> father_rg);

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
   constexpr static int EGO_MOCK_ID{ -100 };

   RoadGraph(const int id);

   std::shared_ptr<RoadGraph> findFirstSectionWithProperty(const std::function<bool(const std::shared_ptr<RoadGraph>)> property);
   std::shared_ptr<RoadGraph> findSectionWithID(const int id);
   std::shared_ptr<RoadGraph> findSectionWithCar(const int car_id);

   struct CarLocation {
      std::shared_ptr<RoadGraph> on_section_or_origin_section_{}; // (Empty if no ego in graph, or) ON SECTION iff...
      std::shared_ptr<RoadGraph> optional_target_section_{};      // ...optional_target_section is empty, otherwise between sections.
      std::shared_ptr<CarPars> the_car_{};
   };

   /// <summary>
   /// Finds ego only if it is on a section.
   /// </summary>
   /// <returns>The section that contains ego, or null.</returns>
   std::shared_ptr<RoadGraph> findSectionWithEgoIfAny() const;

   /// <summary>
   /// Finds ego, wherever it may be.
   /// </summary>
   /// <returns>The location of ego in the graph. Origin and target section are empty if there is no ego in the graph.</returns>
   CarLocation findEgo() const;

   void applyToMeAndAllMySuccessorsAndPredecessors(const std::function<void(const std::shared_ptr<RoadGraph>)> action);

   void cleanFromAllCars();
   void removeEgo();
   void removeNonegoCars();

   StraightRoadSection& getMyRoad();
   Vec2D getOriginPoint() const;
   Vec2D getDrainPoint() const;
   float getAngle() const;

   /// <summary>
   /// Checks if THIS SECTION is rooted in the origin (0/0) and has a zero angle (s.t. small epsilon).
   /// </summary>
   /// <returns></returns>
   bool isRootedInZeroAndUnturned() const;

   std::pair<Vec2D, float> getEgoOriginAndRotation();
   
   /// <summary>
   /// Translates and rotates THE WHOLE GRAPH such that the section with ego on it "isRootedInZeroAndUnturned()".
   /// </summary>
   void normalizeRoadGraphToEgoSection();

   void transformAllCarsToStraightRoadSections();

   int getID() const;
   int getNodeCount() const;

   void setMyRoad(const StraightRoadSection& section);
   void setOriginPoint(const Vec2D& point);
   void setAngle(const float angle);

   void addSuccessor(const std::shared_ptr<RoadGraph> subgraph);
   void addPredecessor(const std::shared_ptr<RoadGraph> subgraph);

   Rec2D getBoundingBox() const;

   std::vector<std::shared_ptr<RoadGraph>> getSuccessors() const;
   std::vector<std::shared_ptr<RoadGraph>> getPredecessors() const;

   std::vector<std::shared_ptr<RoadGraph>> getAllNodes() const;

   std::vector<CarPars> getNonegosOnCrossingTowardsSuccessor(const std::shared_ptr<RoadGraph> r) const;
   std::map<std::shared_ptr<RoadGraph>, std::vector<CarPars>> getNonegosOnCrossingTowardsAllSuccessors() const;
   void addNonegoOnCrossingTowards(const std::shared_ptr<RoadGraph> r, const CarPars& nonego);
   void addNonegoOnCrossingTowards(const int r_id, const CarPars& nonego);
   void clearNonegosOnCrossingsTowardsAny();

   void makeGhost();
   bool isGhost() const;

   std::shared_ptr<xml::CodeXML> generateOSM() const;

   std::vector<ConnectorPolygonEnding> connectors_{}; // TODO: Make private.
   StraightRoadSection my_road_{};                    // TODO: Make private.

private:
   std::shared_ptr<RoadGraph> findFirstSectionWithProperty(
      const std::function<bool(std::shared_ptr<RoadGraph>)> property,
      std::set<std::shared_ptr<RoadGraph>>& visited);

   bool isOriginAtZero() const;
   bool isAngleZero() const;

   Vec2D origin_point_{ 0.0F, 0.0F };
   float angle_{ 0 }; // In RAD
   int id_{};

   std::vector<std::shared_ptr<RoadGraph>> successors_{};
   std::vector<std::shared_ptr<RoadGraph>> predecessors_{};

   std::map<std::shared_ptr<RoadGraph>, std::vector<CarPars>> nonegos_towards_successors_{};

   bool ghost_section_{ false }; // If true, the road is not painted nor its connections to others; only the cars are painted.
};

} // vfm