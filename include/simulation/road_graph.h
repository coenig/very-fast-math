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

class Way {
public:
   inline Way() :
      relation_id_{ global_id_first_free++ },
      road_link_id_{ global_id_first_free++ },
      left_border_id_{ global_id_first_free++ },
      right_border_id_{ global_id_first_free++ },
      origin_node_left_id_{ global_id_first_free++ },
      target_node_left_id_{ global_id_first_free++ },
      origin_node_right_id_{ global_id_first_free++ },
      target_node_right_id_{ global_id_first_free++ } {
   }

   inline std::shared_ptr<xml::CodeXML> getNodesXML() const
   {
      Vec2D dir{ target_ - origin_ };
      dir.ortho();
      dir.setLength(3.75 / 2);

      Vec2D origin_left{ origin_ - dir };
      Vec2D origin_right{ origin_ + dir };
      Vec2D target_left{ target_ - dir };
      Vec2D target_right{ target_ + dir };
      Vec2D mid_left{ (origin_left + target_left) / 2 - dir };
      Vec2D mid_right{ (origin_right + target_right) / 2 - dir };

      auto origin_left_coord = StaticHelper::cartesianToWGS84(origin_left.x, origin_left.y);
      auto target_left_coord = StaticHelper::cartesianToWGS84(target_left.x, target_left.y);
      auto origin_right_coord = StaticHelper::cartesianToWGS84(origin_right.x, origin_right.y);
      auto target_right_coord = StaticHelper::cartesianToWGS84(target_right.x, target_right.y);
      auto mid_left_coord = StaticHelper::cartesianToWGS84(mid_left.x, mid_left.y);
      auto mid_right_coord = StaticHelper::cartesianToWGS84(mid_right.x, mid_right.y);
      auto res =
         getNodeXML(origin_node_left_id_, origin_left_coord.first, origin_left_coord.second);
      res->appendAtTheEnd(getNodeXML(target_node_left_id_, target_left_coord.first, target_left_coord.second));
      res->appendAtTheEnd(getNodeXML(origin_node_right_id_, origin_right_coord.first, origin_right_coord.second));
      res->appendAtTheEnd(getNodeXML(target_node_right_id_, target_right_coord.first, target_right_coord.second));
      res->appendAtTheEnd(getNodeXML(target_node_left_id_ + 1000, mid_left_coord.first, mid_left_coord.second));
      res->appendAtTheEnd(getNodeXML(target_node_right_id_ + 1000, mid_right_coord.first, mid_right_coord.second));

      return res;
   }

   //<nd ref = '-25494' / >
//   <nd ref = '-25495' / >
//   <tag k = "Painting::LineWidth" v = "0.300000" / >
//   <tag k = "Painting::MarkingType" v = "Solid" / >
//   <tag k = "Painting::SingleLineOffsets" v = "-0.000000" / >
//   <tag k = "Painting::SingleLineWidths" v = "0.300000" / >
//   <tag k = "color" v = "white" / >
//   <tag k = "subtype" v = "solid" / >
//   <tag k = "type" v = "line_thick" / >
//   <tag k = "width" v = "0.300000" / >
// 
//   <tag k = 'type' v = 'road_link' / >
//   <tag k = "rl:meta:country_code" v = "276" / >
//   <tag k = "rl:meta:is_controlled" v = "false" / >
//   <tag k = "rl:meta:is_motorway" v = "false" / >
//   <tag k = "rl:meta:is_separated_structurally" v = "false" / >
//   <tag k = "rl:meta:is_urban" v = "false" / >
//   <tag k = "rl:predecessors" v = "" / >
//   <tag k = "rl:successors" v = "" / >
//   <tag k = "rl:travel_direction" v = "along" / >
//   <tag k = "rl:type" v = "no_special" / >

   inline std::shared_ptr<xml::CodeXML> getWayXML() const
   {
      auto dummy = xml::CodeXML::emptyXML();
      auto way_inner_left = xml::CodeXML::emptyXML();
      auto way_inner_right = xml::CodeXML::emptyXML();
      auto way_inner_link = xml::CodeXML::emptyXML();

      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "Painting::LineWidth"}, {"v", "0.300000"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "Painting::LineWidth"}, {"v", "0.300000"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "Painting::MarkingType"}, {"v", "Solid"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "Painting::SingleLineOffsets"}, {"v", "-0.000000"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "Painting::SingleLineWidths"}, {"v", "0.300000"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "color"}, {"v", "white"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "subtype"}, {"v", "solid"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "type"}, {"v", "line_thick"} }));
      dummy->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "width"}, {"v", "0.300000"} }));

      way_inner_left->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(origin_node_left_id_) } }));
      way_inner_left->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(target_node_left_id_ + 1000) } }));
      way_inner_left->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(target_node_left_id_) } }));
      way_inner_right->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(origin_node_right_id_) } }));
      way_inner_right->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(target_node_right_id_ + 1000) } }));
      way_inner_right->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(target_node_right_id_) } }));

      way_inner_left->appendAtTheEnd(dummy);
      way_inner_right->appendAtTheEnd(dummy);

      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(origin_node_left_id_) } }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(target_node_left_id_ + 1000) } }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(target_node_left_id_) } }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "type"}, {"v", "road_link"} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:meta:country_code"}, {"v", "276"} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:meta:is_controlled"}, {"v", "false"} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:meta:is_motorway"}, {"v", "false"} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:meta:is_separated_structurally"}, {"v", "false"} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:meta:is_urban"}, {"v", "false"} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:predecessors"}, {"v", ""} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:successors"}, {"v", ""} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:travel_direction"}, {"v", "along"} }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { {"k", "rl:type"}, {"v", "no_special"} }));

      auto relation_inner = xml::CodeXML::emptyXML();
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("member", { { "type","way" }, { "ref", std::to_string(left_border_id_) }, { "role", "left" }}));
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("member", { { "type","way" }, { "ref", std::to_string(right_border_id_) }, { "role", "right" }}));
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","location" }, { "v", "nonurban" }}));
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","rl:direction" }, { "v", "along" }}));
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","rl:id" }, { "v", std::to_string(road_link_id_) }}));
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","speed_limit" }, { "v", "130.000000" }}));
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","subtype" }, { "v", "highway" }}));
      relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","type" }, { "v", "lanelet" }}));

      //   <relation id = "1019" visible = "true" version = "1">
      //   <member type = "way" ref = "-694" role = "left" / >
      //   <member type = "way" ref = "-715" role = "right" / >
      //   <tag k = "location" v = "nonurban" / >
      //   <tag k = "rl:direction" v = "along" / >
      //   <tag k = "rl:id" v = "-71511" / >
      //   <tag k = "speed_limit" v = "130.000000" / >
      //   <tag k = "subtype" v = "highway" / >
      //   <tag k = "type" v = "lanelet" / >
      //   < / relation>


      auto xml = xml::CodeXML::retrieveElementWithXMLContent("way", { {"id", std::to_string(left_border_id_)},{"visible", "true"}, {"version", "1"} }, way_inner_left);
      xml->appendAtTheEnd(xml::CodeXML::retrieveElementWithXMLContent("way", { {"id", std::to_string(right_border_id_)},{"visible", "true"}, {"version", "1"} }, way_inner_right));
      xml->appendAtTheEnd(xml::CodeXML::retrieveElementWithXMLContent("way", { {"id", std::to_string(road_link_id_)},{"visible", "true"}, {"version", "1"} }, way_inner_link));
      xml->appendAtTheEnd(xml::CodeXML::retrieveElementWithXMLContent("relation", { { "id", std::to_string(relation_id_) }, {"visible", "true"}, { "version", "1" } }, relation_inner));

      return xml;
   }

   int relation_id_{};
   int road_link_id_{};
   int left_border_id_{};
   int right_border_id_{};
   int origin_node_left_id_{};
   int target_node_left_id_{};
   int origin_node_right_id_{};
   int target_node_right_id_{};
   Vec2D origin_{};
   Vec2D target_{};
   std::set<int> predecessor_ids_{};
   std::set<int> successor_ids_{};
   int left_way_id_{};
   int right_way_id_{};

private:
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

   //inline void addWaysFromPredecessor(const LaneSegment& predecessor, const Vec2D& origin) {
   //   for (int i = getMinLane(); i <= getMaxLane(); i++) {
   //      Way way{};
   //      ways_.push_back(way);
   //   }
   //}

   //inline void clearWays() 
   //{
   //   ways_.clear();
   //}

   //inline void addWay(const Way& way) 
   //{
   //   ways_.push_back(way);
   //}

   inline std::map<int, Way>& getWays()
   {
      return ways_;
   }

   std::string toString() const;

private:
   float begin_{};
   const int min_lane_{}; // 0: right-most, 1: right-most as emergency, 2: right-most - 1 etc.
   const int max_lane_{};
   std::map<int, Way> ways_{};
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
};

} // vfm