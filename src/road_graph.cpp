#include "simulation/road_graph.h"
#include "geometry/images.h"
#include "geometry/bezier_functions.h"
#include <vector>
#include <cmath>


using namespace vfm;

LaneSegment::LaneSegment(const float begin, const int min_lane, const int max_lane)
   : begin_(begin), min_lane_(min_lane), max_lane_(max_lane)
{}

float LaneSegment::getBegin() const { return begin_; }
int LaneSegment::getMinLane() const { return min_lane_; }
int LaneSegment::getMaxLane() const { return max_lane_; }

int LaneSegment::getActualDrivableMinLane() const { return getMinLane() / 2; }
int LaneSegment::getActualDrivableMaxLane() const { return (getMaxLane() + 1) / 2; }

int LaneSegment::getNumLanes() const { return (getActualDrivableMaxLane() - getActualDrivableMinLane() + 1); }

bool LaneSegment::isMinLaneShoulder() const { return getMinLane() % 2; }
bool LaneSegment::isMaxLaneShoulder() const { return getMaxLane() % 2; }

void LaneSegment::screwUpBegin()
{
   begin_ = std::numeric_limits<float>::quiet_NaN();
}

bool LaneSegment::isScrewedUp()
{
   return std::isnan(begin_);
}

std::string LaneSegment::toString() const
{
   return "<" + std::to_string(getBegin()) + ", [" + std::to_string(getMinLane()) + " -- " + std::to_string(getMaxLane()) + "]>";
}

StraightRoadSection::StraightRoadSection() : StraightRoadSection(-1, -1) {} // Constructs an invalid lane structure.

StraightRoadSection::StraightRoadSection(const int lane_num, const float section_end)
   : StraightRoadSection(lane_num, section_end, std::vector<LaneSegment>{}) {}

StraightRoadSection::StraightRoadSection(const int lane_num, const float section_end, const std::vector<LaneSegment>& segments)
   : num_lanes_(lane_num), section_end_(section_end), Failable("StraightRoadSection") {
   for (const auto& segment : segments) {
      addLaneSegment(segment);
   }
}

void StraightRoadSection::addLaneSegment(const LaneSegment& segment)
{
   if (!segments_.insert({ segment.getBegin(), segment }).second) {
      addError("Segment " + segment.toString() + " cannot override " + segments_.at(segment.getBegin()).toString() + " already present in StraightRoadSection.");
   }
}

void StraightRoadSection::cleanUp(bool add_note)
{
   constexpr int overall_min_lane = 0;
   const     int overall_max_lane = (num_lanes_ - 1) * 2;
   int last_segment_min{ -1 };
   int last_segment_max{ -1 };
   bool change_occurred{};

   if (segments_.empty()) {
      addLaneSegment({ 0, overall_min_lane, overall_max_lane }); // First segment counts until negative, last until positive infinity.
      change_occurred = true;
   }

   for (auto& segment : segments_) {
      if (segment.second.getMaxLane() > overall_max_lane || segment.second.getMinLane() < overall_min_lane) {
         addFatalError("Invalid segment " + segment.second.toString() + " for lane number " + std::to_string(num_lanes_) + ". Skipping this segment.");
         segment.second.screwUpBegin();
         change_occurred = true;
      }
      else {
         if (segment.second.getMaxLane() == last_segment_max && segment.second.getMinLane() == last_segment_min) {
            segment.second.screwUpBegin();
            addDebug("Ignoring dispensable segment " + segment.second.toString() + ".");
         }
         else {
            addDebug("Keeping              segment " + segment.second.toString() + ".");
            last_segment_max = segment.second.getMaxLane();
            last_segment_min = segment.second.getMinLane();
         }
      }
   }

   // Delete invalid items.
   auto iter = segments_.begin();
   while (iter != segments_.end()) {
      if (iter->second.isScrewedUp()) {
         iter = segments_.erase(iter);
         change_occurred = true;
      }
      else {
         ++iter;
      }
   }

   auto last_segment{ segments_.begin() };
   for (auto this_segment{ ++segments_.begin() }; this_segment != segments_.end(); this_segment++) {

      if (this_segment->second.getBegin() - last_segment->second.getBegin()
         < MIN_DISTANCE_BETWEEN_SEGMENTS
         //* std::min(std::abs(this_segment->second.getActualDrivableMinLane() - last_segment->second.getActualDrivableMinLane()),
         //           std::abs(this_segment->second.getActualDrivableMaxLane() - last_segment->second.getActualDrivableMaxLane()))
         ) {
         addError("Segments '" + last_segment->second.toString() + "' and '" + this_segment->second.toString() + "' are too close to each other (< " + std::to_string(MIN_DISTANCE_BETWEEN_SEGMENTS) + ").");
      }

      last_segment = this_segment;
   }

   if (change_occurred && add_note) {
      addNotePlain("");
      addNote("Cleaned up lane segments: " + toString());
      //std::cin.get();
   }
}

std::string StraightRoadSection::toString() const
{
   std::string s{ "{" + std::to_string(num_lanes_) + ", " };
   std::string comma{};

   for (const auto& segment : segments_) {
      s += comma + segment.second.toString();
      comma = ", ";
   }

   return s + "}";
}

void StraightRoadSection::setNumLanes(const int num_lanes) const
{
   num_lanes_ = num_lanes;
}

int StraightRoadSection::getNumLanes() const
{
   return num_lanes_;
}

bool StraightRoadSection::isValid() const
{
   return getNumLanes() >= 0;
}

std::map<float, LaneSegment> StraightRoadSection::getSegments() const
{
   return segments_;
}

std::map<float, LaneSegment>& StraightRoadSection::getSegmentsRef()
{
   return segments_;
}

void vfm::StraightRoadSection::setEgo(const std::shared_ptr<CarPars> ego)
{
   ego_ = ego;
}

void vfm::StraightRoadSection::setOthers(const CarParsVec& others)
{
   others_ = others;
}

void vfm::StraightRoadSection::addOther(const CarPars& other)
{
   others_.push_back(other);
}

void vfm::StraightRoadSection::setFuturePositionsOfOthers(const std::map<int, std::pair<float, float>>& future_positions_of_others)
{
   future_positions_of_others_ = future_positions_of_others;
}

void vfm::StraightRoadSection::setSectionEnd(const float end)
{
   section_end_ = end;
}

std::shared_ptr<CarPars> vfm::StraightRoadSection::getEgo() const
{
   return ego_;
}
CarParsVec StraightRoadSection::getOthers() const
{
   return others_;
}
std::map<int, std::pair<float, float>> StraightRoadSection::getFuturePositionsOfOthers() const
{
   return future_positions_of_others_;
}

float vfm::StraightRoadSection::getLength() const
{
   return section_end_;
}

vfm::RoadGraph::RoadGraph(const int id) : Failable("RoadGraph"), id_{id}
{}

std::shared_ptr<RoadGraph> vfm::RoadGraph::findFirstSectionWithProperty(
   const std::function<bool(const std::shared_ptr<RoadGraph>)> property,
   std::set<std::shared_ptr<RoadGraph>>& visited)
{
   if (!visited.insert(shared_from_this()).second) return nullptr;
   if (property(shared_from_this())) return shared_from_this();

   for (const auto& succ_ptr : successors_) {
      std::shared_ptr<RoadGraph> ptr{ succ_ptr->findFirstSectionWithProperty(property, visited) };
      if (ptr) return ptr;
   }

   for (const auto& pred_ptr : predecessors_) {
      std::shared_ptr<RoadGraph> ptr{ pred_ptr->findFirstSectionWithProperty(property, visited) };
      if (ptr) return ptr;
   }

   return nullptr;
}

std::shared_ptr<RoadGraph> vfm::RoadGraph::findFirstSectionWithProperty(const std::function<bool(std::shared_ptr<RoadGraph>)> property)
{
   std::set<std::shared_ptr<RoadGraph>> visited{};
   return findFirstSectionWithProperty(property, visited);
}

std::shared_ptr<RoadGraph> vfm::RoadGraph::findSectionWithID(const int id)
{
   return findFirstSectionWithProperty([id](const std::shared_ptr<RoadGraph> r) -> bool { 
      return id == r->getID(); 
   });
}

std::shared_ptr<RoadGraph> vfm::RoadGraph::findSectionWithCar(const int car_id)
{
   return findFirstSectionWithProperty([car_id](const std::shared_ptr<RoadGraph> r) -> bool
   {
      for (const auto& other : r->getMyRoad().getOthers()) {
         if (other.car_id_ == car_id) {
            return true;
         }
      }

      return false;
   });
}

std::shared_ptr<RoadGraph> vfm::RoadGraph::findSectionWithEgoIfAny() const
{
   return const_cast<RoadGraph*>(this)->findFirstSectionWithProperty(
      [](const std::shared_ptr<RoadGraph> r) -> bool { return (bool) r->getMyRoad().getEgo(); }
   );
}

RoadGraph::CarLocation vfm::RoadGraph::findEgo() const
{
   auto ego_section = findSectionWithEgoIfAny();

   if (ego_section) {
      return CarLocation{ ego_section, nullptr, ego_section->my_road_.getEgo() };
   }
   else {
      CarLocation location{ nullptr, nullptr, nullptr };

      const_cast<RoadGraph*>(this)->findFirstSectionWithProperty(
         [&location](const std::shared_ptr<RoadGraph> r) -> bool { 
            for (const auto& succ : r->getSuccessors()) {
               for (const auto& car : r->getNonegosOnCrossingTowardsSuccessor(succ)) {
                  if (car.car_id_ == EGO_MOCK_ID) {
                     location.on_section_or_origin_section_ = r;
                     location.optional_target_section_ = succ;
                     location.the_car_ = std::make_shared<CarPars>(car.car_lane_, car.car_rel_pos_, car.car_velocity_, car.car_id_);

                     return true;
                  }
               }
            }

            return false;
         }
      );

      return location;
   }
}

void vfm::RoadGraph::applyToMeAndAllMySuccessorsAndPredecessors(const std::function<void(const std::shared_ptr<RoadGraph>)> action)
{
   findFirstSectionWithProperty([&action](const std::shared_ptr<RoadGraph> r) -> bool {
      action(r);
      return false;
   });
}

void vfm::RoadGraph::cleanFromAllCars()
{
   removeEgo();
   removeNonegoCars();
}

void vfm::RoadGraph::removeEgo()
{
   findFirstSectionWithProperty([](const std::shared_ptr<RoadGraph> r) -> bool {
      r->my_road_.setEgo(nullptr);
      return false;
   });
}

void vfm::RoadGraph::removeNonegoCars()
{
   findFirstSectionWithProperty([](const std::shared_ptr<RoadGraph> r) -> bool {
      r->my_road_.setOthers({});
      r->clearNonegosOnCrossingsTowardsAny();
      return false;
   });
}

StraightRoadSection& vfm::RoadGraph::getMyRoad()
{
   return my_road_;
}

Vec2Df vfm::RoadGraph::getOriginPoint() const
{
   return origin_point_;
}

Vec2Df vfm::RoadGraph::getDrainPoint() const
{
   // section_[sec].drain.x := section_[sec].source.x + (section_[sec]_end * cos_of_section_[sec]_angle) / 100
   // section_[sec].drain.y := section_[sec].source.y + (section_[sec]_end * sin_of_section_[sec]_angle) / 100;

   return { origin_point_.x + const_cast<RoadGraph*>(this)->getMyRoad().section_end_ * std::cos(angle_), 
            origin_point_.y + const_cast<RoadGraph*>(this)->getMyRoad().section_end_ * std::sin(angle_) };
}

float vfm::RoadGraph::getAngle() const
{
   return angle_;
}

bool vfm::RoadGraph::isRootedInZeroAndUnturned() const
{
   return isOriginAtZero() && isAngleZero();
}

std::pair<Vec2D, float> vfm::RoadGraph::getEgoOriginAndRotation()
{
   const auto ego_location = findEgo();

   if (!ego_location.on_section_or_origin_section_ || !ego_location.the_car_) addError("No ego found in or between section(s) on this road graph.");

   if (!ego_location.optional_target_section_) { // Ego on section.
      return { ego_location.on_section_or_origin_section_->getOriginPoint(), ego_location.on_section_or_origin_section_->getAngle() };
   }
   else { // Ego between sections.
      auto p_orig_from = ego_location.on_section_or_origin_section_->getOriginPoint();
      auto p_orig = ego_location.on_section_or_origin_section_->getDrainPoint();
      auto p_targ = ego_location.optional_target_section_->getOriginPoint();
      auto p_targ_to = ego_location.optional_target_section_->getDrainPoint();

      auto nice_points = bezier::getNiceBetweenPoints(p_orig, p_orig - p_orig_from, p_targ, p_targ - p_targ_to);

      Vec2D dir{ bezier::B_prime(
         ego_location.the_car_->car_rel_pos_, 
         p_orig,
         nice_points[0],
         nice_points[2],
         p_targ)
      };

      return { ego_location.on_section_or_origin_section_->getDrainPoint(), dir.angle({ 1, 0 }) };
   }
}

void vfm::RoadGraph::normalizeRoadGraphToEgoSection()
{
   const auto r_ego = findSectionWithEgoIfAny();

   if (!r_ego) addError("No ego found on any straight section in road graph.");

   const Vec2D specialPoint{ r_ego->getOriginPoint() };
   const float theta{ r_ego->getAngle() };
   const float x_s_prime = specialPoint.x * std::cos(theta) - specialPoint.y * std::sin(theta);
   const float y_s_prime = specialPoint.x * std::sin(theta) + specialPoint.y * std::cos(theta);

   for (const auto& r : getAllNodes()) {
      // Rotation
      const float x_rot = r->getOriginPoint().x * cos(theta) - r->getOriginPoint().y * sin(theta);
      const float y_rot = r->getOriginPoint().x * sin(theta) + r->getOriginPoint().y * cos(theta);

      // Translation such that ego section sits at origin.
      r->setOriginPoint({ x_rot - x_s_prime, y_rot - y_s_prime });
      r->setAngle(r->getAngle() - theta);
   }

   assert(r_ego->isRootedInZeroAndUnturned());
}

void vfm::RoadGraph::transformAllCarsToStraightRoadSections()
{


   //// Draw cars in crossing
   //if (i >= FIRST_LANE_CONNECTOR_ID) { // This is the id of the pavement.
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 0, 3, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 1, 3, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 2, 3, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 3, 3, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 0, 13, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 1, 13, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 2, 13, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 3, 13, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 0, 23, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 1, 23, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 2, 23, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 3, 23, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 0, 33, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 1, 33, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 2, 33, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 3, 33, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 0, 43, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 1, 43, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 2, 43, 10, 0 }); // TODO
   //   //r->addNonegoOnCrossingTowards(r_succ, CarPars{ 3, 43, 10, 0 }); // TODO

   //   CarParsVec nonegos_on_crossing{ r->getNonegosOnCrossingTowardsSuccessor(r_succ) };

   //   for (const auto& nonego : nonegos_on_crossing) {
   //      if (nonego.car_lane_ == i - FIRST_LANE_CONNECTOR_ID) {
   //         float arc_length{ bezier::arcLength(1, a_connector_basepoint_translated, between1, between2, b_connector_basepoint_translated) / norm_length_a };
   //         float rel{ nonego.car_rel_pos_ / arc_length };

   //         rel = std::max(0.0f, std::min(1.0f, rel));

   //         Vec2D p{ bezier::pointAtRatio(rel, a_connector_basepoint_translated, between1, between2, b_connector_basepoint_translated) };
   //         Vec2D dir{ bezier::B_prime(rel, a_connector_basepoint_translated, between1, between2, b_connector_basepoint_translated) };
   //         plotCar2D(3, p, CAR_COLOR, BLACK, { norm_length_a, norm_length_a * LANE_WIDTH }, dir.angle({ 1, 0 }));
   //         //plotCar3D(p, CAR_COLOR, BLACK, { norm_length_a, norm_length_a * LANE_WIDTH }, dir.angle({ 1, 0 }));
   //      }
   //   }
   //}
}

static constexpr float EPS{ 0.01 };
bool vfm::RoadGraph::isOriginAtZero() const
{

   return std::abs(origin_point_.x) < EPS && std::abs(origin_point_.y) < EPS;
}

bool vfm::RoadGraph::isAngleZero() const
{
   return std::abs(angle_) < EPS;
}

int vfm::RoadGraph::getID() const
{
   return id_;
}

int vfm::RoadGraph::getNodeCount() const
{
   int cnt{ 0 };

   const_cast<RoadGraph*>(this)->findFirstSectionWithProperty([&cnt](const std::shared_ptr<RoadGraph>) -> bool {
      cnt++;
      return false;
   });

   return cnt;
}

void vfm::RoadGraph::setMyRoad(const StraightRoadSection& section)
{
   my_road_ = section;
}

void vfm::RoadGraph::setOriginPoint(const Vec2D& point)
{
   origin_point_ = point;
}

void vfm::RoadGraph::setAngle(const float angle)
{
   angle_ = angle;
}

void vfm::RoadGraph::addSuccessor(const std::shared_ptr<RoadGraph> subgraph)
{
   for (const auto& el : successors_) if (el == subgraph) return; // Already is a successor

   successors_.push_back(subgraph);
   subgraph->predecessors_.push_back(shared_from_this());
}

void vfm::RoadGraph::addPredecessor(const std::shared_ptr<RoadGraph> subgraph)
{
   for (const auto& el : predecessors_) if (el == subgraph) return; // Already is a predecessor

   predecessors_.push_back(subgraph);
   subgraph->successors_.push_back(shared_from_this());
}

Rec2D vfm::RoadGraph::getBoundingBox() const
{
   Pol2D temp_pol{};

   const_cast<RoadGraph*>(this)->applyToMeAndAllMySuccessorsAndPredecessors([&temp_pol](const std::shared_ptr<RoadGraph> r) {
      temp_pol.add(r->getOriginPoint());
      temp_pol.add(r->getDrainPoint());
   });

   return *temp_pol.getBoundingBox();
}

std::vector<std::shared_ptr<RoadGraph>> vfm::RoadGraph::getSuccessors() const
{
   return successors_;
}

std::vector<std::shared_ptr<RoadGraph>> vfm::RoadGraph::getPredecessors() const
{
   return predecessors_;
}

std::vector<std::shared_ptr<RoadGraph>> vfm::RoadGraph::getAllNodes() const // TODO: Why not use set? (I believe there WAS a reason...)
{
   std::vector<std::shared_ptr<RoadGraph>> res{};

   const_cast<RoadGraph*>(this)->findFirstSectionWithProperty([&res](const std::shared_ptr<RoadGraph> r) -> bool {
      for (const auto& el : res) {
         if (el->getID() == r->getID()) return false;
      }

      res.push_back(r);
      return false;
   });

   return res;
}

std::vector<CarPars> vfm::RoadGraph::getNonegosOnCrossingTowardsSuccessor(const std::shared_ptr<RoadGraph> r) const
{
   return nonegos_towards_successors_.count(r) ? nonegos_towards_successors_.at(r) : CarParsVec{};
}

std::map<std::shared_ptr<RoadGraph>, std::vector<CarPars>> vfm::RoadGraph::getNonegosOnCrossingTowardsAllSuccessors() const
{
   return nonegos_towards_successors_;
}

void vfm::RoadGraph::addNonegoOnCrossingTowards(const std::shared_ptr<RoadGraph> r, const CarPars& nonego)
{
   nonegos_towards_successors_.insert({ r, {} }); // Add empty vector only if this successor not yet in the map.
   nonegos_towards_successors_.at(r).push_back(nonego);
}

void vfm::RoadGraph::addNonegoOnCrossingTowards(const int r_id, const CarPars& nonego)
{
   auto r = findSectionWithID(r_id);

   if (r) {
      addNonegoOnCrossingTowards(r, nonego);
   }
   else {
      addError("Id '" + std::to_string(r_id) + "' not found as successor of section '" + std::to_string(id_) + "'.");
   }
}

void vfm::RoadGraph::clearNonegosOnCrossingsTowardsAny()
{
   nonegos_towards_successors_.clear();
}

using namespace xml;

vfm::Way::Way(const std::shared_ptr<RoadGraph> my_father_rg) : Failable("OSM-Way"),
way_ids_{ WayIDs{} },
   origin_node_left_id_{ global_id_first_free++ },
   target_node_left_id_{ global_id_first_free++ },
   origin_node_right_id_{ global_id_first_free++ },
   target_node_right_id_{ global_id_first_free++ },
   ids_to_connector_successor_nodes_left_{},
   ids_to_connector_successor_nodes_right_{},
   ids_for_connector_successor_ways_{},
   my_road_graph_{ my_father_rg }
{
}

std::shared_ptr<xml::CodeXML> vfm::Way::getNodesXML() const
{
   auto origin = my_road_graph_->getOriginPoint();
   auto drain = my_road_graph_->getDrainPoint();
   Vec2D dir_lat{ drain - origin };
   dir_lat.ortho();
   dir_lat.setLength(LANE_WIDTH / 2);

   Vec2D origin_left{ origin - dir_lat };
   Vec2D origin_right{ origin + dir_lat };
   Vec2D drain_left{ drain - dir_lat };
   Vec2D drain_right{ drain + dir_lat };

   auto origin_left_coord = StaticHelper::cartesianToWGS84(origin_left.x, origin_left.y);
   auto target_left_coord = StaticHelper::cartesianToWGS84(drain_left.x, drain_left.y);
   auto origin_right_coord = StaticHelper::cartesianToWGS84(origin_right.x, origin_right.y);
   auto target_right_coord = StaticHelper::cartesianToWGS84(drain_right.x, drain_right.y);
   auto res =
      getNodeXML(origin_node_left_id_, origin_left_coord.first, origin_left_coord.second);
   res->appendAtTheEnd(getNodeXML(target_node_left_id_, target_left_coord.first, target_left_coord.second));
   res->appendAtTheEnd(getNodeXML(origin_node_right_id_, origin_right_coord.first, origin_right_coord.second));
   res->appendAtTheEnd(getNodeXML(target_node_right_id_, target_right_coord.first, target_right_coord.second));

   // Connectors towards successors.
   for (const auto& r_succ : my_road_graph_->getSuccessors()) {
      Vec2D origin_succ{ r_succ->getOriginPoint() };
      Vec2D drain_succ{ r_succ->getDrainPoint() };
      float dist{ drain.distance(origin_succ) };
      Vec2D dir_mine{ drain - origin };
      Vec2D dir_succ{ origin_succ - drain_succ };
      dir_mine.setLength(dist / 3);
      dir_succ.setLength(dist / 3);

      Vec2D dir_lat_succ{ drain_succ - origin_succ };
      dir_lat_succ.ortho();
      dir_lat_succ.setLength(LANE_WIDTH / 2);
      Vec2D origin_left_succ{ origin_succ - dir_lat_succ };
      Vec2D origin_right_succ{ origin_succ + dir_lat_succ };

      Pol2D pol{};
      Pol2D arrow{};
      pol.bezier(drain, drain + dir_mine, origin_succ + dir_succ, origin_succ, 0.1, true);
      arrow.createArrow(pol, LANE_WIDTH, { drain_right, drain_left }, { origin_right_succ, origin_left_succ });

      std::vector<int> node_ids_left{};
      std::vector<int> node_ids_right{};

      if (!arrow.points_.size() % 2) addError("Bezier road block has odd number of points: '" + std::to_string(arrow.points_.size()) + "'.");

      auto func = [&node_ids_left, &node_ids_right, &res](const Vec2D point, const bool left) {
         int id = global_id_first_free++;
         auto coord = StaticHelper::cartesianToWGS84(point.x, point.y);
         res->appendAtTheEnd(getNodeXML(id, coord.first, coord.second));
         (left ? node_ids_left : node_ids_right).push_back(id);
      };

      for (int i = 0; i < arrow.points_.size() / 2; i++) {
         func(arrow.points_.at(i), true);
      }

      for (int i = arrow.points_.size() / 2; i < arrow.points_.size(); i++) {
         func(arrow.points_.at(i), false);
      }

      ids_to_connector_successor_nodes_left_.push_back(node_ids_left);
      ids_to_connector_successor_nodes_right_.push_back(node_ids_right);
      ids_for_connector_successor_ways_.push_back(WayIDs());
   }

   return res;
}

std::shared_ptr<xml::CodeXML> vfm::Way::getWayXMLGeneric(
   WayIDs way_ids,
   const std::vector<int> left_ids,
   const std::vector<int> right_ids
)
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

   for (const auto& id : left_ids) {
      way_inner_left->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(id) } }));
      way_inner_link->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(id) } }));
   }

   for (const auto& id : right_ids) {
      way_inner_right->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("nd", { {"ref", std::to_string(id) } }));
   }

   way_inner_left->appendAtTheEnd(dummy);
   way_inner_right->appendAtTheEnd(dummy);

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
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("member", { { "type","way" }, { "ref", std::to_string(way_ids.left_border_id_) }, { "role", "left" } }));
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("member", { { "type","way" }, { "ref", std::to_string(way_ids.right_border_id_) }, { "role", "right" } }));
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","location" }, { "v", "nonurban" } }));
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","rl:direction" }, { "v", "along" } }));
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","rl:id" }, { "v", std::to_string(way_ids.road_link_id_) } }));
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","speed_limit" }, { "v", "130.000000" } }));
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","subtype" }, { "v", "highway" } }));
   relation_inner->appendAtTheEnd(xml::CodeXML::retrieveParsOnlyElement("tag", { { "k","type" }, { "v", "lanelet" } }));

   auto xml = xml::CodeXML::retrieveElementWithXMLContent("way", { {"id", std::to_string(way_ids.left_border_id_)},{"visible", "true"}, {"version", "1"} }, way_inner_left);
   xml->appendAtTheEnd(xml::CodeXML::retrieveElementWithXMLContent("way", { {"id", std::to_string(way_ids.right_border_id_)},{"visible", "true"}, {"version", "1"} }, way_inner_right));
   xml->appendAtTheEnd(xml::CodeXML::retrieveElementWithXMLContent("way", { {"id", std::to_string(way_ids.road_link_id_)},{"visible", "true"}, {"version", "1"} }, way_inner_link));
   xml->appendAtTheEnd(xml::CodeXML::retrieveElementWithXMLContent("relation", { { "id", std::to_string(way_ids.relation_id_) }, {"visible", "true"}, { "version", "1" } }, relation_inner));

   return xml;
}

std::shared_ptr<xml::CodeXML> vfm::Way::getWayXML() const
{
   auto res = getWayXMLGeneric(way_ids_, { origin_node_left_id_, target_node_left_id_ }, { origin_node_right_id_, target_node_right_id_ });
   
   if (ids_to_connector_successor_nodes_left_.size() != ids_to_connector_successor_nodes_right_.size()) {
      addError("Different number of connector ways to the left (" 
         + std::to_string(ids_to_connector_successor_nodes_left_.size()) + ") and right (" 
         + std::to_string(ids_to_connector_successor_nodes_right_.size()) + ").");
   }
   
   if (ids_to_connector_successor_nodes_left_.size() != ids_for_connector_successor_ways_.size()) {
      addError("Different number of connector way ids (" 
         + std::to_string(ids_for_connector_successor_ways_.size()) + ") and left node ids ("
         + std::to_string(ids_to_connector_successor_nodes_left_.size()) + ").");
   }

   for (int i = 0; i < ids_to_connector_successor_nodes_left_.size(); i++) {
      auto ways = ids_for_connector_successor_ways_.at(i);
      auto left_ids = ids_to_connector_successor_nodes_left_.at(i);
      auto right_ids = ids_to_connector_successor_nodes_right_.at(i);
      res->appendAtTheEnd(getWayXMLGeneric(ways, left_ids, right_ids));
   }

   return res;
}

std::pair<std::shared_ptr<xml::CodeXML>, std::shared_ptr<xml::CodeXML>> vfm::Way::getXML() const
{
   auto xml_nodes = CodeXML::emptyXML();
   auto xml_ways = CodeXML::emptyXML();

   xml_nodes->appendAtTheEnd(getNodesXML());
   xml_ways->appendAtTheEnd(getWayXML());

   return { xml_nodes, xml_ways };
}

std::shared_ptr<xml::CodeXML> vfm::RoadGraph::generateOSM() const
{
   auto xml = CodeXML::beginXML();
   auto osm_nodes = CodeXML::emptyXML();
   auto osm_ways = CodeXML::emptyXML();
   
   const_cast<RoadGraph*>(this)->findFirstSectionWithProperty([&osm_nodes, &osm_ways](const std::shared_ptr<RoadGraph> r) -> bool {
      auto nodes_ways = Way(r).getXML();
      osm_nodes->appendAtTheEnd(nodes_ways.first);
      osm_ways->appendAtTheEnd(nodes_ways.second);
      return false;
   });

   osm_nodes->appendAtTheEnd(osm_ways);

   xml->appendAtTheEnd(CodeXML::retrieveElementWithXMLContent("osm", { { "version", "0.6" }, { "upload", "false" }, { "generator", "vfm" } }, osm_nodes));

   return xml;
}
