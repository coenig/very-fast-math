#include "simulation/road_graph.h"
#include <vector>
#include <cmath>


using namespace vfm;

LaneSegment::LaneSegment(const float begin, const int min_lane, const int max_lane)
   : begin_(begin), min_lane_(min_lane), max_lane_(max_lane) {
}

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

void StraightRoadSection::cleanUp()
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

   if (change_occurred) {
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

void vfm::StraightRoadSection::setEgo(const std::shared_ptr<CarPars> ego)
{
   ego_ = ego;
}

void vfm::StraightRoadSection::setOthers(const CarParsVec& others)
{
   others_ = others;
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
   if (property(shared_from_this())) return shared_from_this();
   if (!visited.insert(shared_from_this()).second) return nullptr;

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
   return findFirstSectionWithProperty([id](const std::shared_ptr<RoadGraph> r) -> bool { return id == r->getID(); });
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

std::shared_ptr<RoadGraph> vfm::RoadGraph::findSectionWithEgo()
{
   return findFirstSectionWithProperty([](const std::shared_ptr<RoadGraph> r) -> bool { return (bool) r->getMyRoad().getEgo(); });
}

void vfm::RoadGraph::applyToMeAndAllMySuccessorsAndPredecessors(const std::function<void(const std::shared_ptr<RoadGraph>)> action)
{
   findFirstSectionWithProperty([&action](const std::shared_ptr<RoadGraph> r) -> bool {
      action(r);
      return false;
   });
}

StraightRoadSection vfm::RoadGraph::getMyRoad() const
{
   return my_road_;
}

Vec2Df vfm::RoadGraph::getOriginPoint() const
{
   return origin_point_;
}

float vfm::RoadGraph::getAngle() const
{
   return angle_;
}

bool vfm::RoadGraph::isRootedInZeroAndUnturned() const
{
   return isOriginAtZero() && isAngleZero();
}

constexpr float EPS{ 0.01 };
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

int vfm::RoadGraph::getNumberOfNodes() const
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
   successors_.insert(subgraph);
   subgraph->predecessors_.insert(shared_from_this());
}

void vfm::RoadGraph::addPredecessor(const std::shared_ptr<RoadGraph> subgraph)
{
   predecessors_.insert(subgraph);
   subgraph->successors_.insert(shared_from_this());
}

std::set<std::shared_ptr<RoadGraph>> vfm::RoadGraph::getSuccessors() const
{
   return successors_;
}

std::set<std::shared_ptr<RoadGraph>> vfm::RoadGraph::getPredecessors() const
{
   return predecessors_;
}

std::set<std::shared_ptr<RoadGraph>> vfm::RoadGraph::getAllNodes() const
{
   std::set<std::shared_ptr<RoadGraph>> res{};

   const_cast<RoadGraph*>(this)->findFirstSectionWithProperty([&res](const std::shared_ptr<RoadGraph> r) -> bool {
      res.insert(r);
      return false;
   });

   return res;
}
