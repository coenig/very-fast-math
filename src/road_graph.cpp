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

StraightRoadSection::StraightRoadSection() : StraightRoadSection(-1) {} // Constructs an invalid lane structure.
StraightRoadSection::StraightRoadSection(const int lane_num) : StraightRoadSection(lane_num, std::vector<LaneSegment>{}) {}

StraightRoadSection::StraightRoadSection(const int lane_num, const std::vector<LaneSegment>& segments) : num_lanes_(lane_num), Failable("StraightRoadSection") {
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
         addError("Invalid segment " + segment.second.toString() + " for lane number " + std::to_string(num_lanes_) + ". Skipping this segment.");
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
