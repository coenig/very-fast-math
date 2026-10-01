#include "simulation/road_graph.h"
#include "model_checking/cex_processing/mc_trajectory_visualizers.h"
#include <gtest/gtest.h>

using namespace vfm;

namespace {
std::shared_ptr<RoadGraph> section(int id, Vec2D origin) {
   auto road = std::make_shared<RoadGraph>(id);
   road->setOriginPoint(origin);
   road->setMyRoad(StraightRoadSection{1, 1, 20, 3.75});
   road->getMyRoad().addLaneSegment({0, 0, 0});
   return road;
}
void nearPoint(Vec2D actual, Vec2D expected) {
   EXPECT_NEAR(actual.x, expected.x, 0.001);
   EXPECT_NEAR(actual.y, expected.y, 0.001);
}
}

TEST(EndpointRendering, LegacyDefaults) {
   auto a = section(0, {0, 0}), b = section(1, {30, 10});
   a->addSuccessor(b);
   auto p = a->getConnectionPoints(b);
   nearPoint(p[0], a->getDrainPoint());
   nearPoint(p[2], b->getOriginPoint());
   using Side = ConnectorPolygonEnding::Side;
   EXPECT_TRUE(a->matchesConnectionSides(1, Side::drain, Side::source));
   EXPECT_FALSE(a->matchesConnectionSides(1, Side::source, Side::source));
}

TEST(EndpointRendering, AllEndpointPairsSurviveCopyFromEitherRoot) {
   using Side = ConnectorPolygonEnding::Side;
   for (bool from_end : {false, true}) for (bool to_end : {false, true}) {
      auto a = section(0, {0, 0}), b = section(1, {30, 10});
      a->addSuccessor(b);
      a->setConnectionEndpoints(1, from_end, to_end, 10);
      for (auto root : {a, b}) {
         auto copy = root->copy();
         auto ca = copy->findSectionWithID(0), cb = copy->findSectionWithID(1);
         const auto p = ca->getConnectionPoints(cb);
         nearPoint(p[0], from_end ? a->getDrainPoint() : a->getOriginPoint());
         nearPoint(p[1], from_end ? a->getOriginPoint() : a->getDrainPoint());
         nearPoint(p[2], to_end ? b->getDrainPoint() : b->getOriginPoint());
         nearPoint(p[3], to_end ? b->getOriginPoint() : b->getDrainPoint());
         EXPECT_EQ(ca->getConnectionEndpoints(1).length, 10);
         for (auto s : {Side::source, Side::drain}) for (auto t : {Side::source, Side::drain}) {
            EXPECT_EQ(ca->matchesConnectionSides(1, s, t),
               (s == (from_end ? Side::drain : Side::source)) &&
               (t == (to_end ? Side::drain : Side::source)));
         }
      }
   }
}

TEST(EndpointRendering, CarsUseSelectedEndpointsAndTraceLengthInBothDirections) {
   for (bool from_end : {false, true}) for (bool to_end : {false, true}) {
      for (int velocity : {-2, 2}) for (float pos : {0.0f, 10.0f}) {
         auto a = section(0, {0, 0}), b = section(1, {30, 10});
         a->addSuccessor(b);
         a->setConnectionEndpoints(1, from_end, to_end, 10);
         a->addNonegoOnCrossingTowards(b, CarPars{0, pos, velocity, RoadGraph::EGO_MOCK_ID, DEFAULT_CAR_DIMENSIONS_M});
         const auto points = a->getConnectionPoints(b);
         a->transformAllCarsToStraightRoadSections();
         auto ego = a->findSectionWithEgoIfAny();
         ASSERT_NE(ego, nullptr);
         nearPoint(ego->getOriginPoint(), pos == 0 ? points[0] : points[2]);
      }
   }
}

TEST(EndpointRendering, NativeTraceLoaderReadsEndpointFlags) {
   // Exercise the real NuSMV parser (including env aliases) and graph loader.
   std::string text = "Trace Description: test\nTrace Type: Counterexample\n  -> State: 1.1 <-\n"
      "    env.num_lanes = 1\n    env.num_technical_lanes = 1\n    env.lane_width = 375\n";
   for (int i = 0; i < 2; ++i) {
      std::string s = "    env.section_" + std::to_string(i);
      text += s + "_end = 20\n" + s + ".source.x = " + std::to_string(i * 30) + "\n"
         + s + ".source.y = 0\n" + s + ".angle = 0\n"
         + s + "_segment_0_pos_begin = 0\n" + s + "_segment_0_min_lane = 0\n"
         + s + "_segment_0_max_lane = 0\n";
   }
   text += "    env.outgoing_connection_0_of_section_0 = 1\n"
      "    env.endpoint_0_1_from_end = FALSE\n    env.endpoint_0_1_to_end = TRUE\n"
      "    env.arclength_from_sec_0_to_sec_1_on_lane_0 = 51\n";
   auto traces = StaticHelper::extractMCTracesFromNusmv(text);
   ASSERT_EQ(traces.size(), 1);
   using namespace mc::trajectory_generator;
   // Empty trajectory is sufficient: graph loading only consumes the raw trace.
   const InterpretationConfiguration config{"ego", {}, {}, {}, {},
      [](std::vector<ParameterMap>&) { return true; },
      [](MCinterpretedTrace& trace) { trace.noteVehicle("ego"); },
      [](MCinterpretedTrace&) {}, 1.0, '.'};
   const MCinterpretedTrace interpreted{0, std::vector<MCTrace>{MCTrace{}}, config};
   LiveSimGenerator renderer{interpreted};
   auto graph = renderer.getRoadGraphTopologyFrom(traces[0]);
   auto endpoints = graph->getConnectionEndpoints(1);
   EXPECT_FALSE(endpoints.from_end);
   EXPECT_TRUE(endpoints.to_end);
   EXPECT_EQ(endpoints.length, 51);
}

TEST(EndpointRendering, SmoothFramesRespectBothSidesOfAllEndpointTransfers) {
   using namespace mc::trajectory_generator;
   using P = PossibleParameter;
   std::vector<ParameterDetails> details;
   for (auto key : {P::pos_x, P::vel_x, P::on_straight_section, P::traversion_from, P::traversion_to}) {
      details.emplace_back(key, "test", false);
   }
   const InterpretationConfiguration config{"car", {}, {}, {}, details,
      [](std::vector<ParameterMap>&) { return true; },
      [](MCinterpretedTrace& trace) { trace.noteVehicle("car"); },
      [](MCinterpretedTrace&) {}, 1.0, '.'};
   for (bool from_end : {false, true}) for (bool to_end : {false, true}) {
      for (int section_id : {0, 1}) for (bool leaving_section : {false, true}) {
         const bool at_end = section_id == 0 ? from_end : to_end;
         const double section_boundary = at_end ? 20 : 0;
         const double connector_boundary = section_id == 0 ? 0 : 10;
         const double exit = leaving_section ? section_boundary : connector_boundary;
         const double entry = leaving_section ? connector_boundary : section_boundary;
         const double dir_before = exit == 0 ? -1 : 1;
         const double dir_after = entry == 0 ? 1 : -1;
         ParameterMap a{{P::pos_x, exit - dir_before}, {P::vel_x, dir_before * 4},
            {P::on_straight_section, leaving_section ? section_id : -1},
            {P::traversion_from, leaving_section ? -1 : 0}, {P::traversion_to, leaving_section ? -1 : 1}};
         ParameterMap b{{P::pos_x, entry + dir_after * 3}, {P::vel_x, dir_after * 4},
            {P::on_straight_section, leaving_section ? -1 : section_id},
            {P::traversion_from, leaving_section ? 0 : -1}, {P::traversion_to, leaving_section ? 1 : -1}};
         std::string text = "Trace Type: Counterexample\n  -> State: 1.1 <-\n"
            "    env.section_0_end = 20\n    env.section_1_end = 20\n"
            "    env.arclength_from_sec_0_to_sec_1_on_lane_0 = 10\n"
            "    car.on_lane = 0\n    env.endpoint_0_1_from_end = " + std::string(from_end ? "TRUE" : "FALSE")
            + "\n    env.endpoint_0_1_to_end = " + (to_end ? "TRUE" : "FALSE") + "\n";
         const std::map<P, std::string> names{{P::pos_x, "abs_pos"}, {P::vel_x, "v"},
            {P::on_straight_section, "on_straight_section"}, {P::traversion_from, "traversion_from"}, {P::traversion_to, "traversion_to"}};
         for (const auto& p : a) text += "    car." + names.at(p.first) + " = " + std::to_string(p.second) + "\n";
         text += "  -> State: 1.2 <-\n";
         for (const auto& p : b) text += "    car." + names.at(p.first) + " = " + std::to_string(p.second) + "\n";
         auto traces = StaticHelper::extractMCTracesFromNusmv(text);
         ASSERT_EQ(traces.size(), 1);
         MCinterpretedTrace interpreted{0, std::vector<MCTrace>{MCTrace{}}, config};
         FullTrajectory trajectory{{0, a}, {1, b}};
         interpreted.interpolate("car", trajectory, traces[0], 7);
         ASSERT_EQ(trajectory.size(), 9);
         for (int frame = 1; frame < 8; ++frame) {
            const double distance = frame * 0.5;
            const bool before = distance <= 1;
            const auto& params = trajectory[frame].second;
            EXPECT_NEAR(params.at(P::pos_x), before ? a.at(P::pos_x) + dir_before * distance
               : entry + dir_after * (distance - 1), 0.001);
            for (auto key : {P::on_straight_section, P::traversion_from, P::traversion_to})
               EXPECT_EQ(params.at(key), (before ? a : b).at(key));
            EXPECT_NEAR(params.at(P::vel_x), 4 * (before ? dir_before : dir_after), 0.001);
         }
      }
   }
}
