#include <benchmark/benchmark.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <numbers>
#include <random>
#include <strada/cpm/compiled_physics_model.hpp>
#include <strada/cpm/coordinate.hpp>
#include <strada/cpm/ids.hpp>
#include <strada/cpm/query_context.hpp>
#include <strada/parser/parser.hpp>
#include <vector>

using namespace std::literals;

namespace {

enum class Coherence : std::uint8_t { kCoherent, kRandom };

using ::strada::cpm::CompiledPhysicsModel;
using ::strada::cpm::InertialPose;
using ::strada::cpm::LaneId;
using ::strada::cpm::LanePose;
using ::strada::cpm::QueryContext;
using ::strada::cpm::RoadId;
using ::strada::cpm::RoadPose;

// --- Map Fixture Tag Structs ---

struct ExLineSpiralArc {
  static constexpr auto kName = "Ex_Line-Spiral-Arc"sv;
  static constexpr auto kFilePath = "examples/Ex_Line-Spiral-Arc/Ex_Line-Spiral-Arc.xodr"sv;
};

struct UC5RoadJunction {
  static constexpr auto kName = "UC_5Road_Junction"sv;
  static constexpr auto kFilePath = "use_cases/UC_Junction/UC_5Road_Junction.xodr"sv;
};

struct UCMotorwayExitEntry {
  static constexpr auto kName = "UC_Motorway-Exit-Entry"sv;
  static constexpr auto kFilePath = "use_cases/UC_Motorway-Exit-Entry/UC_Motorway-Exit-Entry.xodr"sv;
};

// --- Map Data Cache / Pre-generator ---

template <typename MapFixture>
struct MapDataCache {
  std::unique_ptr<CompiledPhysicsModel> model;
  std::vector<RoadPose> road_poses_coherent;
  std::vector<RoadPose> road_poses_random;
  std::vector<LanePose> lane_poses_coherent;
  std::vector<LanePose> lane_poses_random;
  std::vector<InertialPose> inertial_poses_coherent;
  std::vector<InertialPose> inertial_poses_random;

  static auto Get() -> const MapDataCache& {
    static MapDataCache instance;
    return instance;
  }

 private:
  MapDataCache()
      : model(std::make_unique<CompiledPhysicsModel>(
            strada::parser::ParseFile(std::filesystem::path{STRADA_BENCHMARK_DATA_DIR} / MapFixture::kFilePath))) {
    constexpr auto num_poses = 1000ULL;
    road_poses_coherent.reserve(num_poses);
    road_poses_random.reserve(num_poses);
    lane_poses_coherent.reserve(num_poses);
    lane_poses_random.reserve(num_poses);
    inertial_poses_coherent.reserve(num_poses);
    inertial_poses_random.reserve(num_poses);

    constexpr auto seed = 42U;
    auto gen = std::mt19937{seed};  // NOLINT(cert-msc51-cpp,cert-msc32-c)

    GenerateCoherentPoses(num_poses);
    GenerateRandomPoses(num_poses, gen);
  }

  void GenerateCoherentPoses(std::size_t num_poses) {
    const auto road_count = model->RoadCount();
    if (road_count == 0) {
      return;
    }

    auto coherent_poses_generated = std::size_t{0};
    while (coherent_poses_generated < num_poses) {
      for (auto road_idx = 0ULL; road_idx < road_count && coherent_poses_generated < num_poses; ++road_idx) {
        const auto road_id = static_cast<RoadId>(road_idx);
        GenerateCoherentPosesForRoad(road_id, num_poses, coherent_poses_generated);
      }
    }

    // Fill lane_poses_coherent to num_poses by looping/duplicating if needed
    while (lane_poses_coherent.size() < num_poses && !lane_poses_coherent.empty()) {
      lane_poses_coherent.push_back(lane_poses_coherent[lane_poses_coherent.size() % lane_poses_coherent.size()]);
    }
    if (lane_poses_coherent.empty()) {
      auto lane_pose = LanePose{};
      lane_pose.road = static_cast<RoadId>(0);
      lane_pose.lane = static_cast<LaneId>(0);
      lane_poses_coherent.assign(num_poses, lane_pose);
    }
  }

  void GenerateCoherentPosesForRoad(RoadId road_id, std::size_t num_poses, std::size_t& coherent_poses_generated) {
    const auto len = model->RoadLength(road_id);
    if (len <= 0.0) {
      return;
    }

    constexpr auto road_step_divisor = 100.0;
    constexpr auto min_step = 0.1;
    constexpr auto pose_offset_t = 1.0;
    constexpr auto pose_offset_h = 0.1;

    const auto step = std::max(len / road_step_divisor, min_step);
    const auto num_steps = static_cast<std::size_t>(std::ceil(len / step));
    for (auto step_idx = 0ULL; step_idx <= num_steps && coherent_poses_generated < num_poses; ++step_idx) {
      const auto s_coord = std::min(static_cast<double>(step_idx) * step, len);

      auto road_pose = RoadPose{};
      road_pose.s = s_coord;
      road_pose.t = pose_offset_t;
      road_pose.h = pose_offset_h;
      road_pose.heading = 0.0;
      road_pose.pitch = 0.0;
      road_pose.roll = 0.0;
      road_pose.road = road_id;
      road_poses_coherent.push_back(road_pose);

      // Get corresponding inertial pose
      auto dummy_ctx = QueryContext{};
      const auto inertial_pose = model->RoadToInertial(road_pose, dummy_ctx);
      inertial_poses_coherent.push_back(inertial_pose);

      // Try to get a valid lane pose
      const auto lane_pose_opt = model->RoadToLane(road_pose, dummy_ctx);
      if (lane_pose_opt.has_value()) {
        lane_poses_coherent.push_back(*lane_pose_opt);
      } else {
        // fallback: try to find any valid lane
        auto lane_pose = LanePose{};
        lane_pose.s = s_coord;
        lane_pose.t = 0.0;
        lane_pose.h = pose_offset_h;
        lane_pose.heading = 0.0;
        lane_pose.pitch = 0.0;
        lane_pose.roll = 0.0;
        lane_pose.road = road_id;
        auto lane_id_opt = model->FindLaneId(road_id, 0, -1);
        if (!lane_id_opt.has_value()) {
          lane_id_opt = model->FindLaneId(road_id, 0, 1);
        }
        if (lane_id_opt.has_value()) {
          lane_pose.lane = *lane_id_opt;
          lane_poses_coherent.push_back(lane_pose);
        }
      }
      coherent_poses_generated++;
    }
  }

  void GenerateRandomPoses(std::size_t num_poses, std::mt19937& gen) {
    const auto road_count = model->RoadCount();
    if (road_count == 0) {
      return;
    }

    constexpr auto bbox_margin = 50.0;
    constexpr auto z_min = -10.0;
    constexpr auto z_max = 10.0;
    constexpr auto noise_max = 15.0;
    constexpr auto perturbed_z_scale = 0.1;

    auto min_x = 0.0;
    auto max_x = 0.0;
    auto min_y = 0.0;
    auto max_y = 0.0;
    if (!inertial_poses_coherent.empty()) {
      min_x = max_x = inertial_poses_coherent[0].x;
      min_y = max_y = inertial_poses_coherent[0].y;
      for (const auto& inertial_pose : inertial_poses_coherent) {
        min_x = std::min(min_x, inertial_pose.x);
        max_x = std::max(max_x, inertial_pose.x);
        min_y = std::min(min_y, inertial_pose.y);
        max_y = std::max(max_y, inertial_pose.y);
      }
    }
    min_x -= bbox_margin;
    max_x += bbox_margin;
    min_y -= bbox_margin;
    max_y += bbox_margin;

    auto dist_x = std::uniform_real_distribution<double>{min_x, max_x};
    auto dist_y = std::uniform_real_distribution<double>{min_y, max_y};
    auto dist_z = std::uniform_real_distribution<double>{z_min, z_max};
    auto dist_angle = std::uniform_real_distribution<double>{-std::numbers::pi, std::numbers::pi};

    auto dist_coherent = std::uniform_int_distribution<std::size_t>{0, inertial_poses_coherent.size() - 1};
    auto dist_noise = std::uniform_real_distribution<double>{-noise_max, noise_max};

    for (auto i = 0ULL; i < num_poses; ++i) {
      const auto idx = dist_coherent(gen);
      const auto base_ip = inertial_poses_coherent[idx];

      auto perturbed_ip = base_ip;
      perturbed_ip.x += dist_noise(gen);
      perturbed_ip.y += dist_noise(gen);
      perturbed_ip.z += dist_noise(gen) * perturbed_z_scale;

      auto dummy_ctx = QueryContext{};
      const auto road_pose_opt = model->InertialToRoad(perturbed_ip, dummy_ctx);
      if (road_pose_opt.has_value()) {
        road_poses_random.push_back(*road_pose_opt);
      } else {
        const auto base_rp_opt = model->InertialToRoad(base_ip, dummy_ctx);
        if (base_rp_opt.has_value()) {
          road_poses_random.push_back(*base_rp_opt);
        } else {
          auto road_pose = RoadPose{};
          road_pose.road = static_cast<RoadId>(0);
          road_poses_random.push_back(road_pose);
        }
      }

      auto inertial_pose = InertialPose{};
      inertial_pose.x = dist_x(gen);
      inertial_pose.y = dist_y(gen);
      inertial_pose.z = dist_z(gen);
      inertial_pose.heading = dist_angle(gen);
      inertial_pose.pitch = dist_angle(gen) * perturbed_z_scale;
      inertial_pose.roll = dist_angle(gen) * perturbed_z_scale;
      inertial_poses_random.push_back(inertial_pose);

      const auto lane_pose_opt = model->InertialToLane(perturbed_ip, dummy_ctx);
      if (lane_pose_opt.has_value()) {
        lane_poses_random.push_back(*lane_pose_opt);
      } else {
        const auto base_lp_opt = model->InertialToLane(base_ip, dummy_ctx);
        if (base_lp_opt.has_value()) {
          lane_poses_random.push_back(*base_lp_opt);
        } else {
          auto lane_pose = LanePose{};
          lane_pose.road = static_cast<RoadId>(0);
          lane_pose.lane = static_cast<LaneId>(0);
          lane_poses_random.push_back(lane_pose);
        }
      }
    }
  }
};

// --- Benchmark Functions ---

template <typename MapFixture, Coherence CoherenceMode>
void BM_RoadToInertial(benchmark::State& state) {  // NOLINT(readability-identifier-naming)
  const auto& cache = MapDataCache<MapFixture>::Get();
  const auto& poses = (CoherenceMode == Coherence::kCoherent) ? cache.road_poses_coherent : cache.road_poses_random;
  const auto& model = *cache.model;
  auto ctx = QueryContext{};
  auto pose_idx = 0ULL;

  for (auto _ : state) {  // NOLINT(readability-identifier-length)
    if constexpr (CoherenceMode == Coherence::kRandom) {
      ctx = QueryContext{};
    }
    const auto road_pose = poses[pose_idx];
    auto inertial_pose = model.RoadToInertial(road_pose, ctx);
    benchmark::DoNotOptimize(inertial_pose);
    pose_idx = (pose_idx + 1) % poses.size();
  }
}

template <typename MapFixture, Coherence CoherenceMode>
void BM_LaneToInertial(benchmark::State& state) {  // NOLINT(readability-identifier-naming)
  const auto& cache = MapDataCache<MapFixture>::Get();
  const auto& poses = (CoherenceMode == Coherence::kCoherent) ? cache.lane_poses_coherent : cache.lane_poses_random;
  const auto& model = *cache.model;
  auto ctx = QueryContext{};
  auto pose_idx = 0ULL;

  for (auto _ : state) {  // NOLINT(readability-identifier-length)
    if constexpr (CoherenceMode == Coherence::kRandom) {
      ctx = QueryContext{};
    }
    const auto lane_pose = poses[pose_idx];
    auto inertial_pose = model.LaneToInertial(lane_pose, ctx);
    benchmark::DoNotOptimize(inertial_pose);
    pose_idx = (pose_idx + 1) % poses.size();
  }
}

template <typename MapFixture, Coherence CoherenceMode>
void BM_InertialToRoad(benchmark::State& state) {  // NOLINT(readability-identifier-naming)
  const auto& cache = MapDataCache<MapFixture>::Get();
  const auto& poses =
      (CoherenceMode == Coherence::kCoherent) ? cache.inertial_poses_coherent : cache.inertial_poses_random;
  const auto& model = *cache.model;
  auto ctx = QueryContext{};
  auto pose_idx = 0ULL;

  for (auto _ : state) {  // NOLINT(readability-identifier-length)
    if constexpr (CoherenceMode == Coherence::kRandom) {
      ctx = QueryContext{};
    }
    const auto inertial_pose = poses[pose_idx];
    auto road_pose_opt = model.InertialToRoad(inertial_pose, ctx);
    benchmark::DoNotOptimize(road_pose_opt);
    pose_idx = (pose_idx + 1) % poses.size();
  }
}

template <typename MapFixture, Coherence CoherenceMode>
void BM_InertialToLane(benchmark::State& state) {  // NOLINT(readability-identifier-naming)
  const auto& cache = MapDataCache<MapFixture>::Get();
  const auto& poses =
      (CoherenceMode == Coherence::kCoherent) ? cache.inertial_poses_coherent : cache.inertial_poses_random;
  const auto& model = *cache.model;
  auto ctx = QueryContext{};
  auto pose_idx = 0ULL;

  for (auto _ : state) {  // NOLINT(readability-identifier-length)
    if constexpr (CoherenceMode == Coherence::kRandom) {
      ctx = QueryContext{};
    }
    const auto inertial_pose = poses[pose_idx];
    auto lane_pose_opt = model.InertialToLane(inertial_pose, ctx);
    benchmark::DoNotOptimize(lane_pose_opt);
    pose_idx = (pose_idx + 1) % poses.size();
  }
}

template <typename MapFixture, Coherence CoherenceMode>
void BM_RoadToLane(benchmark::State& state) {  // NOLINT(readability-identifier-naming)
  const auto& cache = MapDataCache<MapFixture>::Get();
  const auto& poses = (CoherenceMode == Coherence::kCoherent) ? cache.road_poses_coherent : cache.road_poses_random;
  const auto& model = *cache.model;
  auto ctx = QueryContext{};
  auto pose_idx = 0ULL;

  for (auto _ : state) {  // NOLINT(readability-identifier-length)
    if constexpr (CoherenceMode == Coherence::kRandom) {
      ctx = QueryContext{};
    }
    const auto road_pose = poses[pose_idx];
    auto lane_pose_opt = model.RoadToLane(road_pose, ctx);
    benchmark::DoNotOptimize(lane_pose_opt);
    pose_idx = (pose_idx + 1) % poses.size();
  }
}

template <typename MapFixture, Coherence CoherenceMode>
void BM_LaneToRoad(benchmark::State& state) {  // NOLINT(readability-identifier-naming)
  const auto& cache = MapDataCache<MapFixture>::Get();
  const auto& poses = (CoherenceMode == Coherence::kCoherent) ? cache.lane_poses_coherent : cache.lane_poses_random;
  const auto& model = *cache.model;
  auto ctx = QueryContext{};
  auto pose_idx = 0ULL;

  for (auto _ : state) {  // NOLINT(readability-identifier-length)
    if constexpr (CoherenceMode == Coherence::kRandom) {
      ctx = QueryContext{};
    }
    const auto lane_pose = poses[pose_idx];
    auto road_pose = model.LaneToRoad(lane_pose, ctx);
    benchmark::DoNotOptimize(road_pose);
    pose_idx = (pose_idx + 1) % poses.size();
  }
}

}  // namespace

// --- Benchmark Registration ---

// RoadToInertial
BENCHMARK_TEMPLATE(BM_RoadToInertial, ExLineSpiralArc, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_RoadToInertial, ExLineSpiralArc, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_RoadToInertial, UC5RoadJunction, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_RoadToInertial, UC5RoadJunction, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_RoadToInertial, UCMotorwayExitEntry, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_RoadToInertial, UCMotorwayExitEntry, Coherence::kRandom);

// LaneToInertial
BENCHMARK_TEMPLATE(BM_LaneToInertial, ExLineSpiralArc, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_LaneToInertial, ExLineSpiralArc, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_LaneToInertial, UC5RoadJunction, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_LaneToInertial, UC5RoadJunction, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_LaneToInertial, UCMotorwayExitEntry, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_LaneToInertial, UCMotorwayExitEntry, Coherence::kRandom);

// InertialToRoad
BENCHMARK_TEMPLATE(BM_InertialToRoad, ExLineSpiralArc, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_InertialToRoad, ExLineSpiralArc, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_InertialToRoad, UC5RoadJunction, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_InertialToRoad, UC5RoadJunction, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_InertialToRoad, UCMotorwayExitEntry, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_InertialToRoad, UCMotorwayExitEntry, Coherence::kRandom);

// InertialToLane
BENCHMARK_TEMPLATE(BM_InertialToLane, ExLineSpiralArc, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_InertialToLane, ExLineSpiralArc, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_InertialToLane, UC5RoadJunction, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_InertialToLane, UC5RoadJunction, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_InertialToLane, UCMotorwayExitEntry, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_InertialToLane, UCMotorwayExitEntry, Coherence::kRandom);

// RoadToLane
BENCHMARK_TEMPLATE(BM_RoadToLane, ExLineSpiralArc, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_RoadToLane, ExLineSpiralArc, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_RoadToLane, UC5RoadJunction, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_RoadToLane, UC5RoadJunction, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_RoadToLane, UCMotorwayExitEntry, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_RoadToLane, UCMotorwayExitEntry, Coherence::kRandom);

// LaneToRoad
BENCHMARK_TEMPLATE(BM_LaneToRoad, ExLineSpiralArc, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_LaneToRoad, ExLineSpiralArc, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_LaneToRoad, UC5RoadJunction, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_LaneToRoad, UC5RoadJunction, Coherence::kRandom);
BENCHMARK_TEMPLATE(BM_LaneToRoad, UCMotorwayExitEntry, Coherence::kCoherent);
BENCHMARK_TEMPLATE(BM_LaneToRoad, UCMotorwayExitEntry, Coherence::kRandom);
