// 优化前冻结实现的逐项对照，同时报告合成基准；不代表现场实时性。
#include "map_server/derived_maps.hpp"
#include "map_server/static_map.hpp"
#define private public
#include "map_server/map_view.hpp"
#undef private
#include "minco_controller/mpc_solver.hpp"
#include "minco_controller/trajectory_safety.hpp"
#include "trajectory_safety_reference.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <random>
using namespace minco_controller;
using Clock = std::chrono::steady_clock;
void require(bool ok) {
  if (!ok)
    throw std::runtime_error("equivalence failed");
}
template <class Function> bool accepted(Function &&fn) {
  try {
    fn();
    return true;
  } catch (const std::runtime_error &) {
    return false;
  }
}
void stats(std::string name, std::vector<double> values) {
  std::sort(values.begin(), values.end());
  double sum = 0;
  for (double v : values)
    sum += v;
  std::cout << name << " mean=" << sum / values.size()
            << " max=" << values.back()
            << " p95=" << values[(values.size() - 1) * 95 / 100] << " ms\n";
}
map_server::Footprint body(bool concave) {
  map_server::Footprint f;
  std::vector<Eigen::Vector2d> xy =
      concave
          ? std::vector<Eigen::Vector2d>{{-.22, -.18}, {.22, -.18}, {.22, 0},
                                         {0, 0},       {0, .18},    {-.22, .18}}
          : std::vector<Eigen::Vector2d>{};
  if (!concave)
    for (int i = 0; i < 16; ++i)
      xy.push_back({.22 * cos(i * M_PI / 8), .22 * sin(i * M_PI / 8)});
  for (auto p : xy) {
    geometry_msgs::msg::Point v;
    v.x = p.x();
    v.y = p.y();
    f.push_back(v);
  }
  return f;
}
map_server::MapView makeMap(int mode) {
  auto s = std::make_shared<map_server::StaticMap>();
  s->geometry = {180, 180, .025, 0, 0};
  s->occupancy = {s->geometry, std::vector<uint8_t>(32400, 0)};
  std::mt19937 gen(22 + mode);
  for (int y = 0; y < 180; ++y)
    for (int x = 0; x < 180; ++x)
      if (mode && ((mode == 2 ? (x == 73 || x == 109) : gen() % 16 == 0)))
        s->occupancy.data[y * 180 + x] = (mode == 3 ? 255 : 1);
  auto d = std::make_shared<map_server::DerivedMaps>();
  d->geometry = s->geometry;
  d->fused_obstacles = s->occupancy;
  d->distance_field = {s->geometry, std::vector<float>(32400, 1)};
  d->collision_mask = s->occupancy;
  d->costmap = {s->geometry, std::vector<float>(32400, 1)};
  map_server::MapView map(s, d);
  if (mode == 4 || mode == 5) {
    auto dyn = std::make_shared<map_server::DynamicObstacleView>();
    dyn->circles = {{{2.22, 2.22}, .13}};
    auto cloud = std::make_shared<map_server::BinaryGrid>();
    cloud->geometry = {10, 10, .05, 2.4, 2.2};
    cloud->data.assign(100, 0);
    cloud->data[55] = 1;
    dyn->cloud_mask = cloud;
    map.dynamic_ = dyn;
    map.dynamic_version_ = 2;
    if (mode == 5) {
      d->dynamic_version = 2;
      for (int y = 85; y < 95; ++y)
        for (int x = 84; x < 94; ++x)
          d->fused_obstacles.data[y * 180 + x] = 1;
    }
  }
  return map;
}
void planesEqual(const std::vector<SafetyPlane> &a,
                 const std::vector<SafetyPlane> &b) {
  require(a.size() == b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    require(a[i].normal == b[i].normal && a[i].offset == b[i].offset &&
            a[i].preview_time == b[i].preview_time &&
            a[i].origin_offset == b[i].origin_offset);
  }
}
template <class Safety>
void cycle(Safety &safety, const State &start,
           const std::unordered_set<size_t> &contacts) {
  std::vector<SafetyPlane> planes;
  State a = start, b = start;
  for (int i = 0; i < 75; ++i) {
    a = start;
    a.x += .0003 * (i % 25);
    b = a;
    b.x += .003;
    require(safety.sampleMotion(a, b, .02, planes, &contacts));
  }
  a = start;
  for (int i = 0; i < 26; ++i) {
    b = a;
    b.x += .003;
    require(safety.checkEscapeMotion(a, b, &contacts));
    a = b;
  }
  safety.checkEscapeEnvelope(start, Control{.1, 0, 0}, .1, .02, contacts);
  State brake = start;
  brake.vx = .1;
  safety.checkBrakingMotion(brake, .12, .3, 1, contacts);
}
template <class Safety>
Control solve(Safety &safety, const State &start,
              const std::unordered_set<size_t> &contacts) {
  MPCConfig config;
  config.horizon = 25;
  config.dt = .02;
  config.control_period = .02;
  config.use_acc_constraints = true;
  config.vx_max = config.vy_max = .3 / std::sqrt(2.);
  config.vx_min = config.vy_min = -config.vx_max;
  config.ax_max = config.ay_max = .6 / std::sqrt(2.);
  config.ax_min = config.ay_min = -config.ax_max;
  config.safety_preview_time = .1;
  MpcSolver solver(config);
  CommandModel model;
  model.fixed_angular_velocity = true;
  std::vector<ReferencePoint> refs(25);
  for (int i = 0; i < 25; ++i) {
    refs[i].pos = {start.x + .002 * (i + 1), start.y};
    refs[i].vel = {.1, 0};
  }
  Control output;
  std::vector<State> pred;
  require(solver.solve(
      start, refs, output, &pred,
      [](const auto &p, CostLinearization &c) {
        c.position = p;
        c.cost = 0;
        c.gradient.setZero();
        return true;
      },
      &start,
      [&](const auto &measured, const auto &cmd, auto &planes) {
        return safety.sampleEnvelope(measured, cmd, .1, .02, .02, planes,
                                     &contacts);
      },
      model,
      [&](const auto &a, const auto &b, auto &planes) {
        return safety.sampleMotion(a, b, .02, planes, &contacts);
      }));
  require(std::hypot(output.vx, output.vy) <= .3 + 1e-8);
  for (size_t i = 1; i < pred.size(); ++i)
    require(safety.checkEscapeMotion(pred[i - 1], pred[i], &contacts));
  safety.checkEscapeEnvelope(start, output, .1, .02, contacts);
  State held = start;
  held.x += output.vx * .12;
  held.y += output.vy * .12;
  held.vx = output.vx;
  held.vy = output.vy;
  safety.checkBrakingMotion(held, .12, .3, 1, contacts);
  return output;
}
void cycleBench() {
  auto map = makeMap(0);
  auto s = std::const_pointer_cast<map_server::StaticMap>(map.staticMap());
  auto d = std::const_pointer_cast<map_server::DerivedMaps>(map.derivedMaps());
  for (int y = 88; y < 93; ++y)
    for (int x = 82; x < 85; ++x)
      s->occupancy.data[y * 180 + x] = d->fused_obstacles.data[y * 180 + x] = 1;
  auto fp = body(false);
  State start;
  start.x = start.y = 2.25;
  baseline::TrajectorySafety old(map, fp, .02);
  TrajectorySafety now(map, fp, .02);
  std::unordered_set<size_t> contacts;
  old.collisionCellCount(start, &contacts);
  std::cout << "cycle initial contacts=" << contacts.size() << "\n";
  std::vector<double> oc, nc, ccold, ccnew, ecold, ecnew;
  for (int i = 0; i < 20; ++i) {
    auto begin = Clock::now();
    cycle(old, start, contacts);
    oc.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin)
                     .count());
    begin = Clock::now();
    cycle(now, start, contacts);
    nc.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin)
                     .count());
  }
  for (int i = 0; i < 200; ++i) {
    auto begin = Clock::now();
    require(old.collisionCellCount(start) > 0);
    ccold.push_back(
        std::chrono::duration<double, std::milli>(Clock::now() - begin)
            .count());
    begin = Clock::now();
    require(now.collisionCellCount(start) > 0);
    ccnew.push_back(
        std::chrono::duration<double, std::milli>(Clock::now() - begin)
            .count());
    State end = start;
    end.x += .003;
    begin = Clock::now();
    require(old.checkEscapeMotion(start, end, &contacts));
    ecold.push_back(
        std::chrono::duration<double, std::milli>(Clock::now() - begin)
            .count());
    begin = Clock::now();
    require(now.checkEscapeMotion(start, end, &contacts));
    ecnew.push_back(
        std::chrono::duration<double, std::milli>(Clock::now() - begin)
            .count());
  }
  stats("old synthetic cycle n=20", oc);
  stats("new synthetic cycle n=20", nc);
  stats("old collisionCellCount", ccold);
  stats("new collisionCellCount", ccnew);
  stats("old checkEscapeMotion", ecold);
  stats("new checkEscapeMotion", ecnew);
  auto u = solve(old, start, contacts), v = solve(now, start, contacts);
  require(std::abs(u.vx - v.vx) < 1e-12 && std::abs(u.vy - v.vy) < 1e-12 &&
          std::abs(u.omega - v.omega) < 1e-12);
  require(v.vx > 0);
  std::cout << "PASS actual MPC output equivalent and nonzero, final "
               "prediction/envelope/braking safe\n";
  // 新快照包含新障碍：新建对象不能继承旧位姿缓存。
  auto changed = std::make_shared<map_server::DerivedMaps>(*d);
  changed->version = 2;
  changed->fused_obstacles.data[90 * 180 + 90] = 1;
  map_server::MapView changedMap(s, changed);
  TrajectorySafety changedSafety(changedMap, fp, .02);
  require(changedSafety.collisionCellCount(start) >
          now.collisionCellCount(start));
  std::cout << "PASS map version change does not reuse contacts\n";
}
int main(int argc, char **) {
  bool compare = true;
  (void)argc;
  std::mt19937 gen(543);
  size_t cases = 0;
  std::vector<double> oldtime, newtime;
  for (int mode = 0; mode < 6; ++mode)
    for (bool concave : {false, true}) {
      auto map = makeMap(mode);
      auto fp = body(concave);
      baseline::TrajectorySafety old(map, fp, .02);
      TrajectorySafety now(map, fp, .02);
      for (int j = 0; j < 120; ++j) {
        State a, b;
        a.x = .05 + (gen() % 4300) / 1000.;
        a.y = .05 + (gen() % 4300) / 1000.;
        a.yaw = (int(gen() % 6000) - 3000) / 1000.;
        b = a;
        b.x += (int(gen() % 100) - 50) / 1000.;
        b.y += (int(gen() % 100) - 50) / 1000.;
        b.yaw += (int(gen() % 200) - 100) / 1000.;
        if (j < 30) {
          a.x = 2.20 + j * .001;
          a.y = 2.25;
          a.yaw = 0;
          b = a;
          b.x += .01;
        }
        if (j == 30) {
          a.x = .24;
          a.y = .24;
          a.yaw = 0;
          b = a;
          b.x -= .001;
        }
        if (j == 31) {
          a.x = 4.24;
          a.y = 2.25;
          a.yaw = 0;
          b = a;
          b.x += .02;
        }
        if (j == 32) {
          a.x = 2.25;
          a.y = 2.25;
          a.yaw = M_PI / 4;
          b = a;
          b.yaw += .03;
        }
        std::unordered_set<size_t> ca, cb;
        int oa = old.collisionCellCount(a, &ca);
        if (compare) {
          require(oa == now.collisionCellCount(a, &cb));
          require(ca == cb);
          require(old.checkEscapeMotion(a, b, &ca) ==
                  now.checkEscapeMotion(a, b, &ca));
          require(accepted([&] { old.checkMotion(a, b, "test", &ca); }) ==
                  accepted([&] { now.checkMotion(a, b, "test", &ca); }));
        }
        std::vector<SafetyPlane> pa, pb;
        auto begin = Clock::now();
        bool va = old.sampleMotion(a, b, .02, pa, &ca);
        oldtime.push_back(
            std::chrono::duration<double, std::milli>(Clock::now() - begin)
                .count());
        begin = Clock::now();
        bool vb = now.sampleMotion(a, b, .02, pb, &ca);
        newtime.push_back(
            std::chrono::duration<double, std::milli>(Clock::now() - begin)
                .count());
        if (compare) {
          require(va == vb);
          planesEqual(pa, pb);
          Control cmd{.1, .03, .05};
          va = old.sampleEnvelope(a, cmd, .1, .02, .02, pa, &ca);
          vb = now.sampleEnvelope(a, cmd, .1, .02, .02, pb, &ca);
          require(va == vb);
          planesEqual(pa, pb);
        }
        ++cases;
      }
    }
  {
    auto invalidMap = makeMap(0);
    baseline::TrajectorySafety oldInvalid(invalidMap, body(false), .02);
    TrajectorySafety newInvalid(invalidMap, body(false), .02);
    State invalid;
    invalid.x = std::numeric_limits<double>::quiet_NaN();
    std::unordered_set<size_t> cells{5};
    require(oldInvalid.collisionCellCount(invalid) == -1 &&
            newInvalid.collisionCellCount(invalid, &cells) == -1 &&
            cells.empty());
  }
  cycleBench();
  stats("old sampleMotion", oldtime);
  stats("new sampleMotion", newtime);
  std::cout << "cases " << cases << "\n";
  auto map = makeMap(1);
  auto fp = body(false);
  baseline::TrajectorySafety old(map, fp, .02);
  RecoveryProfileSession session(true, [&](auto &p, bool) {
    for (auto m : {RecoveryMetric::SAMPLE_MOTION, RecoveryMetric::SQUARE,
                   RecoveryMetric::HULL, RecoveryMetric::APPEND,
                   RecoveryMetric::PROJECTION, RecoveryMetric::VISIT})
      std::cout << int(m) << " calls=" << p.get(m).calls
                << " ms=" << p.get(m).ms << "\n";
    std::cout << "obstacles=" << p.sample_obstacles
              << " planes=" << p.sample_planes
              << " comparisons=" << p.plane_comparisons
              << " duplicates=" << p.plane_duplicates << "\n";
  });
  for (int i = 0; i < 75; ++i) {
    State a, b;
    a.x = 2.2 + i * .0003;
    a.y = 2.25;
    b = a;
    b.x += .003;
    std::unordered_set<size_t> contacts;
    old.collisionCellCount(a, &contacts);
    std::vector<SafetyPlane> planes;
    old.sampleMotion(a, b, .02, planes, &contacts);
  }
}
