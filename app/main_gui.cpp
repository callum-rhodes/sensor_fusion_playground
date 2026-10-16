// SDL2 + OpenGL3 + Dear ImGui front end. Reads Simulation::snapshot() only;
// the only writes into the sim are the live sensor-config sliders.
#define GL_SILENCE_DEPRECATION
#define _USE_MATH_DEFINES
#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <unordered_map>
#include <unordered_set>

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl2.h"

#include "sim/Simulation.hpp"
#include "tracking/CentralizedTracker.hpp"
#include "view/Viewport.hpp"

using namespace sfsim;

// ---------------------------------------------------------------- scenario
static void addTarget(Simulation& sim, TargetId id, Vec3 pos, Vec3 vel,
                      std::unique_ptr<TrajectoryModel> m, double birth = 0, double death = 1e18) {
  Target t;
  t.id = id; t.state.pos = pos; t.state.vel = vel; t.model = std::move(m);
  t.birth_time = birth; t.death_time = death;
  sim.addTarget(std::move(t));
}

static double sensorYaw(const SensorConfig& c) {
  const Vec3 b = c.pose.orientation * Vec3::UnitX();
  return std::atan2(b.y(), b.x());
}

static Simulation makeScenario(uint64_t seed) {
  Simulation sim({.dt = 0.1, .seed = seed});

  auto addSensor = [&](SensorId id, MeasType type, Vec3 pos, double yaw, double rate) {
    SensorConfig sc;
    sc.id = id; sc.pose.position = pos;
    sc.measType = type;
    sc.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
    sc.fov_half_angle = 0.95; sc.max_range = 6500; sc.update_rate_hz = rate; sc.sigma = Vec3(1, 0.01, 0.01);
    sim.addSensor(sc);
  };
  addSensor(0, MeasType::RangeAzEl, Vec3(-3000, -2000, 30), 0.6, 2.0);
  addSensor(1, MeasType::RangeAzEl, Vec3(3000, -2000, 30), M_PI - 0.6, 2.0);
  addSensor(2, MeasType::RangeAzEl, Vec3(0, 3500, 30), -M_PI / 2, 1.0);

  addTarget(sim, 1, Vec3(-1500, -500, 0), Vec3(70, 15, 0), std::make_unique<ConstantVelocity>());
  addTarget(sim, 2, Vec3(1500, 1500, 0), Vec3(-60, -10, 0), std::make_unique<CoordinatedTurn>(0.04));
  addTarget(sim, 3, Vec3(0, -1000, 0), Vec3(0, 40, 5), std::make_unique<CoordinatedTurn>(-0.03));
  addTarget(sim, 4, Vec3(-2500, 2000, 0), Vec3(60, -20, 0), std::make_unique<ConstantVelocity>(), 20.0, 90.0);

  sim.setFusion(std::make_unique<CentralizedTracker>());
  return sim;
}

// ---------------------------------------------------------------- app state
struct Blip { double t; Vec3 p; SensorId sensor; bool clutter; };

struct App {
  int seed = 1;
  Simulation sim = makeScenario(1);
  Viewport vp;
  bool running = true;
  float speed = 1.0f;
  double acc = 0;

  bool showGrid = true, showFov = true, showTruth = true, showMeas = true, showTracks = true,
       showTentative = false, showCov = true, showTrails = true, debugClutter = false;
  float measLife = 3.0f;

  std::unordered_map<uint32_t, std::deque<Vec3>> truthTrail, trackTrail;
  std::deque<Blip> blips;

  App() { vp.pan = {0, 500}; vp.zoom = 0.08; }

  void reset() {
    sim = makeScenario((uint64_t)seed);
    truthTrail.clear(); trackTrail.clear(); blips.clear();
    acc = 0;
  }

  void stepOnce() {
    sim.step();
    const Snapshot& s = sim.snapshot();
    constexpr size_t kTrail = 400;

    std::unordered_set<uint32_t> liveTruth, liveTracks;
    for (auto& [id, st] : s.truth) {
      auto& q = truthTrail[id]; q.push_back(st.pos);
      if (q.size() > kTrail) q.pop_front();
      liveTruth.insert(id);
    }
    for (auto& tr : s.tracks) {
      if (!tr.confirmed) continue;
      auto& q = trackTrail[tr.id]; q.push_back(tr.x.head<3>());
      if (q.size() > kTrail) q.pop_front();
      liveTracks.insert(tr.id);
    }
    std::erase_if(truthTrail, [&](auto& kv) { return !liveTruth.count(kv.first); });
    std::erase_if(trackTrail, [&](auto& kv) { return !liveTracks.count(kv.first); });

    for (auto& m : s.measurements){
      // Get the sensor that produced this measurement, and convert the measurement into world coordinates.
      auto& sensor = sim.sensors()[m.sensor];
      blips.push_back({s.time, sensor.model().Meas2World(m.pose.position, m.pose.orientation, m.z), m.sensor, m.is_clutter});
    } 

    while (!blips.empty() && s.time - blips.front().t > 10.0) blips.pop_front();
  }

  void fitView() {
    Vec3 lo(1e18, 1e18, 0), hi(-1e18, -1e18, 0);
    auto grow = [&](const Vec3& p) {
      lo.x() = std::min(lo.x(), p.x()); lo.y() = std::min(lo.y(), p.y());
      hi.x() = std::max(hi.x(), p.x()); hi.y() = std::max(hi.y(), p.y());
    };
    for (auto& s : sim.sensors()) grow(s.config().pose.position);
    for (auto& [id, st] : sim.snapshot().truth) grow(st.pos);
    if (lo.x() > hi.x()) return;
    vp.pan = {(lo.x() + hi.x()) / 2, (lo.y() + hi.y()) / 2};
    const double ext = std::max({hi.x() - lo.x(), hi.y() - lo.y(), 1000.0}) * 1.3;
    vp.zoom = std::min(vp.screen.x, vp.screen.y) / ext;
  }
};

// ---------------------------------------------------------------- drawing
static const ImU32 kSensorCol[] = {IM_COL32(90, 170, 255, 255), IM_COL32(255, 120, 200, 255),
                                   IM_COL32(120, 230, 230, 255), IM_COL32(200, 160, 255, 255)};
static ImU32 withAlpha(ImU32 c, int a) { return (c & 0x00FFFFFF) | ((ImU32)a << 24); }

static ImVec2 scr(const Viewport& vp, const Vec3& w) {
  Vec2 s = vp.toScreen(w);
  return ImVec2((float)s.x, (float)s.y);
}

static void drawTrail(ImDrawList* dl, const Viewport& vp, const std::deque<Vec3>& q, ImU32 col) {
  for (size_t i = 1; i < q.size(); ++i) {
    const int a = (int)(30 + 150.0 * i / q.size());
    dl->AddLine(scr(vp, q[i - 1]), scr(vp, q[i]), withAlpha(col, a), 1.2f);
  }
}

static void drawGrid(ImDrawList* dl, const Viewport& vp) {
  const double steps[] = {100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000};
  double step = steps[8];
  for (double s : steps) if (s * vp.zoom >= 70) { step = s; break; }
  const Vec3 tl = vp.toWorld({0, 0}), br = vp.toWorld(vp.screen);
  char buf[32];
  for (double x = std::floor(tl.x() / step) * step; x <= br.x(); x += step) {
    const float sx = scr(vp, Vec3(x, 0, 0)).x;
    dl->AddLine(ImVec2(sx, 0), ImVec2(sx, (float)vp.screen.y),
                x == 0 ? IM_COL32(255, 255, 255, 50) : IM_COL32(255, 255, 255, 18));
    std::snprintf(buf, sizeof buf, "%.0f m", x);
    dl->AddText(ImVec2(sx + 3, 3), IM_COL32(255, 255, 255, 90), buf);
  }
  for (double y = std::floor(br.y() / step) * step; y <= tl.y(); y += step) {
    const float sy = scr(vp, Vec3(0, y, 0)).y;
    dl->AddLine(ImVec2(0, sy), ImVec2((float)vp.screen.x, sy),
                y == 0 ? IM_COL32(255, 255, 255, 50) : IM_COL32(255, 255, 255, 18));
    std::snprintf(buf, sizeof buf, "%.0f m", y);
    dl->AddText(ImVec2(3, sy - 14), IM_COL32(255, 255, 255, 90), buf);
  }
}

static void drawCovariance(ImDrawList* dl, const Viewport& vp, const Track& t, ImU32 col) {
  Eigen::Matrix2d C = t.P.topLeftCorner<2, 2>();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
  const double a = 2.0 * std::sqrt(std::max(es.eigenvalues()(1), 0.0));
  const double b = 2.0 * std::sqrt(std::max(es.eigenvalues()(0), 0.0));
  const Eigen::Vector2d u = es.eigenvectors().col(1), v = es.eigenvectors().col(0);
  ImVec2 pts[33];
  for (int i = 0; i < 33; ++i) {
    const double th = 2.0 * M_PI * i / 32;
    const Eigen::Vector2d d = u * (a * std::cos(th)) + v * (b * std::sin(th));
    pts[i] = scr(vp, Vec3(t.x.x() + d.x(), t.x.y() + d.y(), 0));
  }
  dl->AddPolyline(pts, 33, col, ImDrawFlags_None, 1.0f);
}

static void drawScene(App& app) {
  ImDrawList* dl = ImGui::GetBackgroundDrawList();
  const Viewport& vp = app.vp;
  const Snapshot& snap = app.sim.snapshot();
  char buf[64];

  if (app.showGrid) drawGrid(dl, vp);

  // Sensors + top-down FOV sectors
  for (auto& s : app.sim.sensors()) {
    const SensorConfig& c = s.config();
    const ImU32 col = kSensorCol[c.id % 4];
    if (app.showFov) {
      const auto poly = vp.fovSector(c.pose.position, sensorYaw(c), c.fov_half_angle, c.max_range);
      std::vector<ImVec2> pts;
      for (auto& p : poly) pts.push_back(ImVec2((float)p.x, (float)p.y));
      if (c.fov_half_angle <= M_PI / 2)  // convex only; wider cones get an outline
        dl->AddConvexPolyFilled(pts.data(), (int)pts.size(), withAlpha(col, 22));
      dl->AddPolyline(pts.data(), (int)pts.size(), withAlpha(col, 110), ImDrawFlags_Closed, 1.0f);
    }
    const ImVec2 p = scr(vp, c.pose.position);
    dl->AddRectFilled(ImVec2(p.x - 5, p.y - 5), ImVec2(p.x + 5, p.y + 5), col);
    std::snprintf(buf, sizeof buf, "S%u", c.id);
    dl->AddText(ImVec2(p.x + 8, p.y - 7), col, buf);
  }

  // Trails
  if (app.showTrails) {
    if (app.showTruth)
      for (auto& [id, q] : app.truthTrail) drawTrail(dl, vp, q, IM_COL32(80, 220, 120, 255));
    if (app.showTracks)
      for (auto& [id, q] : app.trackTrail) drawTrail(dl, vp, q, IM_COL32(255, 200, 60, 255));
  }

  // Measurements, fading with age
  if (app.showMeas) {
    for (auto& b : app.blips) {
      const double age = snap.time - b.t;
      if (age > app.measLife) continue;
      const int a = (int)(255 * (1.0 - age / app.measLife));
      ImU32 col = kSensorCol[b.sensor % 4];
      if (app.debugClutter && b.clutter) col = IM_COL32(170, 170, 170, 255);
      const ImVec2 p = scr(vp, b.p);
      dl->AddLine(ImVec2(p.x - 3, p.y - 3), ImVec2(p.x + 3, p.y + 3), withAlpha(col, a), 1.5f);
      dl->AddLine(ImVec2(p.x - 3, p.y + 3), ImVec2(p.x + 3, p.y - 3), withAlpha(col, a), 1.5f);
    }
  }

  // Tracks
  if (app.showTracks) {
    for (auto& t : snap.tracks) {
      if (!t.confirmed && !app.showTentative) continue;
      const ImU32 col = t.confirmed ? IM_COL32(255, 200, 60, 255) : IM_COL32(150, 150, 150, 200);
      const Vec3 pos = t.x.head<3>(), vel = t.x.tail<3>();
      const ImVec2 p = scr(vp, pos);
      const float r = (float)Viewport::altitudeToRadius(pos.z(), 7.0) + 3.0f;
      dl->AddCircle(p, r, col, 24, 1.5f);
      dl->AddLine(p, scr(vp, pos + Vec3(vel.x(), vel.y(), 0) * 5.0), col, 1.5f);  // 5 s lead
      if (app.showCov && t.confirmed) drawCovariance(dl, vp, t, withAlpha(col, 120));
      std::snprintf(buf, sizeof buf, "#%u", t.id);
      dl->AddText(ImVec2(p.x + r + 3, p.y + 2), col, buf);
    }
  }

  // Truth
  if (app.showTruth) {
    const ImU32 col = IM_COL32(80, 220, 120, 255);
    for (auto& [id, st] : snap.truth) {
      const ImVec2 p = scr(vp, st.pos);
      dl->AddCircleFilled(p, (float)Viewport::altitudeToRadius(st.pos.z(), 4.0), col, 16);
      std::snprintf(buf, sizeof buf, "T%u z=%.0f m", id, st.pos.z());
      dl->AddText(ImVec2(p.x + 8, p.y - 16), col, buf);
    }
  }

  // FPS counter, bottom-right corner of the main view
  {
    const ImGuiIO& io = ImGui::GetIO();
    char fps[48];
    std::snprintf(fps, sizeof fps, "%.0f FPS  (%.1f ms)", io.Framerate, 1000.0f / io.Framerate);
    const ImVec2 sz = ImGui::CalcTextSize(fps);
    dl->AddText(ImVec2((float)vp.screen.x - sz.x - 10.0f, (float)vp.screen.y - sz.y - 8.0f),
                IM_COL32(255, 255, 255, 140), fps);
  }
}

// ---------------------------------------------------------------- UI panels
static void drawControls(App& app) {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(420, 600), ImGuiCond_FirstUseEver);
  ImGui::Begin("Controls");
  ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);

  if (ImGui::Button(app.running ? "Pause" : "Resume")) app.running = !app.running;
  ImGui::SameLine();
  if (ImGui::Button("Step")) { app.running = false; app.stepOnce(); }
  ImGui::SameLine();
  if (ImGui::Button("Reset")) app.reset();
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) app.fitView();
  ImGui::SliderFloat("speed", &app.speed, 0.1f, 20.0f, "%.1fx", ImGuiSliderFlags_Logarithmic);
  ImGui::InputInt("seed (on Reset)", &app.seed);
  ImGui::Text("sim time: %.1f s", app.sim.time());

  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("grid", &app.showGrid); ImGui::SameLine();
    ImGui::Checkbox("sensor FOV", &app.showFov);
    ImGui::Checkbox("truth", &app.showTruth); ImGui::SameLine();
    ImGui::Checkbox("measurements", &app.showMeas);
    ImGui::Checkbox("tracks", &app.showTracks); ImGui::SameLine();
    ImGui::Checkbox("tentative", &app.showTentative);
    ImGui::Checkbox("covariance (2 sigma)", &app.showCov); ImGui::SameLine();
    ImGui::Checkbox("trails", &app.showTrails);
    ImGui::Checkbox("grey out clutter (uses ground truth)", &app.debugClutter);
    ImGui::SliderFloat("measurement fade (s)", &app.measLife, 0.5f, 10.0f);
    float zoomLog = (float)std::log10(app.vp.zoom);
    if (ImGui::SliderFloat("zoom (log px/m)", &zoomLog, -3.0f, 1.0f))
      app.vp.zoom = std::pow(10.0, (double)zoomLog);
    ImGui::TextDisabled("Wheel = zoom, left/middle drag = pan, Space = pause");
  }

  if (ImGui::CollapsingHeader("Sensors", ImGuiTreeNodeFlags_DefaultOpen)) {
    for (auto& s : app.sim.sensors()) {
      SensorConfig& c = s.config();
      char label[32];
      std::snprintf(label, sizeof label, "Sensor %u", c.id);
      if (!ImGui::TreeNodeEx(label, ImGuiTreeNodeFlags_DefaultOpen)) continue;
      ImGui::PushID((int)c.id);

      float sg[3] = {(float)c.sigma.x(), (float)c.sigma.y(), (float)c.sigma.z()};
      if (ImGui::SliderFloat3("noise sigma xyz (m)", sg, 0.01f, 150.0f, "%.2f", ImGuiSliderFlags_Logarithmic))
        c.sigma = Vec3(sg[0], sg[1], sg[2]);
      float pd = (float)c.p_detect;
      if (ImGui::SliderFloat("P(detect)", &pd, 0.0f, 1.0f)) c.p_detect = pd;
      float cl = (float)c.clutter_rate;
      if (ImGui::SliderFloat("clutter / scan", &cl, 0.0f, 40.0f)) c.clutter_rate = cl;
      float rate = (float)c.update_rate_hz;
      if (ImGui::SliderFloat("update rate (Hz)", &rate, 0.2f, 10.0f)) c.update_rate_hz = rate;
      float fov = (float)(c.fov_half_angle * 180.0 / M_PI);
      if (ImGui::SliderFloat("FOV half-angle (deg)", &fov, 5.0f, 90.0f)) c.fov_half_angle = fov * M_PI / 180.0;
      float rng = (float)c.max_range;
      if (ImGui::SliderFloat("max range (m)", &rng, 500.0f, 15000.0f)) c.max_range = rng;
      float yaw = (float)(sensorYaw(c) * 180.0 / M_PI);
      if (ImGui::SliderFloat("heading (deg)", &yaw, -180.0f, 180.0f))
        c.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw * M_PI / 180.0, Vec3::UnitZ()));

      ImGui::PopID();
      ImGui::TreePop();
    }
  }
  ImGui::PopItemWidth();
  ImGui::End();
}

static void drawTelemetry(App& app) {
  const Snapshot& snap = app.sim.snapshot();
  ImGui::SetNextWindowPos(ImVec2(440, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(560, 260), ImGuiCond_FirstUseEver);
  ImGui::Begin("Telemetry");

  int confirmed = 0;
  for (auto& t : snap.tracks) confirmed += t.confirmed;
  ImGui::Text("truth: %zu   measurements (this tick): %zu   tracks: %d confirmed / %zu total",
              snap.truth.size(), snap.measurements.size(), confirmed, snap.tracks.size());

  if (ImGui::BeginTable("tracks", 8, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    for (const char* h : {"ID", "x (m)", "y (m)", "z (m)", "speed", "hits", "sigma pos", "err to truth"})
      ImGui::TableSetupColumn(h);
    ImGui::TableHeadersRow();
    for (auto& t : snap.tracks) {
      if (!t.confirmed) continue;
      const Vec3 pos = t.x.head<3>();
      double err = 1e18;  // scoring only: nearest ground-truth distance
      for (auto& [id, st] : snap.truth) err = std::min(err, (st.pos - pos).norm());
      ImGui::TableNextRow();
      ImGui::TableNextColumn(); ImGui::Text("#%u", t.id);
      ImGui::TableNextColumn(); ImGui::Text("%.0f", pos.x());
      ImGui::TableNextColumn(); ImGui::Text("%.0f", pos.y());
      ImGui::TableNextColumn(); ImGui::Text("%.0f", pos.z());
      ImGui::TableNextColumn(); ImGui::Text("%.1f", t.x.tail<3>().norm());
      ImGui::TableNextColumn(); ImGui::Text("%d", t.hits);
      ImGui::TableNextColumn(); ImGui::Text("%.1f", std::sqrt(t.P.topLeftCorner<3, 3>().trace() / 3.0));
      ImGui::TableNextColumn();
      if (err < 1e17) ImGui::Text("%.1f", err); else ImGui::TextDisabled("-");
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

// ---------------------------------------------------------------- main
int main(int, char**) {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }
#ifdef __APPLE__
  const char* glsl = "#version 150";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
  const char* glsl = "#version 330";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#endif
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_Window* window = SDL_CreateWindow("Sensor fusion sim", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                        1400, 850, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) { std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError()); return 1; }
  SDL_GLContext gl = SDL_GL_CreateContext(window);
  if (!gl) { std::fprintf(stderr, "GL context failed: %s\n", SDL_GetError()); return 1; }
  SDL_GL_MakeCurrent(window, gl);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGui::StyleColorsDark();
  ImGui_ImplSDL2_InitForOpenGL(window, gl);
  ImGui_ImplOpenGL3_Init(glsl);

  App app;
  app.fitView();
  bool quit = false;

  while (!quit) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      ImGui_ImplSDL2_ProcessEvent(&e);
      if (e.type == SDL_QUIT) quit = true;
      if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_CLOSE &&
          e.window.windowID == SDL_GetWindowID(window)) quit = true;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(),
                                ImGuiDockNodeFlags_PassthruCentralNode);
    app.vp.screen = {io.DisplaySize.x, io.DisplaySize.y};

    // Camera input (only when ImGui isn't using the mouse)
    if (!io.WantCaptureMouse) {
      if (io.MouseWheel != 0.0f)
        app.vp.zoomAbout({io.MousePos.x, io.MousePos.y}, std::pow(1.15, (double)io.MouseWheel));
      if (io.MouseDown[0] || io.MouseDown[2]) {
        app.vp.pan.x -= io.MouseDelta.x / app.vp.zoom;
        app.vp.pan.y += io.MouseDelta.y / app.vp.zoom;
      }
    }
    if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false)) app.running = !app.running;

    // Advance the sim at (real time * speed), capped so a slow frame can't spiral
    if (app.running) {
      app.acc += std::min((double)io.DeltaTime, 0.1) * app.speed;
      int n = 0;
      const double dt = 0.1;  // matches SimConfig.dt in makeScenario
      while (app.acc >= dt && n < 100) { app.stepOnce(); app.acc -= dt; ++n; }
      if (n == 100) app.acc = 0;
    }

    drawScene(app);
    drawControls(app);
    drawTelemetry(app);

    ImGui::Render();
    glViewport(0, 0, (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x),
               (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y));
    glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(gl);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
