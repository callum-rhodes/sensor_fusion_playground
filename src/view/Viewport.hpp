#pragma once
#include <cmath>
#include <vector>
#include "core/Types.hpp"

namespace sfsim {

struct Vec2 { double x = 0, y = 0; };

// Orthographic top-down camera: world x -> screen right, world y -> screen up.
// Drop z for position; use it for icon size / labels / colour.
struct Viewport {
  Vec2 pan{0, 0};        // world coords at screen centre
  double zoom = 0.1;     // pixels per metre
  Vec2 screen{1280, 720};

  Vec2 toScreen(const Vec3& w) const {
    return {screen.x * 0.5 + (w.x() - pan.x) * zoom,
            screen.y * 0.5 - (w.y() - pan.y) * zoom};
  }
  Vec3 toWorld(Vec2 s) const {
    return {pan.x + (s.x - screen.x * 0.5) / zoom, pan.y - (s.y - screen.y * 0.5) / zoom, 0.0};
  }
  void zoomAbout(Vec2 s, double factor) {  // mouse-wheel zoom keeps the cursor point fixed
    const Vec3 before = toWorld(s);
    zoom *= factor;
    const Vec3 after = toWorld(s);
    pan.x += before.x() - after.x();
    pan.y += before.y() - after.y();
  }
  static double altitudeToRadius(double z, double base = 5.0) { return base * (1.0 + z / 5000.0); }

  // Top-down projection of a sensor cone: sector polygon in screen space
  std::vector<Vec2> fovSector(const Vec3& pos, double boresight_yaw, double half_angle,
                              double range, int segments = 24) const {
    std::vector<Vec2> poly{toScreen(pos)};
    for (int i = 0; i <= segments; ++i) {
      const double a = boresight_yaw - half_angle + 2 * half_angle * i / segments;
      poly.push_back(toScreen(pos + Vec3(range * std::cos(a), range * std::sin(a), 0)));
    }
    return poly;
  }
};

}  // namespace sfsim
