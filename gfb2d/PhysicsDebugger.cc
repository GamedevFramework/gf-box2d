/*
 * gf-box2d
 * Copyright (C) 2020 Julien Bernard
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#include "PhysicsDebugger.h"

#include <gf/Curves.h>
#include <gf/RenderTarget.h>
#include <gf/Shapes.h>

#include "PhysicsModel.h"

namespace gfb2d {

  namespace {
//     constexpr float DebugOutlineThickness = 1.0f;
    constexpr float DebugTransformLength = 0.5f;

    constexpr std::size_t DebugCirclePointCount = 23;
  }

  /*
   * PhysicsDebugger
   */

  PhysicsDebugger::PhysicsDebugger(PhysicsModel& model, float opacity)
  : gf::Entity(10000)
  , m_debug(false)
  , m_model(&model)
  , m_draw(model.getScale(), opacity)
  {
  }

  void PhysicsDebugger::setDebug(bool debug) {
    m_debug = debug;
    m_model->world.SetDebugDraw(m_debug ? &m_draw : nullptr);
  }

  void PhysicsDebugger::toggleDebug() {
    m_debug = !m_debug;
    m_model->world.SetDebugDraw(m_debug ? &m_draw : nullptr);
  }

  void PhysicsDebugger::render(gf::RenderTarget& target, const gf::RenderStates& states) {
    if (!m_debug) {
      return;
    }

    m_draw.shapes.clear();
    m_draw.lines.clear();
    m_model->world.DebugDraw();

    float thickness = target.getView().getSize().height * 0.002f;
    gf::RenderStates localStates = states;
    localStates.lineWidth = thickness;

    target.draw(m_draw.shapes, localStates);
    target.draw(m_draw.lines, localStates);
  }

  /*
   * PhysicsDraw
   */

  PhysicsDebugger::PhysicsDraw::PhysicsDraw(float scale, float opacity)
  : lines(gf::PrimitiveType::Lines)
  , m_scale(scale)
  , m_opacity(opacity)
  {
    SetFlags(b2Draw::e_shapeBit /* | b2Draw::e_aabbBit */ | b2Draw::e_pairBit | b2Draw::e_centerOfMassBit);
  }

  void PhysicsDebugger::PhysicsDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
    gf::Vertex line[2];
    line[0].color = line[1].color = toGameColor(color);

    for (int32 i = 0; i < vertexCount - 1; ++i) {
      line[0].position = toGameCoords(vertices[0]);
      line[1].position = toGameCoords(vertices[1]);
      lines.append(line[0]);
      lines.append(line[1]);
    }
  }

  void PhysicsDebugger::PhysicsDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
    gf::Polygon polygon;

    for (int32 i = 0; i < vertexCount; ++i) {
      polygon.addPoint(toGameCoords(vertices[i]));
    }

    shapes.addPolygon(polygon, toGameColor(color));
  }

  void PhysicsDebugger::PhysicsDraw::DrawCircle(const b2Vec2& center, float radius, const b2Color& color) {
    gf::Vertex line[2];
    line[0].color = line[1].color = toGameColor(color);

    gf::Vector2f gameCenter = toGameCoords(center);
    float gameRadius = radius / m_scale;

    for (std::size_t i = 0; i < DebugCirclePointCount - 1; ++i) {
      line[0].position = gameCenter + gameRadius * gf::unit( i      * 2.0f * gf::Pi / DebugCirclePointCount);
      line[1].position = gameCenter + gameRadius * gf::unit((i + 1) * 2.0f * gf::Pi / DebugCirclePointCount);
      lines.append(line[0]);
      lines.append(line[1]);
    }
  }

  void PhysicsDebugger::PhysicsDraw::DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) {
    gf::Vector2f gameCenter = toGameCoords(center);
    float gameRadius = radius / m_scale;
    gf::Color4f gameColor = toGameColor(color);

    shapes.addCircle(gameCenter, gameRadius, gameColor, DebugCirclePointCount);

    gf::Vertex line[2];
    line[0].color = line[1].color = gf::Color::darker(gameColor, 0.3f);
    line[0].position = gameCenter;
    line[1].position = gameCenter + toGameCoords(axis);
    lines.append(line[0]);
    lines.append(line[1]);
  }

  void PhysicsDebugger::PhysicsDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
    gf::Vertex line[2];
    line[0].color = line[1].color = toGameColor(color);
    line[0].position = toGameCoords(p1);
    line[1].position = toGameCoords(p2);
    lines.append(line[0]);
    lines.append(line[1]);
  }

  void PhysicsDebugger::PhysicsDraw::DrawTransform(const b2Transform& xf) {
    gf::Vector2f p = toGameCoords(xf.p);
    gf::Vector2f xAxis = toGameCoords(xf.q.GetXAxis());
    gf::Vector2f yAxis = toGameCoords(xf.q.GetYAxis());

    gf::Vertex line[2];

    line[0].color = line[1].color = gf::Color::Red;
    line[0].position = p;
    line[1].position = p + DebugTransformLength * xAxis;
    lines.append(line[0]);
    lines.append(line[1]);

    line[0].color = line[1].color = gf::Color::Green;
    line[0].position = p;
    line[1].position = p + DebugTransformLength * yAxis;
    lines.append(line[0]);
    lines.append(line[1]);
  }

  void PhysicsDebugger::PhysicsDraw::DrawPoint(const b2Vec2& p, float size, const b2Color& color) {
    shapes.addCircle(toGameCoords(p), size / 2, toGameColor(color));
  }

  gf::Vector2f PhysicsDebugger::PhysicsDraw::toGameCoords(b2Vec2 coords) {
    return gf::vec(coords.x / m_scale, coords.y / m_scale);
  }

  gf::Color4f PhysicsDebugger::PhysicsDraw::toGameColor(b2Color color) {
    return gf::vec(color.r, color.g, color.b, color.a * m_opacity);
  }


}

