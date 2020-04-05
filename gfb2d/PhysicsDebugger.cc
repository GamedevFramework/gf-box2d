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
    m_model->world.DrawDebugData();

    float thickness = target.getView().getSize().height * 0.002f;

    for (auto& polygon : m_draw.polygons) {
      gf::ConvexShape shape(polygon.shape);
      shape.setColor(gf::Color::Transparent);
      shape.setOutlineColor(polygon.color);
      shape.setOutlineThickness(thickness);
      target.draw(shape, states);
    }

    for (auto& polygon : m_draw.solidPolygons) {
      gf::ConvexShape shape(polygon.shape);
      shape.setColor(polygon.color);
      target.draw(shape, states);
    }

    for (auto& circle : m_draw.circles) {
      gf::CircleShape shape(circle.shape);
      shape.setColor(gf::Color::Transparent);
      shape.setOutlineColor(circle.color);
      shape.setOutlineThickness(thickness);
      target.draw(shape, states);
    }

    for (auto& circle : m_draw.solidCircles) {
      gf::CircleShape shape(circle.shape);
      shape.setColor(circle.color);
      target.draw(shape, states);

      gf::Line line(circle.shape.center, circle.shape.center + circle.axis);
      line.setWidth(2.5f * thickness);
      line.setColor(gf::Color::darker(circle.color, 0.3f));
      target.draw(line, states);
    }

    for (auto& segment : m_draw.segments) {
      gf::Line curve(segment.p1, segment.p2);
      curve.setWidth(thickness);
      curve.setColor(segment.color);
      target.draw(curve, states);
    }

    for (auto& transform : m_draw.transforms) {
      gf::Line lineX(transform.position, transform.position + DebugTransformLength * transform.xAxis);
      lineX.setWidth(thickness);
      lineX.setColor(gf::Color::Red);
      target.draw(lineX, states);

      gf::Line lineY(transform.position, transform.position + DebugTransformLength * transform.yAxis);
      lineY.setWidth(thickness);
      lineY.setColor(gf::Color::Green);
      target.draw(lineY, states);
    }

    for (auto& point : m_draw.points) {
      gf::CircleShape circle(point.size / 2);
      circle.setColor(point.color);
      circle.setAnchor(gf::Anchor::Center);
      circle.setPosition(point.position);
      target.draw(circle, states);
    }

    m_draw.polygons.clear();
    m_draw.solidPolygons.clear();
    m_draw.circles.clear();
    m_draw.solidCircles.clear();
    m_draw.segments.clear();
    m_draw.transforms.clear();
    m_draw.points.clear();
  }

  /*
   * PhysicsDraw
   */

  PhysicsDebugger::PhysicsDraw::PhysicsDraw(float scale, float opacity)
  : m_scale(scale)
  , m_opacity(opacity)
  {
    SetFlags(b2Draw::e_shapeBit /* | b2Draw::e_aabbBit */ | b2Draw::e_pairBit | b2Draw::e_centerOfMassBit);
  }

  void PhysicsDebugger::PhysicsDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
    gf::Polygon polygon;

    for (int32 i = 0; i < vertexCount; ++i) {
      polygon.addPoint(toGameCoords(vertices[i]));
    }

    polygons.push_back({ std::move(polygon), toGameColor(color) });
  }

  void PhysicsDebugger::PhysicsDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
    gf::Polygon polygon;

    for (int32 i = 0; i < vertexCount; ++i) {
      polygon.addPoint(toGameCoords(vertices[i]));
    }

    solidPolygons.push_back({ std::move(polygon), toGameColor(color) });
  }

  void PhysicsDebugger::PhysicsDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) {
    circles.push_back({ gf::CircF(toGameCoords(center), radius / m_scale), toGameColor(color) });
  }

  void PhysicsDebugger::PhysicsDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) {
    solidCircles.push_back({ gf::CircF(toGameCoords(center), radius / m_scale), toGameCoords(axis), toGameColor(color) });
  }

  void PhysicsDebugger::PhysicsDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
    segments.push_back({ toGameCoords(p1), toGameCoords(p2), toGameColor(color) });
  }

  void PhysicsDebugger::PhysicsDraw::DrawTransform(const b2Transform& xf) {
    transforms.push_back({ toGameCoords(xf.p), toGameCoords(xf.q.GetXAxis()), toGameCoords(xf.q.GetYAxis()) });
  }

  void PhysicsDebugger::PhysicsDraw::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color) {
    points.push_back({ toGameCoords(p), size, toGameColor(color) });
  }

  gf::Vector2f PhysicsDebugger::PhysicsDraw::toGameCoords(b2Vec2 coords) {
    return gf::vec(coords.x / m_scale, coords.y / m_scale);
  }

  gf::Color4f PhysicsDebugger::PhysicsDraw::toGameColor(b2Color color) {
    return gf::vec(color.r, color.g, color.b, color.a * m_opacity);
  }


}

