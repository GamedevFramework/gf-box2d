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
#include "PhysicsModel.h"

#include <cassert>

namespace gfb2d {

  namespace {

    b2BodyType toBodyType(BodyType type) {
      switch (type) {
        case BodyType::Static:
          return b2_staticBody;
        case BodyType::Kinematic:
          return b2_kinematicBody;
        case BodyType::Dynamic:
          return b2_dynamicBody;
      }

      assert(false);
      return b2_dynamicBody;
    }

    template<typename T>
    b2Fixture *createFixture(b2Body *body, T& shape, FixtureFlags flags) {
      b2FixtureDef def;
      def.isSensor = (flags & FixtureFlags::Sensor) != FixtureFlags::None;
      def.density = 1.0f;
      def.friction = 0.0f;
      def.restitution = 0.0f;
      def.shape = &shape;

      return body->CreateFixture(&def);
    }
  }

  PhysicsModel::PhysicsModel(float scale, gf::Vector2f gravity, gf::Time timestep)
  : m_scale(scale)
  , m_timestep(timestep)
  , m_elapsed() // 0
  , world(computeGameToPhysicsCoordinates(gravity))
  {
  }

  b2Body *PhysicsModel::createSimpleBody(gf::Vector2f position, float angle, BodyType type, BodyFlags flags) {
    b2BodyDef def;
    def.type = toBodyType(type);
    def.position = computeGameToPhysicsCoordinates(position);
    def.angle = angle;
    def.bullet = (flags & BodyFlags::Bullet) != BodyFlags::None;
    return world.CreateBody(&def);
  }

  b2Fixture *PhysicsModel::createCircleFixture(b2Body *body, float radius, FixtureFlags flags) {
    b2CircleShape shape;
    shape.m_radius = radius * m_scale;
    return createFixture(body, shape, flags);
  }

  b2Fixture *PhysicsModel::createRectangleFixture(b2Body *body, gf::Vector2f size, FixtureFlags flags) {
    b2PolygonShape shape;
    shape.SetAsBox(size.width * m_scale / 2.0f, size.height * m_scale / 2.0f);
    return createFixture(body, shape, flags);
  }

  b2Fixture *PhysicsModel::createPolylineFixture(b2Body *body, const gf::Polyline& polyline, FixtureFlags flags) {
    std::vector<b2Vec2> line;

    for (auto& point : polyline) {
      line.emplace_back(computeGameToPhysicsCoordinates(point));
    }

    b2ChainShape shape;

    if (polyline.isLoop()) {
      shape.CreateLoop(line.data(), line.size());
    } else {
      assert(polyline.isChain());
      shape.CreateChain(line.data(), line.size());
    }

    return createFixture(body, shape, flags);
  }

  b2Fixture *PhysicsModel::createPolygonFixture(b2Body *body, const gf::Polygon& polygon, FixtureFlags flags) {
    std::vector<b2Vec2> line;

    for (auto& point : polygon) {
      line.emplace_back(computeGameToPhysicsCoordinates(point));
    }

    b2PolygonShape shape;
    shape.Set(line.data(), line.size());
    return createFixture(body, shape, flags);
  }

  b2Vec2 PhysicsModel::computeGameToPhysicsCoordinates(gf::Vector2f coords) const {
    return { coords.x * m_scale, coords.y * m_scale };
  }

  gf::Vector2f PhysicsModel::computePhysicsToGameCoordinates(b2Vec2 coords) const {
    return gf::vec(coords.x / m_scale, coords.y / m_scale);
  }

  void PhysicsModel::clear() {
    b2Body *current = world.GetBodyList();

    while (current != nullptr) {
      b2Body *next = current->GetNext();
      world.DestroyBody(current);
      current = next;
    }
  }

  void PhysicsModel::update(gf::Time time) {
    static constexpr gf::Time MaxElapsed = gf::seconds(0.5f);
    static constexpr int32 VelocityIterations = 10; // 6;
    static constexpr int32 PositionIterations = 8; // 2;

    m_elapsed += time;

    if (m_elapsed > MaxElapsed) {
      m_elapsed = MaxElapsed;
    }

    while (m_elapsed > m_timestep) {
      world.Step(m_timestep.asSeconds(), VelocityIterations, PositionIterations);
      m_elapsed -= m_timestep;
    }
  }

}
