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
#ifndef GF_BOX2D_PHYSICS_MODEL_H
#define GF_BOX2D_PHYSICS_MODEL_H

#include <cstdint>

#include <box2d/box2d.h>

#include <gf/Model.h>
#include <gf/Polygon.h>
#include <gf/Polyline.h>
#include <gf/Time.h>
#include <gf/Vector.h>

namespace gfb2d {

  enum class BodyType {
    Static,
    Kinematic,
    Dynamic,
  };

  enum class BodyFlags : uint64_t {
    None          = 0x00,
    Bullet        = 0x01,
  };

  constexpr
  BodyFlags operator|(BodyFlags lhs, BodyFlags rhs) {
    return static_cast<BodyFlags>(static_cast<uint64_t>(lhs) | static_cast<uint64_t>(rhs));
  }

  constexpr
  BodyFlags operator&(BodyFlags lhs, BodyFlags rhs) {
    return static_cast<BodyFlags>(static_cast<uint64_t>(lhs) & static_cast<uint64_t>(rhs));
  }

  enum class FixtureFlags : uint64_t {
    None          = 0x00,
    Sensor        = 0x01,
  };

  constexpr
  FixtureFlags operator|(FixtureFlags lhs, FixtureFlags rhs) {
    return static_cast<FixtureFlags>(static_cast<uint64_t>(lhs) | static_cast<uint64_t>(rhs));
  }

  constexpr
  FixtureFlags operator&(FixtureFlags lhs, FixtureFlags rhs) {
    return static_cast<FixtureFlags>(static_cast<uint64_t>(lhs) & static_cast<uint64_t>(rhs));
  }

  class PhysicsModel : public gf::Model {
  public:
    PhysicsModel(float scale, gf::Vector2f gravity, gf::Time timestep = gf::seconds(1 / 60.0f));
    ~PhysicsModel() = default;

    PhysicsModel(const PhysicsModel&) = delete;
    PhysicsModel(PhysicsModel&&) = delete;

    PhysicsModel& operator=(const PhysicsModel&) = delete;
    PhysicsModel& operator=(PhysicsModel&&) = delete;

    float getScale() const {
      return m_scale;
    }

    b2Body *createSimpleBody(gf::Vector2f position, float angle = 0.0f, BodyType type = BodyType::Dynamic, BodyFlags flags = BodyFlags::None);

    b2Fixture *createCircleFixture(b2Body *body, float radius, FixtureFlags flags = FixtureFlags::None);
    b2Fixture *createRectangleFixture(b2Body *body, gf::Vector2f size, FixtureFlags flags = FixtureFlags::None);
    b2Fixture *createPolylineFixture(b2Body *body, const gf::Polyline& polyline, FixtureFlags flags = FixtureFlags::None);
    b2Fixture *createPolygonFixture(b2Body *body, const gf::Polygon& polygon, FixtureFlags flags = FixtureFlags::None);

    void setBodyAngleAndVelocity(b2Body *body, float newAngle, float newVelocity);

    b2Vec2 computeGameToPhysicsCoordinates(gf::Vector2f coords) const;
    gf::Vector2f computePhysicsToGameCoordinates(b2Vec2 coords) const;

    void clear();

    void update(gf::Time time) override;

  private:
    float m_scale;
    gf::Time m_timestep;
    gf::Time m_elapsed;

  public:
    b2World world;
  };

}

#endif // GF_BOX2D_PHYSICS_MODEL_H
