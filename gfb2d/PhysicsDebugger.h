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
#ifndef GF_BOX2D_PHYSICS_DEBUGGER_H
#define GF_BOX2D_PHYSICS_DEBUGGER_H

#include <vector>

#include <box2d/box2d.h>

#include <gf/Circ.h>
#include <gf/Entity.h>
#include <gf/Polygon.h>
#include <gf/Vector.h>

namespace gfb2d {
  class PhysicsModel;

  class PhysicsDebugger : public gf::Entity {
  public:
    PhysicsDebugger(PhysicsModel& model, float opacity = 0.8f);

    void setDebug(bool debug);
    void toggleDebug();

    void render(gf::RenderTarget& target, const gf::RenderStates& states) override;

  private:
    struct Polygon {
      gf::Polygon shape;
      gf::Color4f color;
    };

    struct Circle {
      gf::CircF shape;
      gf::Color4f color;
    };

    struct SolidCircle {
      gf::CircF shape;
      gf::Vector2f axis;
      gf::Color4f color;
    };

    struct Segment {
      gf::Vector2f p1;
      gf::Vector2f p2;
      gf::Color4f color;
    };

    struct Transform {
      gf::Vector2f position;
      gf::Vector2f xAxis;
      gf::Vector2f yAxis;
    };

    struct Point {
      gf::Vector2f position;
      float size;
      gf::Color4f color;
    };

    struct PhysicsDraw : public b2Draw {
    public:
      PhysicsDraw(float scale, float opacity);

      /// Draw a closed polygon provided in CCW order.
      void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;

      /// Draw a solid closed polygon provided in CCW order.
      void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;

      /// Draw a circle.
      void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) override;

      /// Draw a solid circle.
      void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) override;

      /// Draw a line segment.
      void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override;

      /// Draw a transform. Choose your own length scale.
      /// @param xf a transform.
      void DrawTransform(const b2Transform& xf) override;

      /// Draw a point.
      void DrawPoint(const b2Vec2& p, float size, const b2Color& color) override;

      std::vector<Polygon> polygons;
      std::vector<Polygon> solidPolygons;
      std::vector<Circle> circles;
      std::vector<SolidCircle> solidCircles;
      std::vector<Segment> segments;
      std::vector<Transform> transforms;
      std::vector<Point> points;

    private:
      gf::Vector2f toGameCoords(b2Vec2 coords);
      gf::Color4f toGameColor(b2Color color);

    private:
      float m_scale;
      float m_opacity;
    };


  private:
    bool m_debug;
    PhysicsModel *m_model;
    PhysicsDraw m_draw;
  };


}

#endif // GF_BOX2D_PHYSICS_DEBUGGER_H
