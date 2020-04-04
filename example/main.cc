#include <gf/Action.h>
#include <gf/Clock.h>
#include <gf/Color.h>
#include <gf/EntityContainer.h>
#include <gf/Event.h>
#include <gf/ModelContainer.h>
#include <gf/RenderWindow.h>
#include <gf/ViewContainer.h>
#include <gf/Views.h>
#include <gf/Window.h>

#include <gfb2d/PhysicsModel.h>
#include <gfb2d/PhysicsDebugger.h>

int main() {
  static constexpr gf::Vector2i ScreenSize(1280, 720);
  static constexpr gf::Vector2f ViewSize(1000.0f, 1000.0f);
  static constexpr gf::Vector2f ViewCenter(0.0f, 0.0f);

  // initialization

  gf::Window window("gf-box2d", ScreenSize);
  window.setVerticalSyncEnabled(true);
  window.setFramerateLimit(60);

  gf::RenderWindow renderer(window);

  // views

  gf::ViewContainer views;

  gf::ExtendView mainView(ViewCenter, ViewSize);
  views.addView(mainView);

  views.setInitialFramebufferSize(ScreenSize);

  // actions

  gf::ActionContainer actions;

  gf::Action closeWindowAction("Close window");
  closeWindowAction.addCloseControl();
  closeWindowAction.addKeycodeKeyControl(gf::Keycode::Escape);
  actions.addAction(closeWindowAction);

  gf::Action fullscreenAction("Fullscreen");
  fullscreenAction.addKeycodeKeyControl(gf::Keycode::F);
  actions.addAction(fullscreenAction);

  // models

  gf::ModelContainer models;

  gfb2d::PhysicsModel physics(0.01, gf::vec(0.0f, 20.0f));
  models.addModel(physics);

  // entities

  gf::EntityContainer mainEntities;

  gfb2d::PhysicsDebugger debug(physics);
  debug.setDebug(true);
  mainEntities.addEntity(debug);

  // initialize the scene

  for (int i = 0; i < 10; ++i) {
    auto body = physics.createSimpleBody(gf::vec(-400.0f + 80.0f * i + 20.0f, -400.0f));
    if (i % 2 == 0) {
      auto fixture = physics.createCircleFixture(body, 30.0f);
      fixture->SetRestitution(0.6f);
    } else {
      auto fixture = physics.createRectangleFixture(body, gf::vec(60.0f, 40.0f));
      fixture->SetRestitution(0.6f);
    }
  }

  gf::Polyline limit;
  limit.addPoint(gf::vec(-500.0f, 0.0f));
  limit.addPoint(gf::vec(0.0f, 500.0f));
  limit.addPoint(gf::vec(500.0f, 0.0f));
  physics.createPolylineFixture(physics.createSimpleBody(gf::vec(0.0f, 0.0f), 0.0f, gfb2d::BodyType::Static), limit);

  // game loop

  renderer.clear(gf::Color::Black);

  gf::Clock clock;

  while (window.isOpen()) {
    // 1. input

    gf::Event event;

    while (window.pollEvent(event)) {
      actions.processEvent(event);
      views.processEvent(event);
    }

    if (closeWindowAction.isActive()) {
      window.close();
    }

    if (fullscreenAction.isActive()) {
      window.toggleFullscreen();
    }


    // 2. update

    gf::Time time = clock.restart();
    models.update(time);
    mainEntities.update(time);


    // 3. draw

    renderer.clear();

    renderer.setView(mainView);
    mainEntities.render(renderer);

    renderer.display();

    actions.reset();
  }

  return 0;
}
