#include <gtest/gtest.h>
#include "nav_behaviors/translation_motion_model.hpp"

TEST(TranslationMotionModel, AngularSamplesCannotSteerPredictedFootprint)
{
  nav_behaviors::TranslationMotionModel model;
  mppi::models::ControlConstraints constraints;
  constraints.ax_max = constraints.ay_max = 2.0f;
  constraints.ax_min = constraints.ay_min = -2.0f;
  constraints.az_max = 4.0f;
  model.initialize(constraints, .05f);
  mppi::models::State state; state.reset(3, 8);
  state.speed.angular.z = .6;
  state.cvx.fill(.5f); state.cvy.fill(-.4f);
  // Opposite angular commands would normally produce different footprint sweeps.
  for (unsigned j=0;j<8;++j) {
    state.cwz(0,j)=-2.f; state.cwz(1,j)=0.f; state.cwz(2,j)=2.f;
    state.wz(0,j)=state.wz(1,j)=state.wz(2,j)=.6f;
  }
  model.predict(state);
  for (unsigned i=0;i<3;++i) {
    for (unsigned j=0;j<8;++j) {
      EXPECT_FLOAT_EQ(state.wz(i,j), .6f);
      EXPECT_FLOAT_EQ(state.cwz(i,j), 0.f);
      EXPECT_FLOAT_EQ(state.vx(i,j), state.vx(0,j));
      EXPECT_FLOAT_EQ(state.vy(i,j), state.vy(0,j));
    }
    EXPECT_NEAR(state.vx(i,1), .1, 1e-6);
    EXPECT_NEAR(state.vy(i,1), -.1, 1e-6);
  }
  state.speed.angular.z=0.;
  state.cwz.fill(2.f); model.predict(state);
  for (const auto wz:state.wz) {EXPECT_FLOAT_EQ(wz,0.f);}
}

TEST(TranslationMotionModel, OutputConstraintsPreserveXYAndRemoveYaw)
{
  nav_behaviors::TranslationMotionModel model;
  mppi::models::ControlSequence sequence; sequence.reset(8);
  sequence.vx.fill(.3f);sequence.vy.fill(-.2f);sequence.wz.fill(1.5f);
  model.applyConstraints(sequence);
  for (unsigned i=0;i<8;++i) {
    EXPECT_FLOAT_EQ(sequence.vx(i),.3f);
    EXPECT_FLOAT_EQ(sequence.vy(i),-.2f);
    EXPECT_FLOAT_EQ(sequence.wz(i),0.f);
  }
}

TEST(TranslationMotionModel, ResponseChangesCollisionRolloutButPreservesActuatorCommands)
{
  nav_behaviors::TranslationMotionModel model;
  mppi::models::ControlConstraints limits;
  limits.ax_max=limits.ay_max=2.f; limits.ax_min=limits.ay_min=-2.f; limits.az_max=4.f;
  model.initialize(limits,.05f); model.setResponse(1.05,-.3);
  mppi::models::State state;state.reset(2,5);
  state.cvx.fill(.2f);state.cvy.fill(.1f);
  state.vx.fill(.2f*1.05f+.1f*.3f);state.vy.fill(-.2f*.3f+.1f*1.05f);
  model.predict(state);
  for (size_t i=0;i<2;++i) {
    EXPECT_NEAR(state.vx(i,0),.24,1e-6);EXPECT_NEAR(state.vy(i,0),.045,1e-6);
    EXPECT_NEAR(state.vx(i,1),.24,1e-6);EXPECT_NEAR(state.vy(i,1),.045,1e-6);
    EXPECT_NEAR(state.cvx(i,0),.2,1e-6);EXPECT_NEAR(state.cvy(i,0),.1,1e-6);
  }
}
