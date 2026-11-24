#include <gtest/gtest.h>
#include "nav_behaviors/translation_response.hpp"
using nav_behaviors::TranslationResponse;
TEST(TranslationResponse, LearnsBothSignsFromSettledMotion)
{
  for (double angle : {-.35, .35}) {
    TranslationResponse response;
    const double a = 1.1*std::cos(angle), b = 1.1*std::sin(angle);
    for (int i=0; i<100; ++i) {
      response.observe(.05*i, {.2, .05}, {a*.2-b*.05, b*.2+a*.05}, 0.);
    }
    EXPECT_NEAR(response.real,a,.001); EXPECT_NEAR(response.imag,b,.001);
    response.reset(); EXPECT_DOUBLE_EQ(response.real,1.); EXPECT_DOUBLE_EQ(response.imag,0.);
  }
}
TEST(TranslationResponse, DoesNotLearnStoppedDisconnectedOrRapidlyChangingCommands)
{
  TranslationResponse response;
  for (int i=0;i<100;++i) {response.observe(.05*i,{.2,0},{0,0},0);}
  EXPECT_DOUBLE_EQ(response.real,1.); EXPECT_DOUBLE_EQ(response.imag,0.);
  for (int i=100;i<200;++i) {response.observe(.05*i,{i%2 ? .1 : -.1,0},{0,.2},0);}
  EXPECT_DOUBLE_EQ(response.real,1.); EXPECT_DOUBLE_EQ(response.imag,0.);
}
TEST(TranslationResponse, ObservesInWorldFrameWhenTurretTurns)
{
  TranslationResponse response;
  for (int i=0;i<120;++i) {
    const double t=.05*i, yaw=.35*t;
    const nav_behaviors::XY command{.2*std::cos(yaw),-.2*std::sin(yaw)};
    response.observe(t,command,command,yaw);
  }
  EXPECT_NEAR(response.real,1.,1e-9); EXPECT_NEAR(response.imag,0.,1e-9);
}
