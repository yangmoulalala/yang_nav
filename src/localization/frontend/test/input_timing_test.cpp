#include <gtest/gtest.h>

#include "zlio/input.hpp"

namespace zlio {
namespace {
class FlashScanTiming : public ::testing::Test {
protected:
    Config config;
    Input input{config, {}};

    void SetUp() override {
        for (int i = 0; i <= 50; ++i)
            input.addImu({nanoseconds(.01 * i), Vec3::Zero(), Vec3(0, 0, 9.81)});
        input.start(0);
    }

    Scan flash(double time) {
        return {{Vec3(1, 0, 0), nanoseconds(time)},
                {Vec3(1, 1, 0), nanoseconds(time)}};
    }
};

TEST_F(FlashScanTiming, UpdatesAtAcquisitionWithoutWaitingForNextScan) {
    input.addScan(flash(.1));
    ASSERT_TRUE(input.nextEnd(false));
    EXPECT_EQ(*input.nextEnd(false), nanoseconds(.1));
    EXPECT_EQ(input.points(*input.nextEnd(false)).size(), 2U);
}

TEST_F(FlashScanTiming, AdaptiveWindowDoesNotCreateUpdatesInScanGaps) {
    input.addScan(flash(.1));
    input.addScan(flash(.2));
    ASSERT_EQ(input.nextEnd(false), nanoseconds(.1));
    input.advance(nanoseconds(.1), .03);
    ASSERT_EQ(input.nextEnd(false), nanoseconds(.2));
    input.advance(nanoseconds(.2), .03);
    EXPECT_FALSE(input.nextEnd(false));
}

TEST_F(FlashScanTiming, ContinuousScanRetainsBoundedObservationWindows) {
    Scan scan;
    for (int i = 1; i <= 30; ++i)
        scan.push_back({Vec3(1, 0, 0), nanoseconds(.01 * i)});
    input.addScan(std::move(scan));
    ASSERT_EQ(input.nextEnd(false), nanoseconds(.12));
    input.advance(nanoseconds(.12), .06);
    ASSERT_EQ(input.nextEnd(false), nanoseconds(.18));
}
}  // namespace
}  // namespace zlio
