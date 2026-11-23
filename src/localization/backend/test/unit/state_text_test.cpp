/** @file state_text_test.cpp @brief 检查定位原因摘要的明细省略、长度边界及 UTF-8 完整性。 */
#include <gtest/gtest.h>

#include "apps/state_text.hpp"

namespace backend::ros {
namespace {

TEST(StateText, KeepsShortReasonsAndEmptyInput) {
    EXPECT_EQ(summarizeStateReason("recovered via prior map"), "recovered via prior map");
    EXPECT_EQ(summarizeStateReason(""), "");
}

TEST(StateText, RemovesRegistrationDetailsAndAdditionalLines) {
    EXPECT_EQ(summarizeStateReason(
                  "observation model invalid: prior scan-to-map cannot construct a pose likelihood: "
                  "rmse=0.187584, support=1080/1583, converged=false"),
              "observation model invalid");
    EXPECT_EQ(summarizeStateReason("correction conflict 2/3; rejected sibling evidence"),
              "correction conflict 2/3");
    EXPECT_EQ(summarizeStateReason("estimator failed\nlong exception"), "estimator failed");
    EXPECT_EQ(summarizeStateReason("estimator failed\r\nlong exception"), "estimator failed");
}

TEST(StateText, BoundsUnstructuredReasons) {
    EXPECT_EQ(summarizeStateReason(std::string(80, 'x')), std::string(80, 'x'));
    EXPECT_EQ(summarizeStateReason(std::string(81, 'x')), std::string(77, 'x') + "...");
}

TEST(StateText, DoesNotSplitUtf8CharactersAtTheLimit) {
    const std::string character = "\xE5\xAE\x9A";  // 定
    std::string reason;
    for (int i = 0; i < 30; ++i)
        reason += character;
    EXPECT_EQ(summarizeStateReason(reason), reason.substr(0, 75) + "...");
    EXPECT_EQ(summarizeStateReason(std::string(76, 'x') + reason), std::string(76, 'x') + "...");
    EXPECT_EQ(summarizeStateReason(std::string(77, 'x') + reason), std::string(77, 'x') + "...");
}

}  // namespace
}  // namespace backend::ros
