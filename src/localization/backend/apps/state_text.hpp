/**
 * @file state_text.hpp
 * @brief 为常用定位状态保留有界原因摘要；完整原因由诊断话题提供。
 */
#pragma once

#include <string>
#include <string_view>

namespace backend::ros {

/// 保留首个原因类别，省略冒号后的数值明细，最多占 80 个 UTF-8 字节。
inline std::string summarizeStateReason(const std::string_view reason) {
    const auto summary = reason.substr(0, reason.find_first_of(":;\r\n"));
    constexpr std::size_t limit = 80;
    if (summary.size() <= limit)
        return std::string(summary);
    std::size_t end = limit - 3;
    // 截断边界回退到 UTF-8 字符开头，避免把多字节文字切坏。
    while (end > 0 && (static_cast<unsigned char>(summary[end]) & 0xC0) == 0x80)
        --end;
    return std::string(summary.substr(0, end)) + "...";
}

}  // namespace backend::ros
