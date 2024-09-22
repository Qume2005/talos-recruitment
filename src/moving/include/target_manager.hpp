#ifndef TARGET_MANAGER_HPP
#define TARGET_MANAGER_HPP

#include <array>
#include <tuple>
#include <chrono>

using Target = std::tuple<double, double, double, std::chrono::duration<int64_t>>;

template<size_t N>
class TargetManager {
public:
    // 构造函数初始化列表
    TargetManager(const std::array<std::tuple<double, double, double, std::chrono::duration<int64_t>>, N>& targets)
        : targets_(targets) {}

    // 返回目标数组的起始和结束迭代器
    auto begin() const { return targets_.begin(); }
    auto end() const { return targets_.end(); }

    std::array<Target, N> targets_;
};

#endif // TARGET_MANAGER_HPP


