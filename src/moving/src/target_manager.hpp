#ifndef TARGET_MANAGER_HPP
#define TARGET_MANAGER_HPP

#include <array>
#include <tuple>

constexpr double MAX_BORDER = 17.0;
constexpr double MIN_BORDER = 0.5;

class TargetManager {
public:
    static constexpr std::array<std::tuple<double, double>, 4> TARGETS = {
        std::make_tuple(MAX_BORDER, MAX_BORDER),
        std::make_tuple(MIN_BORDER, MAX_BORDER),
        std::make_tuple(MIN_BORDER, MIN_BORDER),
        std::make_tuple(MAX_BORDER, MIN_BORDER),
    };
    
    auto begin() const { return TARGETS.begin(); }
    auto end() const { return TARGETS.end(); }
};

#endif // TARGET_MANAGER_HPP
