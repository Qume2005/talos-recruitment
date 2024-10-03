#ifndef TURTLE_STATE_HPP
#define TURTLE_STATE_HPP

#include <atomic>

enum class TurtleState {
    MOVING,
    WAITING
};

class TurtleStateManager {
public:
    TurtleStateManager() : state_(TurtleState::MOVING) {};
    void set_state(TurtleState state);
    TurtleState get_state() const;

private:
    std::atomic<TurtleState> state_;
};

void TurtleStateManager::set_state(TurtleState state) {
    state_ = state;
}

TurtleState TurtleStateManager::get_state() const {
    return state_;
}

#endif // TURTLE_STATE_HPP
