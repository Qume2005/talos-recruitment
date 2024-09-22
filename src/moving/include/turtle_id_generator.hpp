#ifndef TURTLE_ID_GENERATOR_HPP
#define TURTLE_ID_GENERATOR_HPP

#include <atomic>
#include <string>

class TurtleIdGenerator {
public:
    TurtleIdGenerator() : current_id_(2) {}
    std::string get() { return "turtle" + std::to_string(current_id_++); }

private:
    std::atomic<int> current_id_;
};

#endif // TURTLE_ID_GENERATOR_HPP
