#include "sonic_zmq_publisher.hpp"

#include "input_interface/transport_freshness.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>

int main() {
    constexpr int port = 5568;
    SonicZmqPublisher publisher(port, "127.0.0.1");
    std::atomic<int> pose_count{0}, planner_count{0}, command_count{0};
    TransportFreshnessGuard pose_guard, planner_guard, command_guard;
    ZMQPackedMessageSubscriber pose("127.0.0.1", port, "pose", 50, false, false, 3);
    ZMQPackedMessageSubscriber planner("127.0.0.1", port, "planner", 50, false, false, 3);
    ZMQPackedMessageSubscriber command("127.0.0.1", port, "command", 50, false, false, 3);
    pose.SetOnDecodedMessage([&](const auto&, const auto& header, const auto& buffers) {
        if (pose_guard.Accept(header, buffers)) ++pose_count;
    });
    planner.SetOnDecodedMessage([&](const auto&, const auto& header, const auto& buffers) {
        if (planner_guard.Accept(header, buffers)) ++planner_count;
    });
    command.SetOnDecodedMessage([&](const auto&, const auto& header, const auto& buffers) {
        if (command_guard.Accept(header, buffers)) ++command_count;
    });
    pose.Start(); planner.Start(); command.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    SonicPoseFrame frame;
    frame.body_quat_w = {1, 0, 0, 0};
    frame.timestamp_monotonic_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    for (std::uint64_t sequence = 1; sequence <= 20; ++sequence) {
        publisher.publish_command(false, false, false, 1, sequence, sequence * 33333);
        publisher.publish_pose(frame, sequence, 1, sequence, sequence * 33333);
        publisher.publish_planner(frame, 1, sequence, sequence * 33333);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pose.Stop(); planner.Stop(); command.Stop();
    assert(pose_count >= 10 && planner_count >= 10 && command_count >= 10);
}
