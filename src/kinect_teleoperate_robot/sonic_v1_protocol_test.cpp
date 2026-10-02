#include "include/SonicV1Publisher.hpp"

#include <input_interface/zmq_packed_message_subscriber.hpp>

#include <cassert>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    constexpr int port=5567;
    SonicV1Publisher publisher(port);
    std::mutex mutex; std::condition_variable ready; std::atomic_bool decoded=false, command_decoded=false;
    ZMQPackedMessageSubscriber subscriber("127.0.0.1",port,"pose",100,false,false,3);
    subscriber.SetOnDecodedMessage([&](const std::string& topic,
        const ZMQPackedMessageSubscriber::DecodedHeader& header,
        const std::vector<ZMQPackedMessageSubscriber::BufferView>& buffers) {
        assert(topic=="pose" && header.version==1 && header.endian=="le" && header.count==1);
        assert(header.fields.size()==5 && buffers.size()==5);
        assert(header.fields[0].name=="joint_pos" && header.fields[0].dtype=="f32" && header.fields[0].shape==std::vector<std::size_t>({1,29}));
        assert(header.fields[1].name=="joint_vel" && header.fields[1].shape==std::vector<std::size_t>({1,29}));
        const auto* pos=static_cast<const float*>(buffers[0].data);
        const auto* vel=static_cast<const float*>(buffers[1].data);
        assert(std::abs(pos[0]-0.125f)<1e-6f && std::abs(pos[28]+0.25f)<1e-6f);
        assert(std::abs(vel[3]-1.5f)<1e-6f);
        decoded=true; ready.notify_one();
    });
    ZMQPackedMessageSubscriber command_subscriber("127.0.0.1",port,"command",100,false,false,3);
    command_subscriber.SetOnDecodedMessage([&](const std::string& topic,
        const ZMQPackedMessageSubscriber::DecodedHeader& header,
        const std::vector<ZMQPackedMessageSubscriber::BufferView>& buffers) {
        assert(topic=="command" && header.version==1 && header.fields.size()==3 && buffers.size()==3);
        assert(header.fields[0].name=="start" && header.fields[1].name=="stop" && header.fields[2].name=="planner");
        const auto start=*static_cast<const std::uint8_t*>(buffers[0].data);
        const auto stop=*static_cast<const std::uint8_t*>(buffers[1].data);
        assert((start==1 || stop==1) && *static_cast<const std::uint8_t*>(buffers[2].data)==0);
        if (start) { command_decoded=true; ready.notify_one(); }
    });
    subscriber.Start();
    command_subscriber.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    G1Reference reference; reference.joint_pos=KinectToG1Retargeter::neutral;
    reference.joint_pos[0]=0.125; reference.joint_pos[28]=-0.25; reference.joint_vel[3]=1.5;
    const int messages=argc>1 ? std::max(20,std::stoi(argv[1])*30) : 20;
    for (int i=0;i<messages;++i) {
        if (i<15) assert(publisher.publish_command(true,false));
        assert(publisher.publish(reference,i));
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    assert(publisher.publish_command(false,true));
    { std::unique_lock<std::mutex> lock(mutex); ready.wait_for(lock,std::chrono::seconds(2),[&]{return decoded.load()&&command_decoded.load();}); }
    subscriber.Stop(); command_subscriber.Stop(); assert(decoded.load()&&command_decoded.load());
    std::cout << "SONIC Protocol v1 decoder integration passed\n";
}
