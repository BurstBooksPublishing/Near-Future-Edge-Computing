#include 
#include 
#include 
#include 
#include 
#include 
#include "MyTypePubSubTypes.h" // generated from IDL

using namespace eprosima::fastdds::dds;

int main()
{
    // Create participant
    DomainParticipantQos pqos;
    DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

    // Register type
    MyTypePubSubType mytype;
    participant->register_type(&mytype);

    // Create publisher
    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);

    // Topic
    Topic* topic = participant->create_topic("FusedSensor", mytype.get_type_name(), TOPIC_QOS_DEFAULT);

    // DataWriter QoS: reliable, keep last depth 3, deadline 50ms, latency budget 5ms
    DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
    wqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
    wqos.history().kind = KEEP_LAST_HISTORY_QOS;
    wqos.history().depth = 3;
    wqos.deadline().period = Duration_t(0, 50 * 1000000); // 50ms
    wqos.latency_budget().duration = Duration_t(0, 5 * 1000000); // 5ms

    DataWriter* writer = publisher->create_datawriter(topic, wqos);

    MyType sample;
    // populate sample fields...
    while (true) {
        // fill sensor-derived payload
        writer->write(&sample); // reliable write; blocks only per QoS and internal flow control
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz publish
    }

    // cleanup (omitted for brevity)
    return 0;
}