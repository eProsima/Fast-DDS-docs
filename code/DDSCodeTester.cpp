/*
#include <fastrtps/Domain.h>
#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>
#include <fastrtps/rtps/reader/RTPSReader.h>
#include <fastrtps/rtps/reader/ReaderListener.h>
#include <fastrtps/rtps/history/WriterHistory.h>
#include <fastrtps/rtps/history/ReaderHistory.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/xmlparser/XMLEndpointParser.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastrtps/log/Log.h>
#include <fastrtps/log/FileConsumer.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/TopicDataType.h>
#include <fastrtps_deprecated/security/accesscontrol/GovernanceParser.h>
#include <fastrtps_deprecated/security/accesscontrol/PermissionsParser.h>
*/

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

//using namespace eprosima::fastrtps;
using namespace eprosima::fastdds::dds;
//using namespace ::rtps;
//using namespace ::xmlparser;
//using namespace ::security;
//using namespace ::types;
//using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;


void dds_domain_examples()
{
    {
        //DDS_CREATE_DOMAINPARTICIPANT
        DomainParticipant* participant_with_default_attributes = DomainParticipantFactory::get_instance()->create_participant(0);
        if (!participant_with_default_attributes)
        {
            //error
            return;
        }

        DomainParticipantQos qos;
        qos.entity_factory().autoenable_created_entities = false;
        DomainParticipant* participant_with_non_default_qos = DomainParticipantFactory::get_instance()->create_participant(0, qos);
        if (!participant_with_non_default_qos)
        {
            //error
            return;
        }

        DomainParticipantListener listener;
        DomainParticipant* participant_with_default_qos_and_listener =
                DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT, &listener);
        if (!participant_with_default_qos_and_listener)
        {
            //error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DOMAINPARTICIPANTQOS
        DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0);
        if (!participant)
        {
            //error
            return;
        }

        //get the current QoS, and change desired values or create a completely one from scratch
        DomainParticipantQos qos = participant->get_qos();
        qos.entity_factory().autoenable_created_entities = false;

        //Assign the new Qos to the object
        participant->set_qos(qos);
        //!--
    }

    {
        //DDS_CHANGE_DOMAINPARTICIPANTQOS_TO_DEFAULT
        //Create a participant with non-default QoS
        DomainParticipantQos qos;
        qos.entity_factory().autoenable_created_entities = false;
        DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0, qos);
        if (!participant)
        {
            //error
            return;
        }

        //Set the QoS on the participant to the default
        if (participant->set_qos(PARTICIPANT_QOS_DEFAULT) != ReturnCode_t::RETCODE_OK)
        {
            //error
            return;
        }

        //The previous instruction is equivalent to the following:
        if(participant->set_qos(DomainParticipantFactory::get_instance()->get_default_participant_qos()) != ReturnCode_t::RETCODE_OK)
        {
            //error
            return;
        }
        //!--
    }

    {
        //DDS_DELETE_DOMAINPARTICIPANT
        DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0);
        if (!participant)
        {
            //error
            return;
        }

        // (...)

        if (DomainParticipantFactory::get_instance()->delete_participant(participant) != ReturnCode_t::RETCODE_OK)
        {
            //error
            return;
        }
        //!--
    }

    {
        //DDS_CHANGE_DEFAULT_DOMAINPARTICIPANTQOS
        DomainParticipantQos qos = DomainParticipantFactory::get_instance()->get_default_participant_qos();

        qos.entity_factory().autoenable_created_entities = true;
        if(DomainParticipantFactory::get_instance()->set_default_participant_qos(qos) != ReturnCode_t::RETCODE_OK)
        {
            //error
            return;
        }

        DomainParticipant* enabled_participant = DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (!enabled_participant)
        {
            //error
            return;
        }

        qos.entity_factory().autoenable_created_entities = false;
        if(DomainParticipantFactory::get_instance()->set_default_participant_qos(qos) != ReturnCode_t::RETCODE_OK)
        {
            //error
            return;
        }

        DomainParticipant* disabled_participant = DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
        if (!disabled_participant)
        {
            //error
            return;
        }
        //!--
    }

}


