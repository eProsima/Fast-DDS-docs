// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TCPV4_TRANSPORT_H
#define TCPV4_TRANSPORT_H

#include <asio.hpp>
#include <thread>

#include <fastrtps/transport/tcp/RTCPMessageManager.h>
#include <fastrtps/utils/Semaphore.h>
#include "TransportInterface.h"
#include "TCPv4TransportDescriptor.h"
#include "SocketInfo.h"
#include "../utils/IPFinder.h"
#include "tcp/RTCPHeader.h"

#include <vector>
#include <memory>
#include <map>
#include <mutex>

namespace eprosima{
namespace fastrtps{
namespace rtps{
class TCPv4Transport;
class SenderResource;
class RTCPMessageManager;

class TCPAcceptor
{
public:
	asio::ip::tcp::acceptor mAcceptor;
    Locator_t mLocator;
	uint32_t mReceiveBufferSize;
    uint32_t mMaxMsgSize;
    std::function<std::shared_ptr<MessageReceiver>()> mAcceptCallback;

    TCPAcceptor(asio::io_service& io_service, const Locator_t& locator, ReceiverResource* receiverResource,
        uint32_t receiveBufferSize, uint32_t maxMsgSize);

    ~TCPAcceptor()
    {
        mAcceptCallback = nullptr;
    }
	void Accept(TCPv4Transport* parent);
};

class TCPConnector
{
public:
    Locator_t m_locator;
	eProsimaTCPSocket m_socket;

    TCPConnector(asio::io_service& io_service, Locator_t& locator);
    ~TCPConnector(){}

	void Connect(TCPv4Transport* parent);
	void RetryConnect(asio::io_service& io_service, TCPv4Transport* parent);
};


/**
 * This is a default TCPv4 implementation.
 *    - Opening an output channel by passing a locator will open a socket per interface on the given port.
 *       This collection of sockets constitute the "outbound channel". In other words, a channel corresponds
 *       to a port + a direction.
 *
 *    - It is possible to provide a white list at construction, which limits the interfaces the transport
 *       will ever be able to interact with. If left empty, all interfaces are allowed.
 *
 *    - Opening an input channel by passing a locator will open a socket listening on the given port on every
 *       whitelisted interface, and join the multicast channel specified by the locator address. Hence, any locator
 *       that does not correspond to the multicast range will simply open the port without a subsequent join. Joining
 *       multicast groups late is supported by attempting to open the channel again with the same port + a
 *       multicast address (the OpenInputChannel function will fail, however, because no new channel has been
 *       opened in a strict sense).
 * @ingroup TRANSPORT_MODULE
 */
class TCPv4Transport : public TransportInterface
{
public:
    friend class RTCPMessageManager;

    RTPS_DllAPI TCPv4Transport(const TCPv4TransportDescriptor&);

    virtual ~TCPv4Transport() override;

    bool init() override;

    //! Checks whether there are open and bound sockets for the given port.
    virtual bool IsInputChannelOpen(const Locator_t&) const override;

    /**
    * Checks whether there are open and bound sockets for the given port.
    */
    virtual bool IsOutputChannelOpen(const Locator_t&) const override;
    bool IsOutputChannelBound(const Locator_t&) const;
    bool IsOutputChannelConnected(const Locator_t&) const;
    void BindInputSocket(const Locator_t&, std::shared_ptr<TCPSocketInfo>);
    void BindOutputChannel(const Locator_t&);
    void UnbindInputSocket(std::shared_ptr<TCPSocketInfo>);

    //! Checks for TCPv4 kind.
    virtual bool IsLocatorSupported(const Locator_t&) const override;

    //! Reports whether Locators correspond to the same port.
    virtual bool DoLocatorsMatch(const Locator_t&, const Locator_t&) const override;

    /**
    * Converts a given remote locator (that is, a locator referring to a remote
    * destination) to the main local locator whose channel can write to that
    * destination. In this case it will return a 0.0.0.0 address on that port.
    */
    virtual Locator_t RemoteToMainLocal(const Locator_t&) const override;

    //! Sets the ID of the participant that has created the transport.
    virtual void SetParticipantGUIDPrefix(const GuidPrefix_t& prefix) override;

    /**
    * Starts listening on the specified port, and if the specified address is in the
    * multicast range, it joins the specified multicast group,
    */
    virtual bool OpenInputChannel(const Locator_t&, ReceiverResource*, uint32_t) override;

    /**
    * Opens a socket on the given address and port (as long as they are white listed).
    */
    virtual bool OpenOutputChannel(Locator_t&) override;

    //! Removes the listening socket for the specified port.
    virtual bool CloseInputChannel(const Locator_t&) override;

    //! Removes all outbound sockets on the given port.
    virtual bool CloseOutputChannel(const Locator_t&) override;

    /**
    * Blocking Send through the specified channel. In both modes, using a localLocator of 0.0.0.0 will
    * send through all whitelisted interfaces provided the channel is open.
    * @param sendBuffer Slice into the raw data to send.
    * @param sendBufferSize Size of the raw data. It will be used as a bounds check for the previous argument.
    * It must not exceed the sendBufferSize fed to this class during construction.
    * @param localLocator Locator mapping to the channel we're sending from.
    * @param remoteLocator Locator describing the remote destination we're sending to.
    */
    virtual bool Send(const octet* sendBuffer, uint32_t sendBufferSize, const Locator_t& localLocator,
                        const Locator_t& remoteLocator) override;
    /**
    * Blocking Receive from the specified channel.
    * @param receiveBuffer vector with enough capacity (not size) to accomodate a full receive buffer. That
    * capacity must not be less than the receiveBufferSize supplied to this class during construction.
    * @param localLocator Locator mapping to the local channel we're listening to.
    * @param[out] remoteLocator Locator describing the remote restination we received a packet from.
    */
   bool Receive(std::shared_ptr<TCPSocketInfo> socketInfo, octet* receiveBuffer, uint32_t receiveBufferCapacity,
       uint32_t& receiveBufferSize, uint16_t& logicalPort);

    virtual LocatorList_t NormalizeLocator(const Locator_t& locator) override;

    virtual LocatorList_t ShrinkLocatorLists(const std::vector<LocatorList_t>& locatorLists) override;

    virtual bool is_local_locator(const Locator_t& locator) const override;

    TransportDescriptorInterface* get_configuration() override { return &mConfiguration_; }

    virtual void AddDefaultLocator(LocatorList_t &defaultList) override;

    void SocketAccepted(TCPAcceptor* acceptor, const asio::error_code& error, asio::ip::tcp::socket s);
    void SocketConnected(Locator_t& locator, const asio::error_code& error);
protected:
    enum eSocketErrorCodes
    {
        eNoError,
        eBrokenPipe,
        eAsioError,
        eSystemError,
        eException
    };

    //! Constructor with no descriptor is necessary for implementations derived from this class.
    TCPv4Transport();
    TCPv4TransportDescriptor mConfiguration_;
    uint32_t mReceiveBufferSize;
    bool mActive;
    RTCPMessageManager* mRTCPMessageManager;
    std::vector<std::thread*> mThreadPool;
    asio::io_service mService;
    std::unique_ptr<std::thread> ioServiceThread;

    mutable std::recursive_mutex mSocketsMapMutex;
    std::recursive_mutex mThreadPoolMutex;

    std::map<Locator_t, TCPConnector*> mPendingOutputSockets;
    std::vector<std::shared_ptr<TCPSocketInfo>> mOutputSockets;
    std::map<Locator_t, std::shared_ptr<TCPSocketInfo>> mBoundOutputSockets;

    std::vector<IPFinder::info_IP> mCurrentInterfaces;

    struct LocatorCompare{ bool operator()(const Locator_t& lhs, const Locator_t& rhs) const
                        {return (memcmp(&lhs, &rhs, sizeof(Locator_t)) < 0); } };

    std::map<uint16_t, std::shared_ptr<TCPAcceptor>> mSocketsAcceptors; // The Key is the "Physical Port"
    std::map<uint16_t, std::vector<std::shared_ptr<TCPSocketInfo>>> mInputSockets;   // The Key is the "Physical Port"
    std::map<Locator_t, ReceiverResource*> mReceiverResources;

    bool IsTCPInputSocket(const Locator_t& locator) const;
    bool IsInterfaceAllowed(const asio::ip::address_v4& ip);
    std::vector<asio::ip::address_v4> mInterfaceWhiteList;

    bool OpenAndBindOutputSockets(Locator_t& locator);
    void OpenAndBindUnicastOutputSocket(Locator_t& locator);
    bool EnqueueLogicalOutputPort(Locator_t& locator);
    bool EnqueueLogicalInputPort(const Locator_t& locator);

    bool OpenAndBindInputSockets(const Locator_t& locator, ReceiverResource* receiverResource, uint32_t maxMsgSize);
    void CloseTCPSocket(std::shared_ptr<TCPSocketInfo> socketInfo);
    void ReleaseTCPSocket(std::shared_ptr<TCPSocketInfo> socketInfo);

    // Functions to be called from a new thread, which takes cares of performing a blocking receive
    void performListenOperation(std::shared_ptr<TCPSocketInfo> pSocketInfo);
    void performRTPCManagementThread(std::shared_ptr<TCPSocketInfo> pSocketInfo);

    bool ReadBody(octet* receiveBuffer, uint32_t receiveBufferCapacity, uint32_t* bytes_received,
        std::shared_ptr<TCPSocketInfo> pSocketInfo, std::size_t body_size);

    bool SendThroughSocket(const octet* sendBuffer, uint32_t sendBufferSize, const Locator_t& remoteLocator,
        std::shared_ptr<TCPSocketInfo> socket);

    size_t Send(std::shared_ptr<TCPSocketInfo> socketInfo, const octet* data,
        size_t size, eSocketErrorCodes &error) const;
    size_t Send(std::shared_ptr<TCPSocketInfo> socketInfo, const octet* data, size_t size) const;

};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif
