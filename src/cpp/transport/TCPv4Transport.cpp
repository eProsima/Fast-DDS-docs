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

#include <fastrtps/transport/TCPv4Transport.h>
#include <utility>
#include <cstring>
#include <algorithm>
#include <fastrtps/log/Log.h>
#include <fastrtps/rtps/messages/RTPSMessageCreator.h>
#include "asio.hpp"
#include <fastrtps/rtps/network/ReceiverResource.h>
#include <fastrtps/utils/eClock.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>

using namespace std;
using namespace asio;

namespace eprosima{
namespace fastrtps{
namespace rtps {

static const int s_default_keep_alive_frequency = 50000; // 50 SECONDS
static const int s_default_keep_alive_timeout = 10000; // 10 SECONDS

static void GetIP4s(std::vector<IPFinder::info_IP>& locNames, bool return_loopback = false)
{
    IPFinder::getIPs(&locNames, return_loopback);
    auto new_end = remove_if(locNames.begin(),
        locNames.end(),
        [](IPFinder::info_IP ip) {return ip.type != IPFinder::IP4 && ip.type != IPFinder::IP4_LOCAL; });
    locNames.erase(new_end, locNames.end());
}

static asio::ip::address_v4::bytes_type locatorToNative(Locator_t& locator)
{
    if (locator.has_IP4_WAN_address())
    {
        return{ {locator.get_IP4_WAN_address()[0],
            locator.get_IP4_WAN_address()[1], locator.get_IP4_WAN_address()[2], locator.get_IP4_WAN_address()[3]} };
    }
    else
    {
        return{ {locator.get_IP4_address()[0],
            locator.get_IP4_address()[1], locator.get_IP4_address()[2], locator.get_IP4_address()[3]} };
    }
}

TCPAcceptor::TCPAcceptor(asio::io_service& io_service, const Locator_t& locator, uint32_t maxMsgSize)
    : mAcceptor(io_service, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), locator.get_physical_port()))
    , mLocator(locator)
    , mMaxMsgSize(maxMsgSize)
{
#ifndef ASIO_HAS_MOVE
    mSocket = std::make_shared<asio::ip::tcp::socket>(io_service);
    mEndPoint = asio::ip::tcp::endpoint(asio::ip::tcp::v4(), locator.get_physical_port());
#endif
}


#ifdef ASIO_HAS_MOVE
void TCPAcceptor::Accept(TCPv4Transport* parent, asio::io_service&)
{
    mAcceptor.async_accept(std::bind(&TCPv4Transport::SocketAccepted, parent, this, std::placeholders::_1,
        std::placeholders::_2));
}
#else
void TCPAcceptor::Accept(TCPv4Transport* parent, asio::io_service& io_service)
{
    if (mSocket == nullptr)
    {
        mSocket = std::make_shared<asio::ip::tcp::socket>(io_service);
    }
    mAcceptor.async_accept(*mSocket.get(), mEndPoint, std::bind(&TCPv4Transport::SocketAccepted, parent, this, std::placeholders::_1));
}
#endif

TCPConnector::TCPConnector(asio::io_service& io_service, Locator_t& locator)
    : m_locator(locator)
    , m_socket(createTCPSocket(io_service))
{
}

TCPConnector::~TCPConnector()
{
    getSocketPtr(m_socket)->close();
}

void TCPConnector::Connect(TCPv4Transport* parent, SenderResource *senderResource)
{
    getSocketPtr(m_socket)->open(ip::tcp::v4());
    auto ipAddress = asio::ip::address_v4(locatorToNative(m_locator));
    ip::tcp::endpoint endpoint(ipAddress, static_cast<uint16_t>(m_locator.get_physical_port()));
    getSocketPtr(m_socket)->async_connect(endpoint, std::bind(&TCPv4Transport::SocketConnected, parent,
        m_locator, senderResource, std::placeholders::_1));
}

void TCPConnector::RetryConnect(asio::io_service& io_service, TCPv4Transport* parent, SenderResource *senderResource)
{
    getSocketPtr(m_socket)->close();
    m_socket = createTCPSocket(io_service);
    Connect(parent, senderResource);
}

TCPv4Transport::TCPv4Transport(const TCPv4TransportDescriptor& descriptor)
    : mConfiguration_(descriptor)
    , mActive(true)
    , mRTCPMessageManager(nullptr)
{
    for (const auto& interface : descriptor.interfaceWhiteList)
    {
        mInterfaceWhiteList.emplace_back(ip::address_v4::from_string(interface));
    }
}

TCPv4TransportDescriptor::TCPv4TransportDescriptor() :
    TransportDescriptorInterface(s_maximumMessageSize)
    , keep_alive_frequency_ms(s_default_keep_alive_frequency)
    , keep_alive_timeout_ms(s_default_keep_alive_timeout)
    , max_logical_port(100)
    , logical_port_range(20)
    , logical_port_increment(2)
    , metadata_logical_port(0)
{
    memset(wan_addr, 0, 4);
}

TCPv4TransportDescriptor::TCPv4TransportDescriptor(const TCPv4TransportDescriptor& t) :
    TransportDescriptorInterface(t)
    , listening_ports(t.listening_ports)
    , keep_alive_frequency_ms(t.keep_alive_frequency_ms)
    , keep_alive_timeout_ms(t.keep_alive_timeout_ms)
    , max_logical_port(t.max_logical_port)
    , logical_port_range(t.logical_port_range)
    , logical_port_increment(t.logical_port_increment)
    , metadata_logical_port(t.metadata_logical_port)
{
    memcpy(wan_addr, t.wan_addr, 4);
}

TransportInterface* TCPv4TransportDescriptor::create_transport() const
{
    return new TCPv4Transport(*this);
}

TCPv4Transport::TCPv4Transport()
    : mActive(true)
    , mRTCPMessageManager(nullptr)
{
}

TCPv4Transport::~TCPv4Transport()
{
    std::vector<TCPSocketInfo*> vDeletedSockets;

    // Collect all the existing sockets to delete them outside of the mutex.
    {
        std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
        mActive = false;

        for (auto it = mSocketAcceptors.begin(); it != mSocketAcceptors.end(); ++it)
        {
            delete it->second;
        }
        mSocketAcceptors.clear();

        for (auto it = mSocketConnectors.begin(); it != mSocketConnectors.end(); ++it)
        {
            delete(it->second);
        }
        mSocketConnectors.clear();

        for (auto it = mOutputSockets.begin(); it != mOutputSockets.end(); ++it)
        {
            vDeletedSockets.push_back(*it);
        }
        mOutputSockets.clear();

        for (auto it = mInputSockets.begin(); it != mInputSockets.end(); ++it)
        {
            for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
            {
                vDeletedSockets.push_back(*it2);
            }
        }
        mInputSockets.clear();
    }

    for (auto it = vDeletedSockets.begin(); it != vDeletedSockets.end(); ++it)
    {
        ReleaseTCPSocket(*it);
        (*it) = nullptr;
    }

    CleanDeletedSockets();

    if (ioServiceThread)
    {
        mService.stop();
        ioServiceThread->join();
    }

    delete mRTCPMessageManager;
}

bool TCPv4Transport::init()
{
    if (mConfiguration_.sendBufferSize == 0 || mConfiguration_.receiveBufferSize == 0)
    {
        // Check system buffer sizes.
        ip::tcp::socket socket(mService);
        socket.open(ip::tcp::v4());

        if (mConfiguration_.sendBufferSize == 0)
        {
            socket_base::send_buffer_size option;
            socket.get_option(option);
            mConfiguration_.sendBufferSize = option.value();

            if (mConfiguration_.sendBufferSize < s_minimumSocketBuffer)
            {
                mConfiguration_.sendBufferSize = s_minimumSocketBuffer;
                //mSendBufferSize = s_minimumSocketBuffer;
            }
        }

        if (mConfiguration_.receiveBufferSize == 0)
        {
            socket_base::receive_buffer_size option;
            socket.get_option(option);
            mConfiguration_.receiveBufferSize = option.value();

            if (mConfiguration_.receiveBufferSize < s_minimumSocketBuffer)
            {
                mConfiguration_.receiveBufferSize = s_minimumSocketBuffer;
                //mReceiveBufferSize = s_minimumSocketBuffer;
            }
        }

        socket.close();
    }

    if (mConfiguration_.maxMessageSize > s_maximumMessageSize)
    {
        logError(RTPS_MSG_OUT, "maxMessageSize cannot be greater than 65000");
        return false;
    }

    if (mConfiguration_.maxMessageSize > mConfiguration_.sendBufferSize)
    {
        logError(RTPS_MSG_OUT, "maxMessageSize cannot be greater than sendBufferSize");
        return false;
    }

    if (mConfiguration_.maxMessageSize > mConfiguration_.receiveBufferSize)
    {
        logError(RTPS_MSG_OUT, "maxMessageSize cannot be greater than receiveBufferSize");
        return false;
    }

    if (mRTCPMessageManager == nullptr)
    {
        mRTCPMessageManager = new RTCPMessageManager(this);
    }

    // TODO(Ricardo) Create an event that update this list.
    GetIP4s(mCurrentInterfaces);

    auto ioServiceFunction = [&]()
    {
        io_service::work work(mService);
        mService.run();
    };
    ioServiceThread.reset(new std::thread(ioServiceFunction));

    return true;
}

bool TCPv4Transport::IsInputChannelOpen(const Locator_t& locator) const
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    return IsLocatorSupported(locator) && mReceiverResources.find(locator) != mReceiverResources.end();
}

bool TCPv4Transport::IsInputSocketOpen(const Locator_t& locator) const
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    return IsLocatorSupported(locator) && (mInputSockets.find(locator.get_physical_port()) != mInputSockets.end() ||
        (mSocketAcceptors.find(locator.get_physical_port()) != mSocketAcceptors.end()));
}

bool TCPv4Transport::IsOutputChannelOpen(const Locator_t& locator, SenderResource *senderResource) const
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    if (!IsLocatorSupported(locator))
        return false;

    if (mSocketConnectors.find(locator) != mSocketConnectors.end())
        return true;
    else
    {
        for (auto it = mOutputSockets.begin(); it != mOutputSockets.end(); ++it)
        {
            if ((*it)->GetLocator().compare_IP4_address_and_port(locator))
            {
                if (senderResource != nullptr)
                {
                    senderResource->SetSocketInfo((*it));
                    AssociateSenderToSocket(*it, senderResource);
                }
                return true;
            }
        }
    }

    return false;
}

void TCPv4Transport::BindInputSocket(const Locator_t& locator, TCPSocketInfo *socketInfo)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    if (!IsLocatorSupported(locator))
    {
        return;
    }

    auto it = mBoundOutputSockets.find(locator);
    if (it == mBoundOutputSockets.end())
    {
        mBoundOutputSockets[locator].push_back(socketInfo); // First element, just add it
    }
    else
    {
        // There are more, check isn't already added
        auto found = std::find(mBoundOutputSockets[locator].begin(), mBoundOutputSockets[locator].end(), socketInfo);
        if (found == mBoundOutputSockets[locator].end())
        {
            mBoundOutputSockets[locator].push_back(socketInfo);
        }
    }


    //assert(it == mBoundOutputSockets.end() || (it != mBoundOutputSockets.end() && it->second != socketInfo));
    //if (it == mBoundOutputSockets.end())
    //{
        // std::cout << "################## A" << std::endl;
        // std::cout << "LOCATOR KIND: " << locator.kind << std::endl;
        // std::cout << "LOCATOR Address: " << locator.to_IP4_string() << std::endl;
        // std::cout << "LOCATOR Physical: " << locator.get_physical_port() << std::endl;
        // std::cout << "LOCATOR Logical: " << locator.get_logical_port() << std::endl;
        // std::cout << "ENDPOINT Local: " << socketInfo->getSocket()->local_endpoint().port() << std::endl;
        // std::cout << "ENDPOINT Remote: " << socketInfo->getSocket()->remote_endpoint().port() << std::endl;
        // mBoundOutputSockets[locator].push_back(socketInfo);
        // std::cout << mBoundOutputSockets.size() << std::endl;
        // std::cout << "A ##################" << std::endl;
    //}
}

void TCPv4Transport::UnbindInputSocket(TCPSocketInfo *pSocket)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    for (auto it = mBoundOutputSockets.begin(); it != mBoundOutputSockets.end();)
    {
        auto& vector = it->second;
        auto toRemove = std::find(vector.begin(), vector.end(), pSocket);
        if (toRemove != vector.end())
        {
            vector.erase(toRemove);
        }

        if (vector.empty())
        {
            it = mBoundOutputSockets.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void TCPv4Transport::BindOutputChannel(const Locator_t& locator, SenderResource *senderResource)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    if (!IsLocatorSupported(locator))
    {
        return;
    }

    assert(mBoundOutputSockets.find(locator) == mBoundOutputSockets.end());
    for (auto it = mOutputSockets.begin(); it != mOutputSockets.end(); ++it)
    {
        if ((*it)->GetLocator().compare_IP4_address_and_port(locator))
        {
            // std::cout << "################## B" << std::endl;
            // std::cout << "LOCATOR KIND: " << locator.kind << std::endl;
            // std::cout << "LOCATOR Address: " << locator.to_IP4_string() << std::endl;
            // std::cout << "LOCATOR Physical: " << locator.get_physical_port() << std::endl;
            // std::cout << "LOCATOR Logical: " << locator.get_logical_port() << std::endl;
            // std::cout << "ENDPOINT Local: " << (*it)->getSocket()->local_endpoint().port() << std::endl;
            // std::cout << "ENDPOINT Remote: " << (*it)->getSocket()->remote_endpoint().port() << std::endl;
            // mBoundOutputSockets[locator] = (*it);
            // std::cout << mBoundOutputSockets.size() << std::endl;
            // std::cout << "B ##################" << std::endl;
            BindInputSocket(locator, *it);
            if (senderResource != nullptr)
            {
                senderResource->SetSocketInfo(*it);
                AssociateSenderToSocket(*it, senderResource);
            }
        }
    }
}

bool TCPv4Transport::IsOutputChannelBound(const Locator_t& locator, SenderResource *senderResource) const
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    if (!IsLocatorSupported(locator))
        return false;

    auto socket = mBoundOutputSockets.find(locator);
    if (socket != mBoundOutputSockets.end())
    {
        if (senderResource != nullptr)
        {
            auto& vector = (*socket).second;
            if (vector.size() == 1)
            {
                senderResource->SetSocketInfo(vector.front());
                AssociateSenderToSocket(vector.front(), senderResource);
            }
        }
        return true;
    }

    return false;
}

bool TCPv4Transport::IsOutputChannelConnected(const Locator_t& locator) const
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    if (!IsLocatorSupported(locator))
        return false;

    for (auto it = mOutputSockets.begin(); it != mOutputSockets.end(); ++it)
    {
        if ((*it)->GetLocator().compare_IP4_address_and_port(locator))
        {
            return true;
        }
    }

    return IsOutputChannelBound(locator);
}

bool TCPv4Transport::OpenOutputChannel(Locator_t& locator, SenderResource* senderResource)
{
    bool success = false;
    if (IsLocatorSupported(locator))
    {
        if (!IsTCPInputSocket(locator, senderResource))
        {
            std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
            if (!IsOutputChannelConnected(locator))
            {
                if (!IsOutputChannelOpen(locator))
                {
                    success = OpenAndBindOutputSockets(locator, senderResource);
                }
                else
                {
                    BindOutputChannel(locator, senderResource);
                    success = EnqueueLogicalOutputPort(locator);
                }
            }
            else
            {
                if (!IsOutputChannelBound(locator, senderResource))
                {
                    BindOutputChannel(locator, senderResource);
                    success = EnqueueLogicalOutputPort(locator);
                }
                else
                    success = true;
            }
        }
        else
        {
            if (mBoundOutputSockets.find(locator) == mBoundOutputSockets.end())
            {
                mPendingOutputPorts.push_back(locator);
                auto socketIt = mInputSockets.find(locator.get_physical_port());
                if (socketIt != mInputSockets.end())
                {
                    for (auto it = socketIt->second.begin(); it != socketIt->second.end(); ++it)
                    {
                        BindInputSocket(locator, *it);
                    }
                }
            }
            success = true;
        }
    }
    else
    {
        success = true;
    }
    return success;
}

bool TCPv4Transport::OpenExtraOutputChannel(Locator_t& locator, SenderResource* senderResource)
{
    return OpenOutputChannel(locator, senderResource);
}

bool TCPv4Transport::OpenInputChannel(const Locator_t& locator, ReceiverResource* receiverResource,
    uint32_t maxMsgSize)
{
    bool success = false;
    if (IsLocatorSupported(locator))
    {
        std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
        if (mReceiverResources.find(locator) == mReceiverResources.end())
        {
            mReceiverResources[locator] = receiverResource;

            logInfo(RTCP, " OpenInputChannel (physical: " << locator.get_physical_port() << "; logical: " << locator.get_logical_port() << ")");
            if (IsTCPInputSocket(locator, nullptr))
            {
                success = true;
                if (!IsInputSocketOpen(locator))
                {
                    success = OpenAndBindInputSockets(locator, maxMsgSize);
                }
            }
            else
            {
                success = true;
                if (IsOutputChannelConnected(locator))
                {
                    TCPSocketInfo* pSocket = nullptr;
                    for (auto it = mOutputSockets.begin(); it != mOutputSockets.end(); ++it)
                    {
                        if ((*it)->GetLocator().compare_IP4_address_and_port(locator))
                        {
                            pSocket = *it;
                            break;
                        }
                    }

                    if (pSocket != nullptr)
                    {
                        RegisterReceiverResources(pSocket, locator);
                    }
                }
            }
        }
        else
        {
            success = false;
        }

    }
    return success;
}

bool TCPv4Transport::CloseOutputChannel(const Locator_t& locator)
{
    std::vector<TCPSocketInfo*> vDeletedSockets;
    {
        std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
        if (!IsOutputChannelOpen(locator))
            return false;

        if (IsTCPInputSocket(locator, nullptr))
        {
            for (auto it = mPendingOutputPorts.begin(); it != mPendingOutputPorts.end();)
            {
                if (*it == locator)
                {
                    it = mPendingOutputPorts.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
        mBoundOutputSockets.erase(locator);

        if (mSocketConnectors.find(locator) != mSocketConnectors.end())
        {
            auto& pendingSocket = mSocketConnectors.at(locator);
            getSocketPtr(pendingSocket->m_socket)->close();
            delete(mSocketConnectors[locator]);
            mSocketConnectors.erase(locator);
        }

        for (auto it = mOutputSockets.begin(); it != mOutputSockets.end();)
        {
            if ((*it)->GetLocator() == locator)
            {
                vDeletedSockets.emplace_back(*it);
                it = mOutputSockets.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    for (auto it = vDeletedSockets.begin(); it != vDeletedSockets.end(); ++it)
    {
        ReleaseTCPSocket(*it);
        (*it) = nullptr;
    }
    vDeletedSockets.clear();

    return true;
}

void TCPv4Transport::CloseInputSocket(TCPSocketInfo *socketInfo)
{
    if (socketInfo->IsAlive())
    {
        std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
        socketInfo->ChangeStatus(TCPSocketInfo::eConnectionStatus::eDisconnected);
        auto inputSocketIt = mInputSockets.find(socketInfo->GetLocator().get_physical_port());
        if (inputSocketIt != mInputSockets.end())
        {
            auto& vector = mInputSockets[socketInfo->GetLocator().get_physical_port()];
            auto it = std::find(vector.begin(), vector.end(), socketInfo);
            vector.erase(it);

            if (vector.size() == 0)
            {
                mInputSockets.erase(inputSocketIt);
            }
        }

        UnbindInputSocket(socketInfo);
        // UnregisterReceiverResource(socketInfo); // Unnecessary
        ReleaseTCPSocket(socketInfo);
    }
}

bool TCPv4Transport::CloseInputChannel(const Locator_t& locator)
{
    bool bClosed = false;
    std::vector<TCPSocketInfo*> vDeletedSockets;
    {
        std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);

        if (mReceiverResources.find(locator) != mReceiverResources.end())
        {
            mReceiverResources.erase(locator);
            bClosed = true;
        }

        auto acceptorIt = mSocketAcceptors.find(locator.get_physical_port());
        if (acceptorIt != mSocketAcceptors.end())
        {
            delete acceptorIt->second;
            mSocketAcceptors.erase(acceptorIt);
            bClosed = true;
        }

        auto InputSocketIt = mInputSockets.find(locator.get_physical_port());
        if (InputSocketIt != mInputSockets.end())
        {
            std::vector<TCPSocketInfo*> sockets = InputSocketIt->second;
            for (auto it = sockets.begin(); it != sockets.end(); ++it)
            {
                vDeletedSockets.emplace_back(*it);
            }

            mInputSockets.erase(InputSocketIt);
            bClosed = true;
        }
    }

    for (auto it = vDeletedSockets.begin(); it != vDeletedSockets.end(); ++it)
    {
        UnbindInputSocket(*it);
        ReleaseTCPSocket(*it);
        *it = nullptr;
    }

    return bClosed;
}

bool TCPv4Transport::IsTCPInputSocket(const Locator_t& locator, SenderResource *senderResource) const
{
    if (is_local_locator(locator))
    {
        for (auto it = mConfiguration_.listening_ports.begin(); it != mConfiguration_.listening_ports.end(); ++it)
        {
            if (locator.get_physical_port() == *it)
            {
                if (senderResource != nullptr)
                {
                    auto sit = mInputSockets.find(locator.get_physical_port());
                    if (sit != mInputSockets.end())
                    {
                        auto& inputs = mInputSockets.at(locator.get_physical_port());
                        if (inputs.size() > 1)
                        {
                            //logWarning(RTCP, "More than one inputSocket for physical port " << locator.get_physical_port());
                            senderResource->SetSocketInfo(nullptr);
                        }
                        else if (inputs.size() == 1)
                        {
                            senderResource->SetSocketInfo(*inputs.begin());
                        }
                    }
                }
                return true;
            }
        }
    }
    return false;
}

bool TCPv4Transport::IsInterfaceAllowed(const ip::address_v4& ip)
{
    if (mInterfaceWhiteList.empty())
        return true;

    if (ip == ip::address_v4::any())
        return true;

    return find(mInterfaceWhiteList.begin(), mInterfaceWhiteList.end(), ip) != mInterfaceWhiteList.end();
}

bool TCPv4Transport::EnqueueLogicalOutputPort(Locator_t& locator)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    for (auto it = mOutputSockets.begin(); it != mOutputSockets.end(); ++it)
    {
        if ((*it)->GetLocator().compare_IP4_address_and_port(locator)
                && std::find((*it)->mLogicalOutputPorts.begin(),
            (*it)->mLogicalOutputPorts.end(), locator.get_logical_port())
                == (*it)->mLogicalOutputPorts.end()
            && std::find((*it)->mPendingLogicalOutputPorts.begin(),
            (*it)->mPendingLogicalOutputPorts.end(), locator.get_logical_port())
                == (*it)->mPendingLogicalOutputPorts.end())
        {
            (*it)->mPendingLogicalOutputPorts.push_back(locator.get_logical_port());
            return true;
        }
    }
    return false;
}

bool TCPv4Transport::OpenAndBindOutputSockets(Locator_t& locator, SenderResource *senderResource)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    try
    {
        locator.set_IP4_WAN_address(mConfiguration_.wan_addr[0],
                                    mConfiguration_.wan_addr[1],
                                    mConfiguration_.wan_addr[2],
                                    mConfiguration_.wan_addr[3]);
        OpenAndBindUnicastOutputSocket(locator, senderResource);
    }
    catch (asio::system_error const& e)
    {
        (void)e;
        logInfo(RTPS_MSG_OUT,"TCPv4 Error binding at port:(" << locator.get_port() << ")" << " with msg:" << e.what());
        CloseOutputChannel(locator);
        return false;
    }

    return true;
}

bool TCPv4Transport::OpenAndBindInputSockets(const Locator_t& locator, uint32_t maxMsgSize)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    try
    {
        TCPAcceptor* newAcceptor = new TCPAcceptor(mService, locator, maxMsgSize);
        mSocketAcceptors.insert(std::make_pair(locator.get_physical_port(), newAcceptor));
        logInfo(RTCP, " OpenAndBindInput (physical: " << locator.get_physical_port() << "; logical: " << locator.get_logical_port() << ")");
        newAcceptor->Accept(this, mService);
    }
    catch (asio::system_error const& e)
    {
        (void)e;
        logInfo(RTPS_MSG_OUT, "TCPv4 Error binding at port: (" << locator.get_physical_port() << ")" << " with msg: " << e.what());
        return false;
    }

    return true;
}

void TCPv4Transport::performRTPCManagementThread(TCPSocketInfo *pSocketInfo)
{
    std::chrono::time_point<std::chrono::system_clock> time_now = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> next_time = time_now +
        std::chrono::milliseconds(mConfiguration_.keep_alive_frequency_ms);
    std::chrono::time_point<std::chrono::system_clock> timeout_time =
        time_now + std::chrono::milliseconds(mConfiguration_.keep_alive_timeout_ms);

    bool bSendOpenLogicalPort = false;
    logInfo(RTCP, "START performRTPCManagementThread " << pSocketInfo->GetLocator() << " (" << pSocketInfo->getSocket()->local_endpoint().address() << "->" << pSocketInfo->getSocket()->remote_endpoint().address() << ")");
    while (pSocketInfo->IsAlive())
    {
        if (pSocketInfo->IsConnectionEstablished())
        {
            { // Logical Ports
                std::unique_lock<std::recursive_mutex> scopedLock(pSocketInfo->mPendingLogicalMutex);
                if (pSocketInfo->mPendingLogicalPort == 0 && !pSocketInfo->mPendingLogicalOutputPorts.empty())
                {
                    pSocketInfo->mPendingLogicalPort = *pSocketInfo->mPendingLogicalOutputPorts.begin();
                    bSendOpenLogicalPort = true;
                }
            }

            if (bSendOpenLogicalPort)
            {
                mRTCPMessageManager->sendOpenLogicalPortRequest(pSocketInfo, pSocketInfo->mPendingLogicalPort);
                bSendOpenLogicalPort = false;
            }

            // KeepAlive
            if (mConfiguration_.keep_alive_frequency_ms > 0 && mConfiguration_.keep_alive_timeout_ms > 0)
            {
                time_now = std::chrono::system_clock::now();

                // Keep Alive Management
                if (!pSocketInfo->mWaitingForKeepAlive && time_now > next_time)
                {
                    mRTCPMessageManager->sendKeepAliveRequest(pSocketInfo);
                    pSocketInfo->mWaitingForKeepAlive = true;
                    next_time = time_now + std::chrono::milliseconds(mConfiguration_.keep_alive_frequency_ms);
                    timeout_time = time_now + std::chrono::milliseconds(mConfiguration_.keep_alive_timeout_ms);
                }
                else if (pSocketInfo->mWaitingForKeepAlive && time_now >= timeout_time)
                {
                    // Disable the socket to erase it after the reception.
                    mRTCPMessageManager->sendUnbindConnectionRequest(pSocketInfo);
                    ReleaseTCPSocket(pSocketInfo);
                    continue;
                }
            }
        }
        eClock::my_sleep(100);
    }
    logInfo(RTCP, "End performRTPCManagementThread " << pSocketInfo->GetLocator());
}

void TCPv4Transport::performListenOperation(TCPSocketInfo *pSocketInfo)
{
    Locator_t remoteLocator;
    uint16_t logicalPort(0);
    logInfo(RTCP, "START PerformListenOperation " << pSocketInfo->GetLocator() << " (" << pSocketInfo->getSocket()->local_endpoint().address() << "->" << pSocketInfo->getSocket()->remote_endpoint().address() << ")");
    while (pSocketInfo->IsAlive())
    {
        // Blocking receive.
        auto msg = pSocketInfo->GetMessageBuffer();
        CDRMessage::initCDRMsg(&msg);
        if (!Receive(pSocketInfo, msg.buffer, msg.max_size, msg.length, logicalPort))
        {
            continue;
        }

        // Processes the data through the CDR Message interface.
        MessageReceiver* receiver = pSocketInfo->GetMessageReceiver(logicalPort);
        if (receiver != nullptr)
        {
            receiver->processCDRMsg(mConfiguration_.rtpsParticipantGuidPrefix,
                &pSocketInfo->mLocator, &msg);
        }
        else
        {
            logWarning(RTCP,"Received Message, but no MessageReceiver attached: " << logicalPort);
        }
    }

    logInfo(RTCP, "End PerformListenOperation " << pSocketInfo->GetLocator());
}

void TCPv4Transport::OpenAndBindUnicastOutputSocket(Locator_t& locator, SenderResource *senderResource)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    if (mActive)
    {
        asio::ip::address_v4 ip = asio::ip::make_address_v4(locator.to_IP4_string());
        if (IsInterfaceAllowed(ip))
        {
            TCPConnector* newConnector = new TCPConnector(mService, locator);
            if (mSocketConnectors.find(locator) != mSocketConnectors.end())
            {
                TCPConnector* oldConnector = mSocketConnectors.at(locator);
                delete oldConnector;
            }
            mSocketConnectors[locator] = newConnector;
            logInfo(RTCP, " OpenAndBindOutput (physical: " << locator.get_physical_port() << "; logical: " << locator.get_logical_port() << ")");
            newConnector->Connect(this, senderResource);
        }
    }
}

bool TCPv4Transport::DoLocatorsMatch(const Locator_t& left, const Locator_t& right) const
{
    return left.get_port() == right.get_port();
}

bool TCPv4Transport::IsLocatorSupported(const Locator_t& locator) const
{
    return locator.kind == LOCATOR_KIND_TCPv4;
}

void TCPv4Transport::AddDefaultLocator(LocatorList_t &/*defaultList*/)
{
    // On TCP, there aren't default send locators.
}

Locator_t TCPv4Transport::RemoteToMainLocal(const Locator_t& remote) const
{
    if (!IsLocatorSupported(remote))
        return false;

    Locator_t mainLocal(remote);
    mainLocal.set_Invalid_Address();
    return mainLocal;
}

void TCPv4Transport::SetParticipantGUIDPrefix(const GuidPrefix_t& prefix)
{
    mConfiguration_.rtpsParticipantGuidPrefix = prefix;
}

bool TCPv4Transport::Send(const octet* sendBuffer, uint32_t sendBufferSize, const Locator_t& localLocator,
    const Locator_t& remoteLocator)
{
//    logInfo(RTCP, " SEND [RTPS Data] to locator " << remoteLocator.get_physical_port() << ":" << remoteLocator.get_logical_port());
    std::vector<TCPSocketInfo*> sockets;
    {
        std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
        if (!IsOutputChannelConnected(remoteLocator) || sendBufferSize > mConfiguration_.sendBufferSize)
        {
            logWarning(RTCP, "SEND [RTPS] Failed: Not connect: " << remoteLocator.get_logical_port());
            return false;
        }

        std::map<Locator_t, std::vector<TCPSocketInfo*>>::iterator it = mBoundOutputSockets.find(remoteLocator);
        if (it != mBoundOutputSockets.end())
        {
            sockets = it->second;
        }
        else
        {
            logWarning(RTCP, "SEND [RTPS] Failed: Not bound: " << remoteLocator.get_logical_port());
            return false;
        }
    }

    bool result = true;
    for (TCPSocketInfo* socket : sockets)
    {
        result = result && Send(sendBuffer, sendBufferSize, localLocator, remoteLocator, socket);
    }
    return result;
}

bool TCPv4Transport::CheckCRC(const TCPHeader &header, const octet *data, uint32_t size)
{
    uint32_t crc(0);
    for (uint32_t i = 0; i < size; ++i)
    {
        crc = RTCPMessageManager::addToCRC(crc, data[i]);
    }
    return crc == header.crc;
}

void TCPv4Transport::CalculateCRC(TCPHeader &header, const octet *data, uint32_t size)
{
    uint32_t crc(0);
    for (uint32_t i = 0; i < size; ++i)
    {
        crc = RTCPMessageManager::addToCRC(crc, data[i]);
    }
    header.crc = crc;
}

void TCPv4Transport::FillTCPHeader(TCPHeader& header, const octet* sendBuffer, uint32_t sendBufferSize,
    uint16_t logicalPort)
{
    header.length = sendBufferSize + static_cast<uint32_t>(TCPHeader::getSize());
    header.logicalPort = logicalPort;
    CalculateCRC(header, sendBuffer, sendBufferSize);
}

bool TCPv4Transport::Send(const octet* sendBuffer, uint32_t sendBufferSize, const Locator_t& /*localLocator*/,
    const Locator_t& remoteLocator, SocketInfo *socketInfo)
{
    TCPSocketInfo* socket = dynamic_cast<TCPSocketInfo*>(socketInfo);
    if (socket != nullptr && socket->IsConnectionEstablished())
    {
        CDRMessage_t msg;
        TCPHeader tcp_header;
        uint16_t logicalPort = socket->mLogicalPortRouting.find(remoteLocator.get_logical_port())
            != socket->mLogicalPortRouting.end()
            ? socket->mLogicalPortRouting.at(remoteLocator.get_logical_port())
            : remoteLocator.get_logical_port();

        FillTCPHeader(tcp_header, sendBuffer, sendBufferSize, logicalPort);

        RTPSMessageCreator::addCustomContent(&msg, (octet*)&tcp_header, TCPHeader::getSize());
        RTPSMessageCreator::addCustomContent(&msg, sendBuffer, sendBufferSize);

        bool success = false;
        {
            std::unique_lock<std::recursive_mutex> sendLock(*socket->GetWriteMutex());
            success |= SendThroughSocket(msg.buffer, msg.length, remoteLocator, socket);
        }

        return success;
    }
    else
    {
        logWarning(RTCP, " SEND [RTPS] Failed: Connection not established" << remoteLocator.get_logical_port());
        eClock::my_sleep(100);
        return false;
    }
}

/**
    * On TCP, we must receive the header (14 Bytes) and then,
    * the rest of the message, whose length is on the header.
    * TCP Header is transparent to the caller, so receiveBuffer
    * doesn't include it.
    * */
bool TCPv4Transport::Receive(TCPSocketInfo *socketInfo, octet* receiveBuffer,
    uint32_t receiveBufferCapacity, uint32_t& receiveBufferSize, uint16_t& logicalPort)
{
    bool success = false;

    { // lock scope
        std::unique_lock<std::recursive_mutex> scopedLock(*socketInfo->GetReadMutex());
        if (!socketInfo->IsAlive())
        {
            success = false;
        }
        else
        {
            success = true;
            try
            {
                // Read the header
                octet header[TCPHEADER_SIZE];
                size_t bytes_received = read(*socketInfo->getSocket(),
                    asio::buffer(header, TCPHeader::getSize()),
                    transfer_exactly(TCPHeader::getSize()));

                logInfo(RTCP, "[RECEIVE] From: " << socketInfo->getSocket()->remote_endpoint().address() << " to " << socketInfo->getSocket()->local_endpoint().address());

                if (bytes_received != TCPHeader::getSize())
                {
                    logError(RTPS_MSG_IN, "Bad TCP header size: " << bytes_received << "(expected: : " << TCPHeader::getSize() << ")");
                    success = false;
                }
                else
                {
                    TCPHeader tcp_header;
                    memcpy(&tcp_header, header, TCPHeader::getSize());
                    size_t body_size = tcp_header.length - static_cast<uint32_t>(TCPHeader::getSize());
                    //logInfo(RTCP, " Received [TCPHeader] (CRC=" << tcp_header.crc << ")");

                    if (body_size > receiveBufferCapacity)
                    {
                        logError(RTPS_MSG_IN, "Size of incoming TCP message is bigger than buffer capacity: "
                            << body_size << " vs. " << receiveBufferCapacity << ".");
                        success = false;
                    }
                    else
                    {
                        logWarning(RTPS_MSG_IN, "Received MSG. Logical Port" << tcp_header.logicalPort);
                        success = ReadBody(receiveBuffer, receiveBufferCapacity, &receiveBufferSize, socketInfo, body_size);
                        //logInfo(RTCP, " Received [ReadBody]");

                        if (!CheckCRC(tcp_header, receiveBuffer, receiveBufferSize))
                        {
                            logWarning(RTPS_MSG_IN, "Bad TCP header CRC");
                        }

                        if (tcp_header.logicalPort == 0)
                        {
                            //logInfo(RTCP, " Receive [RTCP Control]  (" << receiveBufferSize+bytes_received
                            // << " bytes): " << receiveBufferSize << " bytes.");
                            if (!mRTCPMessageManager->processRTCPMessage(socketInfo, receiveBuffer, body_size))
                            {
                                CloseTCPSocket(socketInfo);
                            }
                            success = false;
                        }
                        else
                        {
                            logicalPort = tcp_header.logicalPort;
                            //logInfo(RTCP, " Receive [RTPS Data]: " << receiveBufferSize << " bytes.");
                        }
                    }
                }
            }
            catch (const asio::error_code& code)
            {
                if ((code == asio::error::eof) || (code == asio::error::connection_reset))
                {
                    // Close the channel
                    logInfo(RTCP, "ASIO [RECEIVE]: " << code.message());
                    CloseTCPSocket(socketInfo);
                }
                success = false;
            }
            catch (const asio::system_error& error)
            {
                // Close the channel
                logInfo(RTCP, "ASIO [RECEIVE]: " << error.what());
                CloseTCPSocket(socketInfo);
                success = false;
            }
        }
    }
    success = success && receiveBufferSize > 0;

    if (!success)
    {
        if (mInputSockets.find(socketInfo->GetLocator().get_physical_port()) != mInputSockets.end())
        {
            auto& sockets = mInputSockets.at(socketInfo->GetLocator().get_physical_port());
            for (auto it = sockets.begin(); it != sockets.end();)
            {
                if (!(*it)->IsAlive())
                {
                    it = sockets.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
    }
    return success;
}

bool TCPv4Transport::ReadBody(octet* receiveBuffer, uint32_t receiveBufferCapacity,
    uint32_t* bytes_received, TCPSocketInfo *pSocketInfo,
    std::size_t body_size)
{
    *bytes_received = static_cast<uint32_t>(read(*pSocketInfo->getSocket(),
        asio::buffer(receiveBuffer, receiveBufferCapacity), transfer_exactly(body_size)));

    if (*bytes_received != body_size)
    {
        logError(RTPS_MSG_IN, "Bad TCP body size: " << bytes_received << "(expected: " << TCPHeader::getSize() << ")");
        return false;
    }

    return true;
}

bool TCPv4Transport::SendThroughSocket(const octet* sendBuffer, uint32_t sendBufferSize,
    const Locator_t& remoteLocator, TCPSocketInfo *socket)
{
    asio::ip::address_v4::bytes_type remoteAddress;
    remoteLocator.copy_IP4_address(remoteAddress.data());
    auto destinationEndpoint = ip::tcp::endpoint(asio::ip::address_v4(remoteAddress),
        static_cast<uint16_t>(remoteLocator.get_port()));

    size_t bytesSent = 0;
    (void)destinationEndpoint;
    logInfo(RTPS_MSG_OUT, "TCPv4: " << sendBufferSize << " bytes TO endpoint: " << destinationEndpoint
        << " FROM " << socket->getSocket()->local_endpoint());

    //logInfo(RTCP, "SOCKET SEND to physical port " << socket->getSocket()->remote_endpoint().port());

    eSocketErrorCodes errorCode;
    bytesSent = Send(socket, sendBuffer, sendBufferSize, errorCode);
    switch (errorCode)
    {
    case eNoError:
        //logInfo(RTCP, " Sent [OK]: " << sendBufferSize << " bytes to locator " << remoteLocator.get_logical_port());
        break;
    default:
        // Close the channel
        logInfo(RTCP, " Sent [FAILED]: " << sendBufferSize << " bytes to locator " << remoteLocator.get_logical_port() << " ERROR=" << errorCode);
        Locator_t prevLocator = socket->GetLocator();

        CloseOutputChannel(socket->GetLocator());

        // Create a new connector to retry the connection.
        OpenAndBindUnicastOutputSocket(prevLocator, nullptr);
        break;
    }

    logInfo(RTPS_MSG_OUT, "SENT " << bytesSent);
    return bytesSent > 0;
}

LocatorList_t TCPv4Transport::NormalizeLocator(const Locator_t& locator)
{
    LocatorList_t list;

    if (locator.is_Any())
    {
        std::vector<IPFinder::info_IP> locNames;
        GetIP4s(locNames);
        for (const auto& infoIP : locNames)
        {
            Locator_t newloc(locator);
            newloc.set_IP4_address(infoIP.locator);
            list.push_back(newloc);
        }
    }
    else
        list.push_back(locator);

    return list;
}

LocatorList_t TCPv4Transport::ShrinkLocatorLists(const std::vector<LocatorList_t>& locatorLists)
{
    LocatorList_t unicastResult;
    for (auto& locatorList : locatorLists)
    {
        LocatorListConstIterator it = locatorList.begin();
        LocatorList_t pendingUnicast;

        while (it != locatorList.end())
        {
            assert((*it).kind == LOCATOR_KIND_TCPv4);

            // Check is local interface.
            auto localInterface = mCurrentInterfaces.begin();
            for (; localInterface != mCurrentInterfaces.end(); ++localInterface)
            {
                if (localInterface->locator.compare_IP4_address(*it))
                {
                    // Loopback locator
                    Locator_t loopbackLocator;
                    loopbackLocator.set_IP4_address(127, 0, 0, 1);
                    loopbackLocator.set_port(it->get_physical_port());
                    pendingUnicast.push_back(loopbackLocator);
                    break;
                }
            }

            if (localInterface == mCurrentInterfaces.end())
                pendingUnicast.push_back(*it);

            ++it;
        }

        unicastResult.push_back(pendingUnicast);
    }

    LocatorList_t result(std::move(unicastResult));
    return result;
}

bool TCPv4Transport::is_local_locator(const Locator_t& locator) const
{
    assert(locator.kind == LOCATOR_KIND_TCPv4);

    if (locator.is_IP4_Local())
        return true;

    for (auto localInterface : mCurrentInterfaces)
        if (locator.compare_IP4_address(localInterface.locator))
        {
            return true;
        }

    return false;
}

void TCPv4Transport::RegisterReceiverResources(TCPSocketInfo* socketInfo, const Locator_t& locator)
{
    // Create one message receiver for each registered receiver resources.
    for (auto it = mReceiverResources.begin(); it != mReceiverResources.end(); ++it)
    {
        if (it->first.compare_IP4_address_and_port(locator) &&
            socketInfo->GetMessageReceiver(it->first.get_logical_port()) == nullptr)
        {
            socketInfo->AddMessageReceiver(it->first.get_logical_port(), it->second->CreateMessageReceiver());
            socketInfo->mLogicalInputPorts.emplace_back(it->first.get_logical_port());
        }
    }
}

#ifdef ASIO_HAS_MOVE
void TCPv4Transport::SocketAccepted(TCPAcceptor* acceptor, const asio::error_code& error, asio::ip::tcp::socket socket)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);

    if (mSocketAcceptors.find(acceptor->mLocator.get_physical_port()) != mSocketAcceptors.end())
    {
        if (!error.value())
        {
            // Store the new connection.
            eProsimaTCPSocket unicastSocket = eProsimaTCPSocket(std::move(socket));
            TCPSocketInfo *socketInfo = new TCPSocketInfo(unicastSocket,
                acceptor->mLocator, false, true, acceptor->mMaxMsgSize);
            socketInfo->ChangeStatus(TCPSocketInfo::eConnectionStatus::eWaitingForBind);
            RegisterReceiverResources(socketInfo, acceptor->mLocator);

            for (auto it = mPendingOutputPorts.begin(); it != mPendingOutputPorts.end(); ++it)
            {
                if (acceptor->mLocator.compare_IP4_address_and_port(*it))
                {
                    BindInputSocket(*it, socketInfo);
                }
            }
            socketInfo->SetThread(new std::thread(&TCPv4Transport::performListenOperation, this, socketInfo));
            socketInfo->SetRTCPThread(new std::thread(&TCPv4Transport::performRTPCManagementThread, this, socketInfo));

            mInputSockets[acceptor->mLocator.get_physical_port()].emplace_back(socketInfo);
            logInfo(RTCP, " Accepted connection (physical local: " << acceptor->mLocator.get_physical_port() << ", remote: " << socketInfo->getSocket()->remote_endpoint().port() <<  ") IP: " << socketInfo->getSocket()->remote_endpoint().address());
        }
        else
        {
            logInfo(RTCP, " Accepting connection failed (error: " << error.message() << ")");
        }

        // Accept new connections for the same port.
        mSocketAcceptors.at(acceptor->mLocator.get_physical_port())->Accept(this, mService);
    }
}
#else

void TCPv4Transport::SocketAccepted(TCPAcceptor* acceptor, const asio::error_code& error)
{
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);

    if (mSocketAcceptors.find(acceptor->mLocator.get_physical_port()) != mSocketAcceptors.end())
    {
        if (!error.value())
        {
            // Store the new connection and release the pointer to the acceptor to create a new one for the next waiting.
            eProsimaTCPSocket unicastSocket = eProsimaTCPSocket(acceptor->mSocket);
            acceptor->mSocket = nullptr;

            TCPSocketInfo *socketInfo = new TCPSocketInfo(unicastSocket,
                acceptor->mLocator, false, true, acceptor->mMaxMsgSize);
            socketInfo->SetThread(new std::thread(&TCPv4Transport::performListenOperation, this, socketInfo));
            socketInfo->SetRTCPThread(new std::thread(&TCPv4Transport::performRTPCManagementThread, this, socketInfo));
            socketInfo->ChangeStatus(TCPSocketInfo::eConnectionStatus::eWaitingForBind);

            // Create one message receiver for each registered receiver resources.
            RegisterReceiverResources(socketInfo, acceptor->mLocator);
            mInputSockets[acceptor->mLocator.get_physical_port()].emplace_back(socketInfo);

            logInfo(RTCP, " Accepted connection (physical local: " << acceptor->mLocator.get_physical_port() << ", remote: " << socketInfo->getSocket()->remote_endpoint().port() <<  ") IP: " << socketInfo->getSocket()->remote_endpoint().address());
        }
        else
        {
            logInfo(RTCP, " Accepting connection failed (error: " << error.message() << ")");
        }

        // Accept new connections for the same port.
        mSocketAcceptors.at(acceptor->mLocator.get_physical_port())->Accept(this, mService);
    }
}
#endif

void TCPv4Transport::SocketConnected(Locator_t& locator, SenderResource *senderResource, const asio::error_code& error)
{
    std::string value = error.message();
    std::unique_lock<std::recursive_mutex> scopedLock(mSocketsMapMutex);
    if (mSocketConnectors.find(locator) != mSocketConnectors.end())
    {
        auto& pendingConector = mSocketConnectors.at(locator);
        if (!error.value())
        {
            TCPSocketInfo *outputSocket = new TCPSocketInfo(pendingConector->m_socket,
                locator, true, false, false);

            // Create one message receiver for each registered receiver resources.
            RegisterReceiverResources(outputSocket, locator);
            outputSocket->ChangeStatus(TCPSocketInfo::eConnectionStatus::eConnected);
            outputSocket->SetThread(new std::thread(&TCPv4Transport::performListenOperation, this, outputSocket));
            outputSocket->SetRTCPThread(new std::thread(&TCPv4Transport::performRTPCManagementThread, this, outputSocket));
            mOutputSockets.push_back(outputSocket);
            BindOutputChannel(locator);

            logInfo(RTCP, " Socket Connected (physical remote: " << locator.get_physical_port()
                << ", local: " << outputSocket->getSocket()->local_endpoint().port()
                << ") IP: " << outputSocket->getSocket()->remote_endpoint().address());

            // RTCP Control Message
            mRTCPMessageManager->sendConnectionRequest(outputSocket, mConfiguration_.metadata_logical_port);
            if (senderResource != nullptr)
            {
                senderResource->SetSocketInfo(outputSocket);
                AssociateSenderToSocket(outputSocket, senderResource);
            }
        }
        else
        {
            eClock::my_sleep(100);
            pendingConector->RetryConnect(mService, this, senderResource);
        }
    }
}

size_t TCPv4Transport::Send(TCPSocketInfo *socketInfo, const octet *data,
        size_t size, eSocketErrorCodes &errorCode) const
{
    size_t bytesSent = 0;
    try
    {
        std::unique_lock<std::recursive_mutex> scopedLock(*socketInfo->GetWriteMutex());
        bytesSent = socketInfo->getSocket()->send(asio::buffer(data, size));
        logInfo(RTCP, "[SENT] From: " << socketInfo->getSocket()->local_endpoint().address() << " to " << socketInfo->getSocket()->remote_endpoint().address());
        errorCode = eSocketErrorCodes::eNoError;
    }
    catch (const asio::error_code& error)
    {
        logInfo(RTCP, "ASIO [SEND]: " << error.message());
        if ((asio::error::eof == error) || (asio::error::connection_reset == error))
        {
            errorCode = eSocketErrorCodes::eBrokenPipe;
        }
        else
        {
            errorCode = eSocketErrorCodes::eAsioError;
        }
    }
    catch (const asio::system_error& error)
    {
        logInfo(RTCP, "ASIO [SEND]: " << error.what());
        errorCode = eSocketErrorCodes::eSystemError;
    }
    catch (const std::exception& error)
    {
        logInfo(RTCP, "ASIO [SEND]: " << error.what());
        errorCode = eSocketErrorCodes::eException;
    }

    return bytesSent;
}

size_t TCPv4Transport::Send(TCPSocketInfo *socketInfo, const octet *data, size_t size) const
{
    eSocketErrorCodes error;
    return Send(socketInfo, data, size, error);
}

void TCPv4Transport::CloseTCPSocket(TCPSocketInfo *socketInfo)
{
    if (socketInfo->IsAlive())
    {
        if (socketInfo->GetIsInputSocket())
        {
            CloseInputSocket(socketInfo);
        }
        else
        {
            Locator_t prevLocator = socketInfo->GetLocator();
            CloseOutputChannel(prevLocator);

            // Create a new connector to retry the connection.
            OpenAndBindUnicastOutputSocket(prevLocator, nullptr);
        }
    }
}

void TCPv4Transport::CleanDeletedSockets()
{
    std::unique_lock<std::recursive_mutex> scopedLock(mDeletedSocketsPoolMutex);
    for (auto it = mDeletedSocketsPool.begin(); it != mDeletedSocketsPool.end(); ++it)
    {
        std::thread* rtcpThread = (*it)->ReleaseRTCPThread();
        std::thread* thread = (*it)->ReleaseThread();
        if (rtcpThread != nullptr)
        {
            rtcpThread->join();
            delete(rtcpThread);
        }
        if (thread != nullptr)
        {
            thread->join();
            delete(thread);
        }

        delete(*it);
    }
    mDeletedSocketsPool.clear();
}

void TCPv4Transport::ReleaseTCPSocket(TCPSocketInfo *socketInfo)
{
    if (socketInfo != nullptr && socketInfo->IsAlive())
    {
        CleanDeletedSockets();

        socketInfo->Disable();
        socketInfo->ChangeStatus(TCPSocketInfo::eConnectionStatus::eDisconnected);
        socketInfo->getSocket()->cancel();
        socketInfo->getSocket()->close();

        // Remove from senders
        auto it = mSocketToSenders.find(socketInfo);
        if (it != mSocketToSenders.end())
        {
            auto& senders = mSocketToSenders.at(socketInfo);
            for (auto& sender : senders)
            {
                sender->SetSocketInfo(nullptr);
            }
        }

        {
            std::unique_lock<std::recursive_mutex> scopedLock(mDeletedSocketsPoolMutex);
            mDeletedSocketsPool.emplace_back(socketInfo);
        }
        socketInfo = nullptr;
    }
}

void TCPv4Transport::AssociateSenderToSocket(TCPSocketInfo *socket, SenderResource *sender) const
{
    auto it = mSocketToSenders.find(socket);
    if (it == mSocketToSenders.end())
    {
        mSocketToSenders[socket].emplace_back(sender);
    }
    else
    {
        auto srit = std::find((*it).second.begin(), (*it).second.end(), sender);
        if (srit == (*it).second.end())
        {
            (*it).second.emplace_back(sender);
        }
    }
}

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima
