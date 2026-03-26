#ifndef _SDK_CLIENT_HPP_
#define _SDK_CLIENT_HPP_

#include <chrono>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <memory>
#include <sstream>
#include <functional>
#include <mutex>
#include <zmq.hpp>

#include "ClientPlatformSpecific.hpp"
#include "ManusSDK.h"


enum class ClientReturnCode : int
{
	ClientReturnCode_Success = 0,
	ClientReturnCode_FailedPlatformSpecificInitialization,
	ClientReturnCode_FailedToResizeWindow,
	ClientReturnCode_FailedToInitialize,
	ClientReturnCode_FailedToFindHosts,
	ClientReturnCode_FailedToConnect,
	ClientReturnCode_UnrecognizedStateEncountered,
	ClientReturnCode_FailedToShutDownSDK,
	ClientReturnCode_FailedPlatformSpecificShutdown,
	ClientReturnCode_FailedToRestart,
	ClientReturnCode_FailedWrongTimeToGetData,
	ClientReturnCode_MAX_CLIENT_RETURN_CODE_SIZE
};

class ClientSkeleton
{
public:
	RawSkeletonInfo info;
	std::vector<SkeletonNode> nodes;
};

class ClientSkeletonCollection
{
public:
	std::vector<ClientSkeleton> skeletons;
};

class SDKClient : public SDKClientPlatformSpecific
{
public:
	SDKClient();
	~SDKClient();

	ClientReturnCode Initialize();
	ClientReturnCode Run();
	ClientReturnCode ShutDown();

	static void OnConnectedCallback(const ManusHost* const p_Host);
	static void OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_Skeleton);
	static void OnLandscapeCallback(const Landscape* const p_Landscape);
	float RoundFloatValue(float p_Value, int p_NumDecimalsToKeep);
	void AdvanceConsolePosition(short int p_Y);

protected:
	virtual ClientReturnCode InitializeSDK();
	virtual ClientReturnCode RegisterAllCallbacks();
	virtual ClientReturnCode LookingForHosts();
	virtual ClientReturnCode ConnectingToCore();
	ClientReturnCode Connect();

protected:
	static SDKClient* s_Instance;
	uint32_t m_FrameId = 0;
	Landscape* m_Landscape = nullptr;
	uint32_t m_FirstLeftGloveID = 0;
	uint32_t m_FirstRightGloveID = 0;

	std::shared_ptr<zmq::context_t> m_ZmqPubContext;
	std::shared_ptr<zmq::socket_t> m_ZmqPublisher;

	uint32_t m_NumberOfHostsFound = 0;
	uint32_t m_SecondsToFindHosts = 2;
	std::string m_ZmqHost = "tcp://192.168.10.222:2044";

	std::unique_ptr<ManusHost[]> m_AvailableHosts = nullptr;

	std::mutex m_SkeletonMutex;
	ClientSkeletonCollection* m_NextSkeleton = nullptr;
	ClientSkeletonCollection* m_Skeleton = nullptr;
};

#endif
