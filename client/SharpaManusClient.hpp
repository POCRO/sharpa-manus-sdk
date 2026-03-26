#ifndef _SDK_CLIENT_HPP_
#define _SDK_CLIENT_HPP_ 

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <fstream>
#include <memory>
#include <sstream>
#include <functional>
#include <mutex>
#include <zmq.hpp>

#include "ClientPlatformSpecific.hpp"
#include "ManusSDK.h"

// Add ConnectionType enum
enum class ConnectionType : int
{
	ConnectionType_Integrated = 0,
	ConnectionType_Local,
	ConnectionType_Network
};

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

	static void OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_Skeleton);
	static void OnLandscapeCallback(const Landscape* const p_Landscape);

	// Add calibration related functions
	void LoadGloveCalibration(uint32_t p_GloveId, const std::string& p_CalibrationFileName);
	void TestCalibrationFileLoading(); // Test calibration file loading functionality

protected:
	virtual ClientReturnCode InitializeSDK();
	virtual ClientReturnCode RegisterAllCallbacks();
	ClientReturnCode Connect();

protected:
	static SDKClient* s_Instance;
	uint32_t m_FrameId = 0;
	Landscape* m_Landscape = nullptr;
	uint32_t m_FirstLeftGloveID = UINT32_MAX;  // Initialize with invalid value
	uint32_t m_FirstRightGloveID = UINT32_MAX;  // Initialize with invalid value

	std::shared_ptr<zmq::context_t> m_ZmqPubContext;
	std::shared_ptr<zmq::socket_t> m_ZmqPublisher;

	uint32_t m_NumberOfHostsFound = 0;
	uint32_t m_SecondsToFindHosts = 2;
	std::string m_ZmqHost = "tcp://127.0.0.1:2044";  // Bind to localhost port 2044

	std::unique_ptr<ManusHost[]> m_AvailableHosts = nullptr;

	std::mutex m_SkeletonMutex;
	ClientSkeletonCollection* m_NextSkeleton = nullptr;
	ClientSkeletonCollection* m_Skeleton = nullptr;

	// Add member variables for integrated mode support
	ConnectionType m_ConnectionType = ConnectionType::ConnectionType_Integrated;
	bool m_Running = true;
	uint32_t m_FrameCounter = 0;
	
	// Add calibration status flags
	bool m_LeftGloveCalibrationLoaded = false;
	bool m_RightGloveCalibrationLoaded = false;
};

#endif
