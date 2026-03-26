#include "SharpaManusClient.hpp"
#include "ManusSDKTypes.h"
#include "ClientLogging.hpp"
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <zmq.hpp>
#include <cmath>
#include <chrono>
#include "sharpa_hand.pb.h"
#include "ClientPlatformSpecific.hpp"

#define M_PI 3.14159265358979323846

using ManusSDK::ClientLog;

SDKClient* SDKClient::s_Instance = nullptr;

SDKClient::SDKClient()
{
	s_Instance = this;
	m_FrameId = 0;
}

SDKClient::~SDKClient()
{
	s_Instance = nullptr;
}

ClientReturnCode SDKClient::Initialize()
{
	if (!PlatformSpecificInitialization())
	{
		return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificInitialization;
	}

	m_ZmqPubContext = std::make_shared<zmq::context_t>(1);
	m_ZmqPublisher = std::make_shared<zmq::socket_t>(*m_ZmqPubContext, ZMQ_PUB);
	
	// 设置发送水位，防止内存溢出
	int sndhwm = 10;  // 发送高水位标记
	m_ZmqPublisher->set(zmq::sockopt::sndhwm, sndhwm);
	int linger = 0;   // 立即关闭
	m_ZmqPublisher->set(zmq::sockopt::linger, linger);
	
	m_ZmqPublisher->bind(m_ZmqHost);

	const ClientReturnCode t_IntializeResult = InitializeSDK();
	if (t_IntializeResult != ClientReturnCode::ClientReturnCode_Success)
	{
		ClientLog::error("Failed to initialize the SDK. Are you sure the correct ManusSDKLibary is used?");
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::Connect()
{
	ClientReturnCode t_Result;

	t_Result = LookingForHosts();
	if (t_Result != ClientReturnCode::ClientReturnCode_Success) {
		return t_Result;
	}

	t_Result = ConnectingToCore();
	if (t_Result != ClientReturnCode::ClientReturnCode_Success) { 
		return t_Result; 
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::Run()
{
	ClearConsole();

	while (Connect() != ClientReturnCode::ClientReturnCode_Success)
	{
		ClientLog::print("minimal client could not connect.trying again in a second.");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	
	return ClientReturnCode::ClientReturnCode_Success;
}


ClientReturnCode SDKClient::ShutDown()
{
	const SDKReturnCode t_Result = CoreSdk_ShutDown();
	if (t_Result != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to shut down the SDK wrapper. The value returned was {}.", (int32_t)t_Result);
		return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;
	}

	if (!PlatformSpecificShutdown())
	{
		return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificShutdown;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::ConnectingToCore()
{
	SDKReturnCode t_ConnectResult = SDKReturnCode::SDKReturnCode_Error;
	t_ConnectResult = CoreSdk_ConnectToHost(m_AvailableHosts[0]);

	if (t_ConnectResult == SDKReturnCode::SDKReturnCode_NotConnected)
	{
		return ClientReturnCode::ClientReturnCode_Success;
	}
	if (t_ConnectResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to connect to Core. The error given was {}.", (int32_t)t_ConnectResult);
		return ClientReturnCode::ClientReturnCode_FailedToConnect;
	}
	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::LookingForHosts()
{
	ClientLog::print("Looking for hosts...");

	const SDKReturnCode t_StartResult = CoreSdk_LookForHosts(m_SecondsToFindHosts, false);
	if (t_StartResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to look for hosts. The error given was {}.", (int32_t)t_StartResult);
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	m_NumberOfHostsFound = 0;
	const SDKReturnCode t_NumberResult = CoreSdk_GetNumberOfAvailableHostsFound(&m_NumberOfHostsFound);
	if (t_NumberResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to get the number of available hosts. The error given was {}.", (int32_t)t_NumberResult);
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	if (m_NumberOfHostsFound == 0)
	{
		ClientLog::warn("No hosts found.");
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	m_AvailableHosts.reset(new ManusHost[m_NumberOfHostsFound]);
	const SDKReturnCode t_HostsResult = CoreSdk_GetAvailableHostsFound(m_AvailableHosts.get(), m_NumberOfHostsFound);
	if (t_HostsResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to get the available hosts. The error given was {}.", (int32_t)t_HostsResult);
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

void SDKClient::OnConnectedCallback(const ManusHost* const p_Host)
{
	ClientLog::print("Connected to manus core.");

	ManusVersion t_SdkVersion;
	ManusVersion t_CoreVersion;
	bool t_IsCompatible;

	const SDKReturnCode t_Result = CoreSdk_GetVersionsAndCheckCompatibility(&t_SdkVersion, &t_CoreVersion, &t_IsCompatible);

	if (t_Result == SDKReturnCode::SDKReturnCode_Success)
	{
		const std::string t_Versions = "Sdk version : " + std::string(t_SdkVersion.versionInfo) + ", Core version : " + std::string(t_CoreVersion.versionInfo) + ".";

		if (t_IsCompatible)
		{
			ClientLog::print("Versions are compatible.{}", t_Versions);
		}
		else
		{
			ClientLog::warn("Versions are not compatible with each other.{}", t_Versions);
		}
	}
	else
	{
		ClientLog::error("Failed to get the versions from the SDK. The value returned was {}.", (int32_t)t_Result);
	}

	uint32_t t_SessionId;
	const SDKReturnCode t_SessionIdResult = CoreSdk_GetSessionId(&t_SessionId);
	if (t_SessionIdResult == SDKReturnCode::SDKReturnCode_Success && t_SessionId != 0)
	{
		ClientLog::print("Session Id: {}", t_SessionId);
	}
	else
	{
		ClientLog::print("Failed to get the Session ID from Core. The value returned was{}.", (int32_t)t_SessionIdResult);
	}

	const SDKReturnCode t_HandMotionResult = CoreSdk_SetRawSkeletonHandMotion(HandMotion_None);
	if (t_HandMotionResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::print("Failed to set the hand motion mode. The value returned was {}.", (int32_t)t_HandMotionResult);
	}
}

void SDKClient::OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_SkeletonStreamInfo)
{	
	if (s_Instance)
	{
		ClientSkeletonCollection* t_NxtClientSkeleton = new ClientSkeletonCollection();
		t_NxtClientSkeleton->skeletons.resize(p_SkeletonStreamInfo->skeletonsCount);

		proto::MocapKeypoints mocap_keypoints_msg;

		ManusTimestamp t_Timestamp = p_SkeletonStreamInfo->publishTime;
		uint64_t timestamp = t_Timestamp.time;
		uint64_t sec = timestamp / 1000000000;
		uint64_t nanosec = timestamp % 1000000000;

		std::string id = std::to_string(s_Instance->m_FrameId);
		s_Instance->m_FrameId++;

		proto::Header* header_msg = mocap_keypoints_msg.mutable_header();
		proto::Stamp* stamp_msg = header_msg->mutable_stamp();
		stamp_msg->set_sec(sec);
		stamp_msg->set_nanosec(nanosec);
		header_msg->set_frame_id(id);

		for (uint32_t i = 0; i < p_SkeletonStreamInfo->skeletonsCount; i++)
		{
			CoreSdk_GetRawSkeletonInfo(i, &t_NxtClientSkeleton->skeletons[i].info);

			Side t_Side = Side::Side_Invalid;
			std::string t_GloveSide = "Invalid";
			
			if (t_NxtClientSkeleton->skeletons[i].info.gloveId == s_Instance->m_FirstLeftGloveID){
				t_Side = Side::Side_Left;
				t_GloveSide = "Left";
			}
			else if (t_NxtClientSkeleton->skeletons[i].info.gloveId == s_Instance->m_FirstRightGloveID){
				t_Side = Side::Side_Right;
				t_GloveSide = "Right";
			}
			else{
				continue;
			}

			t_NxtClientSkeleton->skeletons[i].nodes.resize(t_NxtClientSkeleton->skeletons[i].info.nodesCount);
			t_NxtClientSkeleton->skeletons[i].info.publishTime = p_SkeletonStreamInfo->publishTime;
			CoreSdk_GetRawSkeletonData(i, t_NxtClientSkeleton->skeletons[i].nodes.data(), t_NxtClientSkeleton->skeletons[i].info.nodesCount);

			if (t_NxtClientSkeleton->skeletons[i].nodes.size() > 0)
			{
				const auto& rootNode = t_NxtClientSkeleton->skeletons[i].nodes[0];
				
				Eigen::Vector3f rootPos(rootNode.transform.position.x, rootNode.transform.position.y, rootNode.transform.position.z);
				Eigen::Quaternionf rootRot(rootNode.transform.rotation.w, rootNode.transform.rotation.x, 
										rootNode.transform.rotation.y, rootNode.transform.rotation.z);
				rootRot.normalize();
				
				std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> allPoints;
				
				for (int j = 0; j < t_NxtClientSkeleton->skeletons[i].nodes.size(); j++)
				{
					const auto& node = t_NxtClientSkeleton->skeletons[i].nodes[j];
					Eigen::Vector3f nodePos(node.transform.position.x, node.transform.position.y, node.transform.position.z);
					Eigen::Quaternionf nodeRot(node.transform.rotation.w, node.transform.rotation.x, 
											node.transform.rotation.y, node.transform.rotation.z);
					nodeRot.normalize();
					
					Eigen::Vector3f relativePos = rootRot.inverse() * (nodePos - rootPos);
					Eigen::Quaternionf relativeRot = rootRot.inverse()*nodeRot;
					
					float angle = - M_PI / 2.0f;
					Eigen::Quaternionf yRotation90(cos(angle/2), 0, sin(angle/2), 0);
					
					Eigen::Vector3f rotatedPos = yRotation90 * relativePos;
					Eigen::Quaternionf rotatedRot = yRotation90 * relativeRot * yRotation90.inverse();
					
					// for thumb tip, rotate 45 degrees around z axis
					if (j == 4)
					{
						float rotAngle = 0.0f;
						if (t_Side == Side::Side_Left)
						{
							rotAngle = 45.f;
						}
						else if (t_Side == Side::Side_Right)
						{
							rotAngle = -45.f;
						}
						float zAngle = rotAngle * M_PI / 180.0f; 
						Eigen::Quaternionf zRotation(cos(zAngle/2), 0, 0, sin(zAngle/2));
						rotatedRot = rotatedRot * zRotation;
					}
					
					allPoints.push_back(std::make_pair(rotatedPos, rotatedRot));
				}
				
				for (const auto& point : allPoints)
				{
					proto::Pose* hand_pose;
					if (t_Side == Side::Side_Left)
					{
						hand_pose = mocap_keypoints_msg.add_left_mocap_pose();
					}
					else if (t_Side == Side::Side_Right)
					{
						hand_pose = mocap_keypoints_msg.add_right_mocap_pose();
					}
					proto::Point* joint_position = hand_pose->mutable_position();
					proto::Quaternion* joint_orientation = hand_pose->mutable_orientation();

					joint_position->set_x(point.first.x());
					joint_position->set_y(point.first.y());
					joint_position->set_z(point.first.z());
					joint_orientation->set_w(point.second.w());
					joint_orientation->set_x(point.second.x());
					joint_orientation->set_y(point.second.y());
					joint_orientation->set_z(point.second.z());
				}
				
				auto now = std::chrono::system_clock::now();
				auto duration = now.time_since_epoch();
				auto sys_sec = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
				auto sys_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;
				
				ClientLog::info("Frame[{}] skeletion: {} is published. - System time: {}.{}s", 
					std::to_string(s_Instance->m_FrameId), 
					t_GloveSide,
					std::to_string(sys_sec),
					std::to_string(sys_nanosec));
			}
		}
		
		std::string serialized_data;
		mocap_keypoints_msg.SerializeToString(&serialized_data);
		zmq::message_t zmq_msg(serialized_data.size());
		memcpy(zmq_msg.data(), serialized_data.data(), serialized_data.size());

		s_Instance->m_ZmqPublisher->send(zmq_msg, zmq::send_flags::none);
	}
}

void SDKClient::OnLandscapeCallback(const Landscape* const p_Landscape)
{
	if (s_Instance == nullptr)return;
	if (s_Instance->m_Landscape != nullptr) return;

	Landscape* t_Landscape = new Landscape(*p_Landscape);
	s_Instance->m_Landscape = t_Landscape;

	for (size_t i = 0; i < s_Instance->m_Landscape->gloveDevices.gloveCount; i++)
	{
		if (s_Instance->m_Landscape->gloveDevices.gloves[i].side == Side::Side_Left)
		{
			s_Instance->m_FirstLeftGloveID = s_Instance->m_Landscape->gloveDevices.gloves[i].id;
			ClientLog::info("First left glove ID: {}", s_Instance->m_FirstLeftGloveID);
			continue;
		}
		if (s_Instance->m_Landscape->gloveDevices.gloves[i].side == Side::Side_Right)
		{
			s_Instance->m_FirstRightGloveID = s_Instance->m_Landscape->gloveDevices.gloves[i].id;
			ClientLog::info("First right glove ID: {}", s_Instance->m_FirstRightGloveID);
			continue;
		}
	}
}

ClientReturnCode SDKClient::RegisterAllCallbacks()
{
	const SDKReturnCode t_RegisterConnectCallbackResult = CoreSdk_RegisterCallbackForOnConnect(*OnConnectedCallback);
	if (t_RegisterConnectCallbackResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to register callback function for after connecting to Manus Core. The value returned was {}.", (int32_t)t_RegisterConnectCallbackResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	const SDKReturnCode t_RegisterSkeletonCallbackResult = CoreSdk_RegisterCallbackForRawSkeletonStream(*OnRawSkeletonStreamCallback);
	if (t_RegisterSkeletonCallbackResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to register callback function for processing skeletal data from Manus Core. The value returned was {}.", (int32_t)t_RegisterSkeletonCallbackResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	const SDKReturnCode t_RegisterLandscapeCallbackResult = CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback);
	if (t_RegisterLandscapeCallbackResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to register callback for landscape from Manus Core. The value returned was {}.", (int32_t)t_RegisterLandscapeCallbackResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::InitializeSDK()
{

	SDKReturnCode t_InitializeResult;
	t_InitializeResult = CoreSdk_InitializeCore();

	if (t_InitializeResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to initialize the Manus Core SDK. The value returned was {}.", (int32_t)t_InitializeResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	const ClientReturnCode t_CallBackResults = RegisterAllCallbacks();
	if (t_CallBackResults != ::ClientReturnCode::ClientReturnCode_Success)
	{
		ClientLog::error("Failed to initialize callbacks.");
		return t_CallBackResults;
	}

	CoordinateSystemVUH t_VUH;
	CoordinateSystemVUH_Init(&t_VUH);
	t_VUH.handedness = Side::Side_Right;
	t_VUH.up = AxisPolarity::AxisPolarity_PositiveZ;
	t_VUH.view = AxisView::AxisView_XFromViewer;
	t_VUH.unitScale = 1.0f;

	const SDKReturnCode t_CoordinateResult = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, true);

	if (t_CoordinateResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to initialize the Manus Core SDK coordinate system. The value returned was {}.", (int32_t)t_InitializeResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

int main(int argc, char* argv[])
{
	ManusSDK::ClientLog::print("Starting SDK client!");
	
	ClientReturnCode t_Result;
	SDKClient t_SDKClient;

	t_Result = t_SDKClient.Initialize();

	if (t_Result != ClientReturnCode::ClientReturnCode_Success)
	{
		t_SDKClient.ShutDown();
		return static_cast<int>(t_Result);
	}
	ManusSDK::ClientLog::print("SDK client is initialized.");

	t_Result = t_SDKClient.Run();
	if (t_Result != ClientReturnCode::ClientReturnCode_Success)
	{
		t_SDKClient.ShutDown();
		return static_cast<int>(t_Result);
	}

	ManusSDK::ClientLog::print("SDK client is done, shutting down.");
	t_Result = t_SDKClient.ShutDown();
	return static_cast<int>(t_Result);
}