/**
 * @file MuJoCoWorldProvider.h
 *
 * This file declares a module that receives the ground truth state of the world
 * from an external MuJoCo simulation (see Src/Apps/BHumanMuJoCo). It replaces the
 * camera: the thread it runs in is paced by the packets received.
 * The sender already transforms everything into B-Human's field coordinates of
 * this robot's team (mm, rad).
 *
 * Packet layout (little endian):
 *   char[4] "BHMW",
 *   float ownPose[3] (x, y, rotation),
 *   float ball[6] (position x, y, z, velocity x, y, z),
 *   uint32 numOfPlayers,
 *   numOfPlayers * {int32 number, int32 ownTeam, int32 upright, float x, float y, float rotation},
 *   optional uint32 age (time since the state was sampled in ms, i.e. the simulated camera latency)
 *
 * As on the robot, FrameInfo is the time the "image" was taken. The Color thread therefore
 * uses the head pose and odometry of that time, which is what makes a delayed state consistent.
 */

#pragma once

#include "Framework/Module.h"
#include "Network/UdpComm.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/StaticInitialPose.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

MODULE(MuJoCoWorldProvider,
{,
  REQUIRES(CameraIntrinsics),
  PROVIDES(CameraInfo),
  PROVIDES(FrameInfo),
  REQUIRES(FrameInfo), // The packet is received when providing FrameInfo, so everything else must come later.
  PROVIDES(GroundTruthWorldState),
  PROVIDES(StaticInitialPose),
  LOADS_PARAMETERS(
  {,
    (int) portBase, /**< The packets are received on port portBase + player number. */
    (int) maxWaitTime, /**< Maximum time to wait for the next packet (in ms). */
    (int) width, /**< The width of the simulated camera image. */
    (int) height, /**< The height of the simulated camera image. */
  }),
});

class MuJoCoWorldProvider : public MuJoCoWorldProviderBase
{
  UdpComm socket;
  GroundTruthWorldState world; /**< The state of the latest packet. */
  unsigned age = 0; /**< The age of the latest packet (in ms). */
  bool received = false; /**< Was a packet received yet? */
  Pose2f spawnPose; /**< The robot's pose in the first packet. The referee also returns it there after a penalty. */

  /** Waits for the next packet. Only the newest one is used. */
  void receive();

  void update(CameraInfo& theCameraInfo) override;
  void update(FrameInfo& frameInfo) override;
  void update(GroundTruthWorldState& groundTruthWorldState) override;
  void update(StaticInitialPose& staticInitialPose) override;

public:
  MuJoCoWorldProvider();
};
