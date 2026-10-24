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
 *   numOfPlayers * {int32 number, int32 ownTeam, int32 upright, float x, float y, float rotation}
 */

#pragma once

#include "Framework/Module.h"
#include "Network/UdpComm.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

MODULE(MuJoCoWorldProvider,
{,
  REQUIRES(CameraIntrinsics),
  PROVIDES(CameraInfo),
  PROVIDES(FrameInfo),
  PROVIDES(GroundTruthWorldState),
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

  void update(CameraInfo& theCameraInfo) override;
  void update(FrameInfo& theFrameInfo) override;
  void update(GroundTruthWorldState& theGroundTruthWorldState) override;

public:
  MuJoCoWorldProvider();
};
