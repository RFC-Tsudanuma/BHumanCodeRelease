/**
 * @file MuJoCoWorldProvider.cpp
 *
 * This file implements a module that receives the ground truth state of the world
 * from an external MuJoCo simulation.
 */

#include "MuJoCoWorldProvider.h"
#include "Framework/Settings.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include <cstdint>
#include <cstring>

MAKE_MODULE(MuJoCoWorldProvider);

namespace
{
  struct Player
  {
    std::int32_t number;
    std::int32_t ownTeam;
    std::int32_t upright;
    float pose[3];
  };

  struct Packet
  {
    char header[4];
    float ownPose[3];
    float ball[6];
    std::uint32_t numOfPlayers;
    Player players[20];
  };

  constexpr std::size_t headerSize = offsetof(Packet, players);
}

MuJoCoWorldProvider::MuJoCoWorldProvider()
{
  VERIFY(socket.setBlocking(false));
  VERIFY(socket.bind("0.0.0.0", portBase + Global::getSettings().playerNumber));
}

void MuJoCoWorldProvider::update(CameraInfo& theCameraInfo)
{
  theCameraInfo.width = width;
  theCameraInfo.height = height;
  theCameraInfo.openingAngleWidth = theCameraIntrinsics.openingAngleWidth;
  theCameraInfo.openingAngleHeight = theCameraIntrinsics.openingAngleHeight;
  theCameraInfo.opticalCenter = {theCameraIntrinsics.opticalCenter.x() * width,
                                 theCameraIntrinsics.opticalCenter.y() * height};
  theCameraInfo.updateFocalLength();
}

void MuJoCoWorldProvider::update(FrameInfo& theFrameInfo)
{
  theFrameInfo.time = Time::getCurrentSystemTime();
}

void MuJoCoWorldProvider::update(GroundTruthWorldState& theGroundTruthWorldState)
{
  // Wait for the next packet. Only the newest one is used.
  Packet packet;
  int size = 0;
  for(int waited = 0; size <= 0 && waited < maxWaitTime; ++waited)
    if((size = socket.read(reinterpret_cast<char*>(&packet), sizeof(packet))) <= 0)
      Thread::sleep(1);
  for(Packet newer; true;)
  {
    const int newerSize = socket.read(reinterpret_cast<char*>(&newer), sizeof(newer));
    if(newerSize <= 0)
      break;
    packet = newer;
    size = newerSize;
  }

  if(size < static_cast<int>(headerSize) || std::memcmp(packet.header, "BHMW", 4)
     || size != static_cast<int>(headerSize + packet.numOfPlayers * sizeof(Player)))
    return;

  theGroundTruthWorldState.ownPose = Pose2f(packet.ownPose[2], packet.ownPose[0], packet.ownPose[1]);
  theGroundTruthWorldState.balls.resize(1);
  theGroundTruthWorldState.balls[0].position = Vector3f(packet.ball[0], packet.ball[1], packet.ball[2]);
  theGroundTruthWorldState.balls[0].velocity = Vector3f(packet.ball[3], packet.ball[4], packet.ball[5]);
  theGroundTruthWorldState.ownTeamPlayers.clear();
  theGroundTruthWorldState.opponentTeamPlayers.clear();
  for(std::uint32_t i = 0; i < packet.numOfPlayers; ++i)
  {
    const Player& p = packet.players[i];
    GroundTruthWorldState::GroundTruthPlayer player;
    player.number = p.number;
    player.pose = Pose2f(p.pose[2], p.pose[0], p.pose[1]);
    player.upright = p.upright != 0;
    (p.ownTeam ? theGroundTruthWorldState.ownTeamPlayers : theGroundTruthWorldState.opponentTeamPlayers).push_back(player);
  }
}
