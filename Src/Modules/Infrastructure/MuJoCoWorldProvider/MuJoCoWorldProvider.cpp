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
#include <algorithm>
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

void MuJoCoWorldProvider::update(FrameInfo& frameInfo)
{
  receive();
  frameInfo.time = std::max(frameInfo.time, Time::getCurrentSystemTime() - age);
}

void MuJoCoWorldProvider::update(GroundTruthWorldState& groundTruthWorldState)
{
  groundTruthWorldState = world;
}

void MuJoCoWorldProvider::update(StaticInitialPose& staticInitialPose)
{
  staticInitialPose.isActive = received;
  staticInitialPose.staticPoseOnField = spawnPose;
}

void MuJoCoWorldProvider::receive()
{
  alignas(Packet) char data[sizeof(Packet) + sizeof(std::uint32_t)];
  const Packet& packet = *reinterpret_cast<const Packet*>(data);
  int size = 0;
  for(int waited = 0; size <= 0 && waited < maxWaitTime; ++waited)
    if((size = socket.read(data, sizeof(data))) <= 0)
      Thread::sleep(1);
  char newer[sizeof(data)];
  for(int newerSize; (newerSize = socket.read(newer, sizeof(newer))) > 0; size = newerSize)
    std::memcpy(data, newer, newerSize);

  if(size < static_cast<int>(headerSize) || std::memcmp(packet.header, "BHMW", 4) || packet.numOfPlayers > 20)
    return;
  const std::size_t sizeWithoutAge = headerSize + packet.numOfPlayers * sizeof(Player);
  if(static_cast<std::size_t>(size) != sizeWithoutAge && static_cast<std::size_t>(size) != sizeWithoutAge + sizeof(std::uint32_t))
    return;

  std::uint32_t packetAge = 0;
  if(static_cast<std::size_t>(size) > sizeWithoutAge)
    std::memcpy(&packetAge, data + sizeWithoutAge, sizeof(packetAge));
  age = packetAge;

  world.ownPose = Pose2f(packet.ownPose[2], packet.ownPose[0], packet.ownPose[1]);
  world.balls.resize(1);
  world.balls[0].position = Vector3f(packet.ball[0], packet.ball[1], packet.ball[2]);
  world.balls[0].velocity = Vector3f(packet.ball[3], packet.ball[4], packet.ball[5]);
  world.ownTeamPlayers.clear();
  world.opponentTeamPlayers.clear();
  for(std::uint32_t i = 0; i < packet.numOfPlayers; ++i)
  {
    const Player& p = packet.players[i];
    GroundTruthWorldState::GroundTruthPlayer player;
    player.number = p.number;
    player.pose = Pose2f(p.pose[2], p.pose[0], p.pose[1]);
    player.upright = p.upright != 0;
    (p.ownTeam ? world.ownTeamPlayers : world.opponentTeamPlayers).push_back(player);
  }

  if(!received)
    spawnPose = world.ownPose;
  received = true;
}
