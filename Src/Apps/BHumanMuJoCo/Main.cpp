/**
 * @file Main.cpp
 *
 * Runs one B-Human robot on a PC against a K1 simulated in an external MuJoCo
 * simulation. The joints are connected through the Booster SDK (DDS topics
 * <prefix>low_state and <prefix>joint_ctrl, see BoosterProvider). Instead of
 * camera images, the ground truth state of the world is received via UDP (see
 * MuJoCoWorldProvider). The GameController packets are expected on port
 * 3838 + 10000 + player number (see GameControllerDataProvider).
 * This is built with Make/Linux/compileMuJoCo, which defines TARGET_MUJOCO.
 */

#include <booster/robot/channel/channel_factory.hpp>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

#include "Framework/Robot.h"
#include "Framework/Settings.h"
#include "Math/Constants.h"
#include "Platform/SystemCall.h"
#include "Streaming/FunctionList.h"
#include "Streaming/TypeRegistry.h"

// Used by BoosterProvider
namespace Main
{
  std::string topicPrefix = "rt/";
}

static bool run = true;

SystemCall::Mode SystemCall::getMode()
{
  // Behave like on the robot, i.e. threads are paced by their data and not by SimRobot.
  return physicalRobot;
}

static void sigHandler(int)
{
  run = false;
}

int main(int argc, char* argv[])
{
  setvbuf(stdout, nullptr, _IONBF, 0);
  setvbuf(stderr, nullptr, _IONBF, 0);

  const char* bhDir = nullptr;
  std::string robotName = "MuJoCo";
  std::string location = "Default";
  std::string scenario = "MuJoCo";
  std::string networkInterface = "lo";
  int teamNumber = 5;
  int playerNumber = 0;
  for(int i = 1; i < argc; ++i)
    if(!strcmp(argv[i], "-c") && i + 1 < argc)
      bhDir = argv[++i];
    else if(!strcmp(argv[i], "--player") && i + 1 < argc)
      playerNumber = std::atoi(argv[++i]);
    else if(!strcmp(argv[i], "--team") && i + 1 < argc)
      teamNumber = std::atoi(argv[++i]);
    else if(!strcmp(argv[i], "--topic-prefix") && i + 1 < argc)
      Main::topicPrefix = argv[++i];
    else if(!strcmp(argv[i], "--location") && i + 1 < argc)
      location = argv[++i];
    else if(!strcmp(argv[i], "--scenario") && i + 1 < argc)
      scenario = argv[++i];
    else if(!strcmp(argv[i], "--robot-name") && i + 1 < argc)
      robotName = argv[++i];
    else if(!strcmp(argv[i], "--interface") && i + 1 < argc)
      networkInterface = argv[++i];
    else
      playerNumber = 0;
  if(playerNumber < 1)
  {
    fprintf(stderr, "Usage: %s --player <n> [--team <n>] [--topic-prefix <rt/robot_6/>] [--location <name>]\n"
            "          [--scenario <name>] [--robot-name <name>] [--interface <lo>] [-c <bh dir>]\n", argv[0]);
    return EXIT_FAILURE;
  }
  if(bhDir && chdir(bhDir) != 0)
  {
    fprintf(stderr, "chdir to %s failed!\n", bhDir);
    return EXIT_FAILURE;
  }

  // Acquire static data, e.g. about types
  FunctionList::execute();

  // Several robots run on the same PC.
  SystemCall::mute(true);

  booster::robot::ChannelFactory::Instance()->Init(0, networkInterface);

  Settings settings(robotName, Constants::motionCycleTime, Settings::k1, teamNumber,
                    Settings::blue, Settings::red, playerNumber, location, scenario, 89);
  printf("Hi, I am %s, player %d of team %d (topics %s*, location %s, scenario %s).\n",
         robotName.c_str(), playerNumber, teamNumber, Main::topicPrefix.c_str(), location.c_str(), scenario.c_str());

  signal(SIGTERM, sigHandler);
  signal(SIGINT, sigHandler);

  Robot robot(settings, std::string());
  robot.start();
  while(run)
    pause();
  robot.announceStop();
  robot.stop();
  return EXIT_SUCCESS;
}
