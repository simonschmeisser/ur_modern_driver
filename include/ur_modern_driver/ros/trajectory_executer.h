#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"

struct TrajectoryPoint
{
  std::array<double, 6> positions;
  std::array<double, 6> velocities;
  std::chrono::microseconds time_from_start;

  TrajectoryPoint()
  {
  }

  TrajectoryPoint(std::array<double, 6> &pos, std::array<double, 6> &vel, std::chrono::microseconds tfs)
    : positions(pos), velocities(vel), time_from_start(tfs)
  {
  }
};

class TrajectoryExecuter
{
protected:
  std::atomic<bool> running_;
  URCommander &commander_;

public:
  TrajectoryExecuter(URCommander &commander);
  virtual ~TrajectoryExecuter() {}

  virtual bool start() = 0;
  virtual bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt) = 0;
  virtual void stop() = 0;
};
