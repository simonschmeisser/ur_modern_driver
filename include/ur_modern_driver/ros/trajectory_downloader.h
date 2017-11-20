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
#include "ur_modern_driver/ros/trajectory_executer.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"


class TrajectoryDownloader : public TrajectoryExecuter
{
private:
  std::array<double, 6> last_positions_;
  URServer server_;

  double servoj_time_, servoj_lookahead_time_, servoj_gain_;

public:
  TrajectoryDownloader(URCommander &commander, std::string &reverse_ip, int reverse_port, bool version_3);

  bool start() override;
  bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt) override;
  void stop() override;
};
