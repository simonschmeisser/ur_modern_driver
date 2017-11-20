#include "ur_modern_driver/ros/trajectory_downloader.h"
#include <endian.h>
#include <cmath>
#include <ros/ros.h>

TrajectoryDownloader::TrajectoryDownloader(URCommander &commander, std::string &reverse_ip, int reverse_port,
                                       bool version_3)
  : TrajectoryExecuter (commander)
  , server_(reverse_port)
  , servoj_time_(0.008)
  , servoj_lookahead_time_(0.03)
  , servoj_gain_(300.)
{
}

bool TrajectoryDownloader::start()
{
  if (running_)
    return true;  // not sure

  LOG_DEBUG("Robot successfully connected");
  return (running_ = true);
}

bool TrajectoryDownloader::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
  if (!running_)
    return false;

  using namespace std::chrono;

  auto &last = trajectory.back();
  auto &prev = trajectory.front();


  std::array<double, 6> positions;

  std::string program;
  std::stringstream stream;

  stream << "(\n";
  stream << "def mikadoMove():";
  for (auto const &point : trajectory)
  {
    // skip t0
    if (&point == &prev)
      continue;

    stream << "movej(q=[";
    for (size_t i = 0; i < 5; ++i)
          stream << point.positions[i] << ",";
    stream << point.positions[5] << "], r=0.001)\n";
    stream << "sync()\n";
  }
  stream << "end\n";
  stream << ")\n";

  std::cout << "Final program is:" << std::endl;
  program = stream.str();
  std::cout << program;

  LOG_INFO("Uploading trajectory program to robot");

  if (!commander_.uploadProg(program))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }
  std::chrono::microseconds time_estimate = trajectory.back().time_from_start * 10;
  std::this_thread::sleep_for(time_estimate);

  return true;
}

void TrajectoryDownloader::stop()
{
  if (!running_)
    return;

  running_ = false;
}
