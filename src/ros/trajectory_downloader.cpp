#include "ur_modern_driver/ros/trajectory_downloader.h"
#include <chrono>
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
    double deceleration_factor = 2.0;
    ros::param::param<double>("/deceleration_factor", deceleration_factor, deceleration_factor);
    double stopj_deceleration = 1.0;
    ros::param::param<double>("/stopj_deceleration", stopj_deceleration, stopj_deceleration);
    
    typedef std::chrono::duration<double> double_seconds;
    using namespace std::chrono;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

  if (!running_)
    return false;

  using namespace std::chrono;

  auto &first = trajectory.front();
  auto &last = trajectory.back();

  std::string program;

  bool useServoJ = trajectory.size() > 2;

  if (useServoJ) {

      std::stringstream stream;
      stream << "(\n";
      stream << "def mikadoMove():\n";
      stream << "  goals = [";
      for (auto const &point : trajectory)
      {
        // skip t0
        if (&point == &first) {
            continue;
        }

        for (size_t i = 0; i < 5; ++i)
              stream << point.positions[i] << ",";
        stream << point.positions[5];
        if (&point != &last)
            stream << ","; //],
        else
            stream << ""; //]
      }
      stream << "]\n";

      stream << "  dur = [";
      double t = 0;
      for (auto const &point : trajectory)
      {

        // skip t0
        if (&point == &first) {
            continue;
        }
        double dt = std::chrono::duration_cast<double_seconds>(point.time_from_start).count() - t;

        if (&point != &last)
            stream << dt << ",";
        else
            stream << deceleration_factor*dt << "";
        t = std::chrono::duration_cast<double_seconds>(point.time_from_start).count();
      }
      stream << "]\n";

      stream << "  sync()\n"
             << "  l = " << trajectory.size()-1 << "\n"  //did not include trajectory start
             << "  i = 0\n"
             << "  while i < l:\n"
             << "    o = i*6\n"
             << "    goal = [goals[o], goals[o+1], goals[o+2], goals[o+3], goals[o+4], goals[o+5]]\n"
             << "    servoj(goal, t=dur[i])\n"
             << "    i = i +1\n"
             << "  end\n"
             << "  stopj(" << stopj_deceleration << ")\n"
             << "end\n"
             << ")\n";

      program = stream.str();
  } else {
      std::stringstream stream;
      stream << "(\n";
      stream << "def mikadoMove():\n";
      for (auto const &point : trajectory)
      {
        // skip t0
        if (&point == &first)
          continue;

        stream << "movej(q=[";
        for (size_t i = 0; i < 5; ++i)
              stream << point.positions[i] << ",";
        stream << point.positions[5];
        if (&point != &last)
            stream << "], r=0.005)\n";
        else
            stream << "])\n";
      }
      stream << "end\n";
      stream << ")\n";

      program = stream.str();
  }

  std::cout << "Final program is:" << std::endl;
  std::cout << program;

  LOG_INFO("Waiting for previous/other program to finish");
  {
      std::unique_lock<std::mutex> lck(mtx_);
      if (cnd.wait_for(lck, std::chrono::seconds(30), [this]{return !program_running_;}))
      {
      } else {
        LOG_ERROR("TIMEOUT waiting for previouse/other program to finish");
        return false;
      }
  }

  LOG_INFO("Uploading trajectory program to robot");

  if (!commander_.uploadProg(program))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  std::cout << "Creating and uploading programm took "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                << " ms.\n";

  //allow some time for status to change
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  LOG_INFO("Waiting for trajectory program to finish");
  {
    std::unique_lock<std::mutex> lck(mtx_);
    if (cnd.wait_for(lck, std::chrono::seconds(30), [this]{return !program_running_;}))
    {
      LOG_INFO("Program finished");
    } else {
      LOG_ERROR("TIMEOUT waiting for program to finish");
      return false;
    }
  }

  return true;
}

void TrajectoryDownloader::stop()
{
  if (!running_)
    return;

  running_ = false;
}

void TrajectoryDownloader::update_program_running(bool program_running)
{
    LOG_INFO("Programm running changed to %i", program_running);
    std::unique_lock<std::mutex> lck(mtx_);
    program_running_ = program_running;
    cnd.notify_one();
}
