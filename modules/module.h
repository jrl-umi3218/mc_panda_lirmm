#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

#include <mc_panda/panda.h>

namespace mc_robots
{

struct ROBOT_MODULE_API PandaLIRMM : public mc_robots::PandaRobotModule
{
public:
  PandaLIRMM(const std::string & name);

protected:
  void addBox(const std::string & box_name,
              const std::string & parent_link,
              const sva::PTransformd & parentToBox,
              const Eigen::Vector3d & box_size,
              double box_mass);
};

struct ROBOT_MODULE_API Panda2LIRMM : public mc_robots::PandaLIRMM
{
public:
  Panda2LIRMM();
};

struct ROBOT_MODULE_API Panda7LIRMM : public mc_robots::PandaLIRMM
{
public:
  Panda7LIRMM();
};

} // namespace mc_robots
