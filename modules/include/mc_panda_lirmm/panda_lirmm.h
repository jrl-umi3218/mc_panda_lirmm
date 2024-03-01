#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

#include <mc_panda/panda.h>

namespace mc_robots
{

static std::string pandaVariant(bool pump, bool foot, bool hand)
{
  if(pump && !foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_pump'");
    return "pump";
  }
  if(!pump && foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_foot'");
    return "foot";
  }
  if(!pump && !foot && hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_hand'");
    return "hand";
  }
  if(!pump && !foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_default'");
    return "default";
  }
  mc_rtc::log::error("PandaRobotModule does not provide this panda variant...");
  return "";
}

static std::string pandaVariantUppercase(bool pump, bool foot, bool hand)
{
  auto v = pandaVariant(pump, foot, hand);
  if(v.size())
  {
    v[0] = std::toupper(v[0]);
  }
  return v;
}

enum class PandaLIRMMRobots
{
  Panda2LIRMM,
  Panda6LIRMM,
  Panda7LIRMM
};

static std::string to_string(PandaLIRMMRobots robots)
{
  if(robots == PandaLIRMMRobots::Panda2LIRMM)
  {
    return "Panda2LIRMM";
  }
  else if(robots == PandaLIRMMRobots::Panda6LIRMM)
  {
    return "Panda6LIRMM";
  }
  else
  {
    return "Panda7LIRMM";
  }
}

inline static std::string NameFromParams(PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool fixSensorFrame)
{
  auto name = to_string(robot) + pandaVariantUppercase(pump, foot, hand);
  if(!fixSensorFrame)
  {
    name += "Simulation";
  }
  return name;
}

template<typename Callback>
static void ForAllVariants(Callback cb)
{
  auto endEffectorOptions = std::vector<std::array<bool, 3>>{
      {false, false, false}, {true, false, false}, {false, true, false}, {false, false, true}};
  auto fixSensorFrameOptions = {true, false};

  for(auto robot : {PandaLIRMMRobots::Panda2LIRMM, PandaLIRMMRobots::Panda6LIRMM, PandaLIRMMRobots::Panda7LIRMM})
  {
    for(auto endEffector : endEffectorOptions)
    {
      for(auto fixSensorFrame : fixSensorFrameOptions)
      {
        cb(robot, endEffector[0], endEffector[1], endEffector[2], fixSensorFrame);
      }
    }
  }
}

struct ROBOT_MODULE_API PandaLIRMM : public mc_robots::PandaRobotModule
{
public:
  PandaLIRMM(PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool fixSensorFrame = true);

protected:
  void create_urdf();
  void addBox(const std::string & box_name,
              const std::string & parent_link,
              const sva::PTransformd & parentToBox,
              const Eigen::Vector3d & box_size,
              double box_mass);
};

} // namespace mc_robots
