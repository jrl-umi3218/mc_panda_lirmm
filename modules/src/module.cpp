#include <mc_panda_lirmm/panda_lirmm.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    using namespace mc_panda_lirmm;
    ForAllVariants([&names](PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool fixSensorFrame)
                   { names.push_back(NameFromParams(robot, pump, foot, hand, fixSensorFrame)); });
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaLIRMM")

    static auto variant_factory = []()
    {
      std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> variant_factory;
      using namespace mc_panda_lirmm;
      ForAllVariants(
          [&variant_factory](PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool fixSensorFrame)
          {
            variant_factory[NameFromParams(robot, pump, foot, hand, fixSensorFrame)] = [=]()
            { return new PandaLIRMM(robot, pump, foot, hand, fixSensorFrame); };
          });
      return variant_factory;
    }();
    auto it = variant_factory.find(n);
    if(it != variant_factory.end())
    {
      return it->second();
    }
    else
    {
      mc_rtc::log::error("PandaLIRMM module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
