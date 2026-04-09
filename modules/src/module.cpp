#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule_visual.h>

#include <mc_panda/config.h>
#include <mc_panda/panda.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include "config.h"

namespace mc_panda_lirmm
{

/**
 * LIRMM variants without support (table)
 */
enum class PandaLIRMMRobots
{
  Panda2LIRMM, // no table
  Panda7LIRMM, // original attachement position on the table
  Panda6LIRMM, // no support
};

static std::string to_string(PandaLIRMMRobots robots)
{
  using R = PandaLIRMMRobots;
  switch(robots)
  {
    case R::Panda2LIRMM:
      return "Panda2LIRMM";
    case R::Panda7LIRMM:
      return "Panda7LIRMM";
    case R::Panda6LIRMM:
      return "Panda6LIRMM";
    default:
      return "UnknownPandaLIRMM";
  }
}

struct BoxRobotParams
{
  Eigen::Vector3d size;
  mc_rtc::gui::Color color;
  double mass;
  sva::PTransformd box_to_robot = sva::PTransformd::Identity();
};

/**
 * Map of LIRMM robot variants to their associated supports (if any). Each support is defined by its name and
 * parameters.
 */
const std::map<PandaLIRMMRobots, std::map<std::string, BoxRobotParams>> PandaLIRMMRobotSupports = {
    {PandaLIRMMRobots::Panda2LIRMM,
     {{"Table1",
       []
       {
         Eigen::Vector3d size{0.75, 1.0, 0.75};
         return BoxRobotParams{size, mc_rtc::gui::Color::Gray, 10,
                               sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z()})};
       }()},
      {"Table2",
       []
       {
         Eigen::Vector3d size{0.75, 1.0, 0.75};
         Eigen::Vector3d t{-(size.x() / 2 - 0.27), 0, -size.z()};
         t.x() += 0.238;
         t.y() += 0.096;
         return BoxRobotParams{size, mc_rtc::gui::Color::Gray, 10, sva::PTransformd(t)};
       }()}}},
    {PandaLIRMMRobots::Panda7LIRMM,
     {{"Table1",
       []
       {
         Eigen::Vector3d size{1.0, 0.75, 0.75};
         return BoxRobotParams{size, mc_rtc::gui::Color::LightGray, 10,
                               sva::PTransformd(Eigen::Vector3d{size.x() / 2 - 0.19, 0, -size.z()})};
       }()},
      {"Table2",
       []
       {
         Eigen::Vector3d size{1.0, 0.75, 0.75};
         Eigen::Vector3d t{size.x() / 2 - 0.19, 0, -size.z()};
         t.x() -= 0.5725;
         t.y() -= 0.01;
         return BoxRobotParams{size, mc_rtc::gui::Color::LightGray, 10, sva::PTransformd(t)};
       }()}}},
    {PandaLIRMMRobots::Panda6LIRMM,
     {{"MetalSupport", []
       {
         Eigen::Vector3d size{0.25, 0.27, 0.716};
         return BoxRobotParams{size, mc_rtc::gui::Color::Cyan, 40,
                               sva::PTransformd(Eigen::Vector3d{-(size.x() / 4), 0, -size.z()})};
       }()}}}};

inline static std::string ModuleNameFromParams(mc_panda::PandaRobots pandaRobot,
                                               PandaLIRMMRobots lirmmRobot,
                                               mc_panda::Tools tool,
                                               bool calibrated,
                                               const std::string & supportName = "")
{
  auto pandaRm = mc_panda::ModuleNameFromParams(pandaRobot, tool, ""); // ex: FR3_Default
  auto name = to_string(lirmmRobot) + "_" + pandaRm;
  if(!supportName.empty())
  {
    name += "_" + supportName;
  }
  if(calibrated)
  {
    name += "_Calibrated";
  }
  return name;
}

inline static std::string RobotNameFromParams(mc_panda::PandaRobots pandaRobot,
                                              PandaLIRMMRobots lirmmRobot,
                                              mc_panda::Tools tool,
                                              bool calibrated,
                                              const std::string & supportName = "")
{
  return mc_panda::to_lower_case(ModuleNameFromParams(pandaRobot, lirmmRobot, tool, calibrated, supportName));
}

template<typename Callback>
static void ForAllVariants(Callback cb)
{
  using R = mc_panda_lirmm::PandaLIRMMRobots;
  using T = mc_panda::Tools;
  // for(auto endEffector : {T::Default, T::Pump, T::Foot, T::Hand})
  for(auto endEffector : {T::Default, T::Pump, T::Foot, T::Hand, T::Mukca})
  {
    for(mc_panda::PandaRobots pandaRobot : {mc_panda::PandaRobots::FR1, mc_panda::PandaRobots::FR3})
    {
      for(R robot : {R::Panda2LIRMM, R::Panda7LIRMM, R::Panda6LIRMM})
      {
        auto it = PandaLIRMMRobotSupports.find(robot);
        if(it != PandaLIRMMRobotSupports.end() && !it->second.empty())
        {
          for(const auto & support : it->second)
          {
            cb(pandaRobot, robot, endEffector, false, support.first); // non-calibrated, with support
            // Calibrated variants (example: only for Panda2LIRMM)
            cb(mc_panda::PandaRobots::FR1, robot, endEffector, true, support.first);
          }
        }
        else
        {
          cb(pandaRobot, robot, endEffector, false, ""); // non-calibrated, no support
          // Calibrated variants (example: only for Panda2LIRMM)
          cb(mc_panda::PandaRobots::FR1, robot, endEffector, true, "");
        }
      }
    }
  }
}

std::shared_ptr<mc_rbdyn::RobotModule> makeBoxRobotModule(const BoxRobotParams & params)
{
  const auto & size = params.size;
  const auto & color = params.color;
  double mass = params.mass;

  auto boxYaml = fmt::format(R"(
name: robot_support
origin:
  translation: [0, 0, {0}]
  rotation: [0, 0, 0]
material:
  color:
    r: {5}
    g: {6}
    b: {7}
    a: {8}
geometry:
  box:
    size: [{1}, {2}, {3}]
inertia:
  mass: {4}
fixed: true
)",
                             size.z() / 2, size.x(), size.y(), size.z(), mass, color.r, color.g, color.b, color.a);

  auto boxConfig = mc_rtc::Configuration::fromYAMLData(boxYaml);
  return mc_rbdyn::robotModuleFromVisual("robot_support", boxConfig);
}

} // namespace mc_panda_lirmm

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    using namespace mc_panda_lirmm;
    ForAllVariants(
        [&names](mc_panda::PandaRobots pandaRobot, PandaLIRMMRobots lirmmRobot, mc_panda::Tools tool, bool calibrated,
                 const std::string & supportName)
        {
          names.push_back(mc_panda_lirmm::ModuleNameFromParams(pandaRobot, lirmmRobot, tool, calibrated, supportName));
        });
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaLIRMM")

    using namespace mc_panda_lirmm;
    using R = PandaLIRMMRobots;

    bool found = false;
    mc_rbdyn::RobotModule * result = nullptr;

    ForAllVariants(
        [&](mc_panda::PandaRobots pandaRobot, PandaLIRMMRobots lirmmRobot, mc_panda::Tools tool, bool calibrated,
            const std::string & supportName)
        {
          if(found) return;

          std::string lirmm_variant_name =
              mc_panda_lirmm::ModuleNameFromParams(pandaRobot, lirmmRobot, tool, calibrated, supportName);
          if(lirmm_variant_name != n) return;

          std::string panda_variant_name = mc_panda::ModuleNameFromParams(pandaRobot, tool);
          std::string robot_name =
              mc_panda_lirmm::RobotNameFromParams(pandaRobot, lirmmRobot, tool, calibrated, supportName);

          mc_rbdyn::RobotModule * pandaRm = nullptr;
          if(calibrated)
          {
            std::string fr_name = mc_panda::to_lower_case(mc_panda::to_string(pandaRobot));
            std::string lirmmRobotName = mc_panda::to_lower_case(to_string(lirmmRobot));
            auto calibrated_urdf_path =
                fs::path{mc_panda_lirmm::CALIBRATED_PANDA_LIRMM_URDF_PATH} / lirmmRobotName / fr_name;
            auto calibrated_urdf_path_str = calibrated_urdf_path.string();
            if(fs::exists(calibrated_urdf_path / "urdf/panda_default.urdf"))
            {
              mc_rtc::log::info("Using calibrated urdf for {}: {}", lirmm_variant_name, calibrated_urdf_path_str);
            }
            else
            {
              mc_rtc::log::error_and_throw("Calibrated urdf not found for {}: {}", lirmm_variant_name,
                                           calibrated_urdf_path_str);
            }
            auto calibPathsConfig =
                pandaRobot == mc_panda::PandaRobots::FR1 ? mc_panda::FR1DefaultPaths : mc_panda::FR3DefaultPaths;
            calibPathsConfig.urdf_base_path =
                calibrated_urdf_path; // override urdf base path with the calibrated one (fr1 only for now)
            pandaRm = mc_panda::create(panda_variant_name, calibPathsConfig);
          }
          else
          {
            pandaRm = mc_panda::create(panda_variant_name);
          }

          if(!supportName.empty())
          {
            // Get support parameters from map
            const auto & params = PandaLIRMMRobotSupports.at(lirmmRobot).at(supportName);
            auto rmV = makeBoxRobotModule(params);
            auto base = (pandaRobot == mc_panda::PandaRobots::FR3) ? "base" : "world";

            result = new mc_rbdyn::RobotModule(rmV->connect(
                *pandaRm, "robot_support", base, "",
                mc_rbdyn::RobotModule::ConnectionParameters{}.name(robot_name).X_other_connection(params.box_to_robot)));
          }
          else
          {
            result = pandaRm;
          }
          found = true;
        });

    if(result)
    {
      return result;
    }
    else
    {
      mc_rtc::log::error("PandaLIRMM module Cannot create an object of type {}", n);
      std::vector<std::string> available;
      ForAllVariants(
          [&](mc_panda::PandaRobots pandaRobot, PandaLIRMMRobots lirmmRobot, mc_panda::Tools tool, bool calibrated,
              const std::string & supportName)
          {
            std::string name = mc_panda_lirmm::ModuleNameFromParams(pandaRobot, lirmmRobot, tool, calibrated);
            if(!supportName.empty())
            {
              name += "_" + supportName;
            }
            available.push_back(name);
          });
      mc_rtc::log::info("Available variants are: {}", fmt::join(available, ", "));
      return nullptr;
    }
  }
}
