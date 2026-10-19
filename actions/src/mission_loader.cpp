// Loading a DWAFP Mission from YAML (Phase C1). Format: see include/actions/mission_loader.hpp.
//
// What this file does, in order:
//   1. read the two files into strings (read_file),
//   2. parse the text into a YAML tree (parse_text, using the yaml-cpp library),
//   3. copy the fields of the tree into plain C++ structs (parse_fleet, parse_subtasks),
//      converting units on the way (seconds -> milliseconds, step duration -> ms per work unit),
//   4. hand the structs to the Mission constructor, which does the semantic checks
//      (unknown robot or predecessor, cycle, ...).
// Only steps 1-3 live here. Keeping yaml-cpp in this file means the core (workflow.cpp) does not
// depend on any file format.

#include "actions/mission_loader.hpp"

#include <cmath>      // std::isfinite, std::llround
#include <fstream>    // std::ifstream (read a file)
#include <sstream>    // std::ostringstream (collect the file into a string)
#include <stdexcept>  // std::invalid_argument
#include <utility>    // std::move
#include <vector>

#include <yaml-cpp/yaml.h>  // YAML::Node (a YAML tree), YAML::Load (text -> tree)

namespace dwafp
{

namespace
{

// A YAML::Node is one element of the tree: a map ({id: T1, robot: robot1}), a list ([T1, T2])
// or a scalar (5.0). node["key"] returns the child, or an empty ("false") node if it is absent.
//
// optional_field reads one scalar field and converts it to type T, or returns `fallback` when
// the field is absent. "template<typename T>" means the same function works for any type:
// optional_field<double>(...), optional_field<bool>(...), optional_field<std::string>(...).
// A field that is present but of the wrong type (e.g. "work: fast") is an error, not a default.
// `where` names the file and subtask, so the error message says exactly where to look.
template<typename T>
T optional_field(const YAML::Node & node, const char * key, const T & fallback, const std::string & where)
{
  const YAML::Node value = node[key];
  if (!value) {return fallback;}  // absent -> default value
  try {
    return value.as<T>();  // yaml-cpp converts the text ("5", "true") into T, or throws
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(where + ": field '" + key + "' has the wrong type");
  }
}

// Same as optional_field, but the field must be present. Once presence is checked, the fallback
// T{} (an empty / zero value) is never used.
template<typename T>
T required_field(const YAML::Node & node, const char * key, const std::string & where)
{
  if (!node[key]) {throw std::invalid_argument(where + ": missing field '" + key + "'");}
  return optional_field<T>(node, key, T{}, where);
}

// Nominal seconds (YAML) -> integer nominal milliseconds (library), rounded to the nearest ms.
// Example: deadline_sec: 6.0 -> 6000. isfinite rejects "inf" and "nan", which YAML accepts.
TimeMs seconds_to_ms(double seconds, const std::string & where, const char * key)
{
  if (!std::isfinite(seconds)) {
    throw std::invalid_argument(where + ": field '" + std::string(key) + "' must be a finite number");
  }
  return static_cast<TimeMs>(std::llround(seconds * 1000.0));
}

// mock_fleet.yaml -> list of Robot. Example: {name: robot2, step_duration_sec: 1.5} becomes
// Robot{"robot2", 1500.0}: robot2 needs 1500 nominal ms per unit of work.
std::vector<Robot> parse_fleet(const YAML::Node & root, const std::string & fleet_name)
{
  const YAML::Node robots = root["robots"];
  if (!robots || !robots.IsSequence() || robots.size() == 0) {
    throw std::invalid_argument(fleet_name + ": 'robots' must be a non-empty list");
  }
  std::vector<Robot> fleet;
  for (std::size_t i = 0; i < robots.size(); ++i) {
    const std::string where = fleet_name + ": robot #" + std::to_string(i);
    Robot robot;
    robot.name = required_field<std::string>(robots[i], "name", where);
    // Same default as the mock robot's step_duration_sec parameter, so both agree on p_jr.
    const double step_sec = optional_field<double>(robots[i], "step_duration_sec", 1.0, where);
    robot.ms_per_work_unit = step_sec * 1000.0;  // Mission checks that it is > 0
    fleet.push_back(std::move(robot));  // move: hand the struct over instead of copying it
  }
  return fleet;
}

// Mission file -> list of Subtask, in file order (the file order is also the queue order).
// Example (fork_join.yaml, third entry):
//   {id: T3, robot: robot3, work: 5, deadline_sec: 14.0, predecessors: [T1, T2], target_pose: {...}}
//   -> Subtask{"T3", "robot3", 5.0, 14000, {"T1", "T2"}, true (default), pose}
// Names are kept as strings here; the Mission constructor turns them into indices.
std::vector<Subtask> parse_subtasks(const YAML::Node & root, const std::string & mission_name)
{
  const YAML::Node subtasks = root["subtasks"];
  if (!subtasks || !subtasks.IsSequence() || subtasks.size() == 0) {
    throw std::invalid_argument(mission_name + ": 'subtasks' must be a non-empty list");
  }
  std::vector<Subtask> tasks;
  for (std::size_t i = 0; i < subtasks.size(); ++i) {
    const YAML::Node node = subtasks[i];
    Subtask task;
    // The id is read first so the following error messages can name the subtask.
    task.id = required_field<std::string>(node, "id", mission_name + ": subtask #" + std::to_string(i));
    const std::string where = mission_name + ": subtask '" + task.id + "'";
    task.robot = required_field<std::string>(node, "robot", where);
    task.work = optional_field<double>(node, "work", 5.0, where);  // 5 = the mock robot's 5 steps
    task.deadline_ms = seconds_to_ms(required_field<double>(node, "deadline_sec", where), where, "deadline_sec");
    task.predecessors = optional_field<std::vector<std::string>>(node, "predecessors", {}, where);
    task.retryable = optional_field<bool>(node, "retryable", true, where);
    // The target pose is opaque to DWAFP: it is only stored and handed to the robot later.
    // "if (const YAML::Node pose = ...)" declares pose and enters the block only if it exists.
    if (const YAML::Node pose = node["target_pose"]) {
      task.target.frame_id = optional_field<std::string>(pose, "frame_id", "map", where);
      task.target.x = optional_field<double>(pose, "x", 0.0, where);
      task.target.y = optional_field<double>(pose, "y", 0.0, where);
      task.target.yaw = optional_field<double>(pose, "yaw", 0.0, where);
    }
    tasks.push_back(std::move(task));
  }
  return tasks;
}

// yaml-cpp reports syntax errors with its own exception type; turn them into the one error type
// promised by the header, keeping the file name in the message.
YAML::Node parse_text(const std::string & text, const std::string & name)
{
  try {
    return YAML::Load(text);
  } catch (const YAML::Exception & e) {
    throw std::invalid_argument(name + ": invalid YAML: " + e.what());
  }
}

// Whole file -> one string. rdbuf() streams the file's content into the string stream.
std::string read_file(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {throw std::invalid_argument("cannot open '" + path + "'");}
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

}  // namespace

// Text version: the tests pass small YAML strings directly, without writing files.
Mission mission_from_yaml(
  const std::string & mission_yaml, const std::string & fleet_yaml,
  const std::string & mission_name, const std::string & fleet_name)
{
  const YAML::Node mission_root = parse_text(mission_yaml, mission_name);
  const YAML::Node fleet_root = parse_text(fleet_yaml, fleet_name);
  const std::string mission_id = required_field<std::string>(mission_root, "mission_id", mission_name);
  // Parsing errors already name their file; parse outside the try so they are not prefixed twice.
  std::vector<Subtask> subtasks = parse_subtasks(mission_root, mission_name);
  std::vector<Robot> robots = parse_fleet(fleet_root, fleet_name);
  // The Mission constructor does the semantic checks (unknown robot / predecessor, cycle, ...);
  // its messages do not know the file, so the mission file name is added here.
  try {
    return Mission(mission_id, std::move(subtasks), std::move(robots));
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument(mission_name + ": " + e.what());
  }
}

// File version, used by the simulator and the tests: read both files, then the text version.
// The file paths are used as names, so an error reads e.g.
//   ".../missions/fork_join.yaml: subtask 'T3' depends on unknown subtask 'T4'".
Mission load_mission(const std::string & mission_path, const std::string & fleet_path)
{
  return mission_from_yaml(read_file(mission_path), read_file(fleet_path), mission_path, fleet_path);
}

}  // namespace dwafp
