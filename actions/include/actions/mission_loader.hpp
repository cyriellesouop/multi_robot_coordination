// Loading a DWAFP Mission from YAML (Phase C1).
//
// Two inputs, so the fleet has a single source of truth:
//   - the FLEET file is the same one the mock fleet launch uses (config/mock_fleet.yaml):
//       robots:
//         - name: robot1
//           step_duration_sec: 1.0   # nominal seconds per unit of work (default 1.0)
//     Other fields (simulated_delay_sec, delay_at_step) belong to the mock robot and are ignored.
//   - the MISSION file (config/missions/*.yaml):
//       mission_id: chain
//       subtasks:                    # mission order = queue order on each robot
//         - id: T1
//           robot: robot1
//           work: 5                  # optional, default 5 (= the mock robot's 5 steps)
//           deadline_sec: 6.0        # nominal seconds from mission start, required
//           predecessors: []         # optional, default none
//           retryable: true          # optional, default true
//           target_pose: {frame_id: map, x: 1.0, y: 0.0, yaw: 0.0}   # optional, opaque
//
// Every error throws std::invalid_argument with the file and the subtask concerned.
// yaml-cpp is only used in the .cpp, so including this header does not pull it in.

#ifndef ACTIONS__MISSION_LOADER_HPP_
#define ACTIONS__MISSION_LOADER_HPP_

#include <string>

#include "actions/workflow.hpp"

namespace dwafp
{

// Builds a Mission from the TEXT of a mission YAML and a fleet YAML (used by the tests).
// `mission_name` / `fleet_name` only appear in error messages.
Mission mission_from_yaml(
  const std::string & mission_yaml, const std::string & fleet_yaml,
  const std::string & mission_name = "mission", const std::string & fleet_name = "fleet");

// Builds a Mission from a mission file and a fleet file on disk.
Mission load_mission(const std::string & mission_path, const std::string & fleet_path);

}  // namespace dwafp

#endif  // ACTIONS__MISSION_LOADER_HPP_
