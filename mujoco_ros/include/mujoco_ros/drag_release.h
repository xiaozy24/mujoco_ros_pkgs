#pragma once
#include <mujoco/mujoco.h>

namespace mujoco_ros {
// Caller must hold the physics mutex. Only stop a free rigid body; never
// overwrite articulated robot velocities or change the selected body's pose.
inline bool StopReleasedBody(const mjModel* model, mjData* data, int selected_body) {
  if (selected_body <= 0 || selected_body >= model->nbody) return false;
  int body = selected_body;
  // A picked geom may belong to a rigid child of the free body.
  while (body > 0 && model->body_jntnum[body] == 0) body = model->body_parentid[body];
  if (body <= 0 || model->body_jntnum[body] != 1) return false;
  const int joint = model->body_jntadr[body];
  if (joint < 0 || model->jnt_type[joint] != mjJNT_FREE) return false;
  const int dof = model->jnt_dofadr[joint];
  mju_zero(data->qvel + dof, 6);
  mju_zero(data->qacc_warmstart + dof, 6);
  mju_zero(data->xfrc_applied + 6 * selected_body, 6);
  return true;
}
}  // namespace mujoco_ros
