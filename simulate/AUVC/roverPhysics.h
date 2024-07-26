#ifndef PHYSICS_H
#define PHYSICS_H

#include <mujoco/mujoco.h>

typedef void (*plug_test_t)(void);
typedef int (*plug_init_t)(mjModel *m, mjData *d);
typedef int (*plug_update_t)(mjModel *m, mjData *d);

extern "C" void roverPhysicsTestPlug(void);
extern "C" int roverPhysicsInitPlug(mjModel *m, mjData *d);
extern "C" bool roverPhysicsUpdatePlug(mjModel *m, mjData *d);

#endif // PHYSICS_H
