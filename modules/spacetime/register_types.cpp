#include "register_types.h"

#include "core/class_db.h"
#include "gravity_body.h"
#include "gravity_integrator.h"

void register_spacetime_types() {
	ClassDB::register_class<GravityIntegrator>();
	ClassDB::register_class<GravityBody>();
}

void unregister_spacetime_types() {}
