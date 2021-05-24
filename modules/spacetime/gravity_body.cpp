#include "gravity_body.h"

#include "core/engine.h"

void GravityBody::_bind_methods() {
	ClassDB::bind_method("_integrate_forces", &GravityBody::_integrate_forces);

	ClassDB::bind_method(D_METHOD("set_x", "value"), &GravityBody::set_x);
	ClassDB::bind_method(D_METHOD("get_x"), &GravityBody::get_x);
	ClassDB::bind_method(D_METHOD("set_y", "value"), &GravityBody::set_y);
	ClassDB::bind_method(D_METHOD("get_y"), &GravityBody::get_y);
	ClassDB::bind_method(D_METHOD("set_vel_x", "value"), &GravityBody::set_vel_x);
	ClassDB::bind_method(D_METHOD("get_vel_x"), &GravityBody::get_vel_x);
	ClassDB::bind_method(D_METHOD("set_vel_y", "value"), &GravityBody::set_vel_y);
	ClassDB::bind_method(D_METHOD("get_vel_y"), &GravityBody::get_vel_y);
	ClassDB::bind_method(D_METHOD("set_gbody_mass", "value"), &GravityBody::set_gbody_mass);
	ClassDB::bind_method(D_METHOD("get_gbody_mass"), &GravityBody::get_gbody_mass);
	ClassDB::bind_method(D_METHOD("set_significance", "value"), &GravityBody::set_significance);
	ClassDB::bind_method(D_METHOD("get_significance"), &GravityBody::get_significance);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "x"), "set_x", "get_x");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "y"), "set_y", "get_y");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "vel_x"), "set_vel_x", "get_vel_x");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "vel_y"), "set_vel_y", "get_vel_y");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "gbody_mass"), "set_gbody_mass", "get_gbody_mass");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_significant"), "set_significance", "get_significance");
}

GravityBody::GravityBody() {
	x = 0.0;
	y = 0.0;
	vel_x = 0.0;
	vel_y = 0.0;
	previous_timescale = 1.0;
	timescale = 1.0;
	last_accel_x = 0.0;
	last_accel_y = 0.0;
	gbody_mass = 1.0;
	is_significant = false;
}
GravityBody::~GravityBody() {}

void GravityBody::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY:
			if (Engine::get_singleton()->is_editor_hint()) {
				break;
			}
			set_significance(is_significant);
			set_physics_process_internal(true);
			set_use_custom_integrator(true);
			if (is_significant) {
				set_mode(RigidBody2D::MODE_KINEMATIC);
			} else {
				set_linear_velocity(Vector2(vel_x, vel_y));
				set_mode(RigidBody2D::MODE_RIGID);
			}
			if (x == 0 && y == 0) {
				x = get_position().x;
				y = get_position().y;
			}
			internal_mass = gbody_mass * 1000000;
			break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			float delta = get_process_delta_time();
			float128 warped_delta = delta * timescale;
			if (get_mode() == RigidBody2D::MODE_KINEMATIC) {
				vel_x += last_accel_x * warped_delta;
				vel_y += last_accel_y * warped_delta;
				x += vel_x * warped_delta;
				y += vel_y * warped_delta;
				set_position(Vector2(x, y));
				set_rotation(get_angular_velocity() * warped_delta);
			} else {
				/* apply_central_impulse( */
				/*     Vector2(last_accel_x * last_delta, last_accel_y * last_delta));
         */
				set_linear_velocity(get_linear_velocity() * timescale /
											previous_timescale +
									Vector2(last_accel_x * warped_delta * timescale,
											last_accel_y * warped_delta * timescale));
			}
			last_accel_x = 0.0;
			last_accel_y = 0.0;
		} break;
	}
}

void GravityBody::_integrate_forces(Variant variant) {
	x = get_position().x;
	y = get_position().y;
}

void GravityBody::apply_accel(float128 accel_x, float128 accel_y) {
	last_accel_x += accel_x;
	last_accel_y += accel_y;
}

void GravityBody::set_x(real_t value) {
	x = value;
}
real_t GravityBody::get_x() const {
	return x;
}

void GravityBody::set_y(real_t value) {
	y = value;
}
real_t GravityBody::get_y() const {
	return y;
}

void GravityBody::set_vel_x(real_t value) {
	vel_x = value;
}
real_t GravityBody::get_vel_x() const {
	return vel_x;
}

void GravityBody::set_vel_y(real_t value) {
	vel_y = value;
}
real_t GravityBody::get_vel_y() const {
	return vel_y;
}

void GravityBody::set_timescale(real_t value) {
	previous_timescale = timescale;
	timescale = value;
}

void GravityBody::set_gbody_mass(real_t value) {
	gbody_mass = value;
}
real_t GravityBody::get_gbody_mass() const {
	return gbody_mass;
}

void GravityBody::set_significance(bool value) {
	is_significant = value;
	if (value) {
		if (is_in_group("insignificant_bodies")) {
			remove_from_group("insignificant_bodies");
		}
		add_to_group("significant_bodies");
	} else {
		if (is_in_group("significant_bodies")) {
			remove_from_group("significant_bodies");
		}
		add_to_group("insignificant_bodies");
	}
}
bool GravityBody::get_significance() const {
	return is_significant;
}
