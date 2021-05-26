#include "gravity_body.h"

#include "core/engine.h"

void GravityBody::_bind_methods() {
	ClassDB::bind_method("_sync_pos", &GravityBody::_sync_pos);

	ClassDB::bind_method(D_METHOD("set_precise_x", "value"), &GravityBody::set_precise_x);
	ClassDB::bind_method(D_METHOD("get_precise_x"), &GravityBody::get_precise_x);
	ClassDB::bind_method(D_METHOD("set_precise_y", "value"), &GravityBody::set_precise_y);
	ClassDB::bind_method(D_METHOD("get_precise_y"), &GravityBody::get_precise_y);
	ClassDB::bind_method(D_METHOD("set_precise_rotation", "value"), &GravityBody::set_precise_rotation);
	ClassDB::bind_method(D_METHOD("get_precise_rotation"), &GravityBody::get_precise_rotation);
	ClassDB::bind_method(D_METHOD("set_linear_velocity_x", "value"), &GravityBody::set_linear_velocity_x);
	ClassDB::bind_method(D_METHOD("get_linear_velocity_x"), &GravityBody::get_linear_velocity_x);
	ClassDB::bind_method(D_METHOD("set_linear_velocity_y", "value"), &GravityBody::set_linear_velocity_y);
	ClassDB::bind_method(D_METHOD("get_linear_velocity_y"), &GravityBody::get_linear_velocity_y);
	ClassDB::bind_method(D_METHOD("set_angular_velocity", "value"), &GravityBody::set_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &GravityBody::get_angular_velocity);
	ClassDB::bind_method(D_METHOD("set_mass", "value"), &GravityBody::set_mass);
	ClassDB::bind_method(D_METHOD("get_mass"), &GravityBody::get_mass);
	ClassDB::bind_method(D_METHOD("set_is_rigid", "value"), &GravityBody::set_is_rigid);
	ClassDB::bind_method(D_METHOD("get_is_rigid"), &GravityBody::get_is_rigid);
	ClassDB::bind_method(D_METHOD("set_significance", "value"), &GravityBody::set_significance);
	ClassDB::bind_method(D_METHOD("get_significance"), &GravityBody::get_significance);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "precise_x"), "set_precise_x", "get_precise_x");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "precise_y"), "set_precise_y", "get_precise_y");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "precise_rotation"), "set_precise_rotation", "get_precise_rotation");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "linear_velocity_x"), "set_linear_velocity_x", "get_linear_velocity_x");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "linear_velocity_y"), "set_linear_velocity_y", "get_linear_velocity_y");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "angular_velocity"), "set_angular_velocity", "get_angular_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "mass"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_rigid"), "set_is_rigid", "get_is_rigid");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_significant"), "set_significance", "get_significance");
}

GravityBody::GravityBody() :
		PhysicsBody2D(Physics2DServer::BodyMode::BODY_MODE_KINEMATIC) {
	precise_x = 0.0;
	precise_y = 0.0;
	precise_rotation = 0.0;
	linear_vel_x = 0.0;
	linear_vel_y = 0.0;
	angular_vel = 0.0;
	mass = 1.0;
	timescale = 1.0;
	last_accel_x = 0.0;
	last_accel_y = 0.0;
	is_rigid = false;
	is_significant = false;
}
GravityBody::~GravityBody() {}

void GravityBody::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if (Engine::get_singleton()->is_editor_hint()) {
				break;
			}
			set_significance(is_significant);
			set_is_rigid(is_rigid);
			set_physics_process_internal(true);
			RID rid = get_rid();
			Physics2DServer *server = Physics2DServer::get_singleton();
			server->body_set_omit_force_integration(rid, true);
			server->body_set_param(rid, Physics2DServer::BODY_PARAM_MASS, mass);
			server->body_set_force_integration_callback(rid, this, "_sync_pos");
			if (precise_x == 0 && precise_y == 0) {
				precise_x = get_position().x;
				precise_y = get_position().y;
			}
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			float delta = get_physics_process_delta_time();
			float128 warped_delta = delta * timescale;

			linear_vel_x += last_accel_x * warped_delta;
			linear_vel_y += last_accel_y * warped_delta;
			precise_x += linear_vel_x * warped_delta;
			precise_y += linear_vel_y * warped_delta;
			precise_rotation += angular_vel * warped_delta;

			set_position(Vector2(precise_x, precise_y));
			set_rotation(precise_rotation);

			if (is_rigid) {
				RID rid = get_rid();
				Physics2DServer *server = Physics2DServer::get_singleton();
				server->body_set_state(rid, Physics2DServer::BODY_STATE_LINEAR_VELOCITY, Vector2(linear_vel_x, linear_vel_y) * timescale);
				server->body_set_state(rid, Physics2DServer::BODY_STATE_ANGULAR_VELOCITY, (float)(angular_vel * timescale));
				previous_timescale = timescale;
			}

			last_accel_x = 0.0;
			last_accel_y = 0.0;
		} break;
	}
}

void GravityBody::_sync_pos(Object *p_state) {
	if (is_rigid) {
		Physics2DDirectBodyState *state = Object::cast_to<Physics2DDirectBodyState>(p_state);
		ERR_FAIL_COND_MSG(!state, "Method '_direct_state_changed' must receive a valid Physics2DDirectBodyState object as argument");
		if (state->get_contact_count() > 0) {
			set_block_transform_notify(true);

			Transform2D new_transform = state->get_transform();
			set_global_transform(new_transform);
			angular_vel = state->get_angular_velocity() / previous_timescale;
			linear_vel_x = state->get_linear_velocity().x / previous_timescale;
			linear_vel_y = state->get_linear_velocity().y / previous_timescale;
			precise_x = new_transform.get_origin().x;
			precise_y = new_transform.get_origin().y;
			precise_rotation = new_transform.get_rotation();

			set_block_transform_notify(false);
		}
	}
}

void GravityBody::apply_accel(float128 accel_x, float128 accel_y) {
	last_accel_x += accel_x;
	last_accel_y += accel_y;
}

void GravityBody::set_precise_x(String value) {
	precise_x = std::stold(value.ascii().ptr());
}

String GravityBody::get_precise_x() {
	return std::to_string(precise_x).c_str();
}

void GravityBody::set_precise_y(String value) {
	precise_y = std::stold(value.ascii().ptr());
}

String GravityBody::get_precise_y() {
	return std::to_string(precise_y).c_str();
}

void GravityBody::set_precise_rotation(String value) {
	precise_rotation = std::stold(value.ascii().ptr());
}

String GravityBody::get_precise_rotation() {
	return std::to_string(precise_rotation).c_str();
}

void GravityBody::set_linear_velocity_x(String value) {
	linear_vel_x = std::stold(value.ascii().ptr());
}

String GravityBody::get_linear_velocity_x() {
	return std::to_string(linear_vel_x).c_str();
}

void GravityBody::set_linear_velocity_y(String value) {
	linear_vel_y = std::stold(value.ascii().ptr());
}

String GravityBody::get_linear_velocity_y() {
	return std::to_string(linear_vel_y).c_str();
}

void GravityBody::set_angular_velocity(String value) {
	angular_vel = std::stold(value.ascii().ptr());
}

String GravityBody::get_angular_velocity() {
	return std::to_string(angular_vel).c_str();
}

void GravityBody::set_mass(String value) {
	mass = std::stold(value.ascii().ptr());
	Physics2DServer::get_singleton()->body_set_param(get_rid(), Physics2DServer::BODY_PARAM_MASS, mass);
}

String GravityBody::get_mass() const {
	return std::to_string(mass).c_str();
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

void GravityBody::set_is_rigid(bool value) {
	is_rigid = value;
	Physics2DServer *server = Physics2DServer::get_singleton();
	RID rid = get_rid();
	if (is_rigid) {
		server->body_set_mode(get_rid(), Physics2DServer::BODY_MODE_RIGID);
		server->body_set_max_contacts_reported(rid, 1);
	} else {
		server->body_set_mode(get_rid(), Physics2DServer::BODY_MODE_KINEMATIC);
		server->body_set_max_contacts_reported(rid, 0);
	}
}

bool GravityBody::get_is_rigid() const {
	return is_rigid;
}
