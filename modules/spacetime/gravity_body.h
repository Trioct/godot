#ifndef GRAVITY_BODY_H
#define GRAVITY_BODY_H

#include "gravity_integrator.h"
#include "scene/2d/physics_body_2d.h"

typedef long double float128;

class GravityBody : public PhysicsBody2D, PrecisePosition {
	GDCLASS(GravityBody, PhysicsBody2D);
	friend class GravityIntegrator;

private:
	float128 precise_rotation;
	float128 angular_vel;
	float128 mass;

	PoolVector2Array prediction;

	real_t previous_timescale;
	real_t timescale;
	bool is_rigid;
	bool is_significant;

	bool skip_next_process;
	bool sync_to_physics;
	Vector<std::pair<Vector2, Vector2> > last_impulses;
	float128 accel_angular;

protected:
	static void _bind_methods();

public:
	GravityBody();
	~GravityBody();

	void _notification(int p_what);
	void _sync_pos(Object *p_state);

	void apply_accel(float128 accel_x, float128 accel_y);
	void move(real_t delta, bool skip_next_process = false);

	void apply_impulse(Vector2 offset, Vector2 impulse);
	void apply_central_impulse(Vector2 impulse);
	void apply_torque_impulse(real_t impulse);

	// for use only for gdscript
	void set_precise_x(String value);
	String get_precise_x();
	void set_precise_y(String value);
	String get_precise_y();
	void set_precise_rotation(String value);
	String get_precise_rotation();
	void set_linear_velocity_x(String value);
	String get_linear_velocity_x();
	void set_linear_velocity_y(String value);
	String get_linear_velocity_y();
	void set_angular_velocity(String value);
	String get_angular_velocity();
	void set_mass(String value);
	String get_mass() const;

	void set_significance(bool value);
	bool get_significance() const;

	void set_is_rigid(bool value);
	bool get_is_rigid() const;

	PoolVector2Array get_prediction() const;
};

#endif
