#ifndef GRAVITY_BODY_H
#define GRAVITY_BODY_H

#include "gravity_integrator.h"
#include "scene/2d/physics_body_2d.h"

typedef long double float128;

class GravityBody : public PhysicsBody2D {
	GDCLASS(GravityBody, PhysicsBody2D);
	friend class GravityIntegrator;

private:
	float128 precise_x;
	float128 precise_y;
	float128 precise_rotation;

	float128 linear_vel_x;
	float128 linear_vel_y;

	float128 angular_vel;

	float128 mass;

	float timescale;
	bool is_rigid;
	bool is_significant;

	float previous_timescale;
	float128 last_accel_x;
	float128 last_accel_y;

protected:
	static void _bind_methods();

public:
	GravityBody();
	~GravityBody();

	void _notification(int p_what);
	void _sync_pos(Object *p_state);

	void apply_accel(float128 accel_x, float128 accel_y);

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

	void set_timescale(real_t value);

	void set_significance(bool value);
	bool get_significance() const;

	void set_is_rigid(bool value);
	bool get_is_rigid() const;
};

#endif
