#ifndef GRAVITY_BODY_H
#define GRAVITY_BODY_H

#include "gravity_integrator.h"
#include "scene/2d/physics_body_2d.h"

typedef long double float128;

class GravityBody : public RigidBody2D {
	GDCLASS(GravityBody, RigidBody2D);
	friend class GravityIntegrator;

private:
	float128 x;
	float128 y;

	float128 vel_x;
	float128 vel_y;

	float previous_timescale;
	float timescale;
	float128 last_accel_x;
	float128 last_accel_y;

	float gbody_mass;
	float128 internal_mass;

	bool is_significant;

protected:
	static void _bind_methods();

public:
	GravityBody();
	~GravityBody();

	void _notification(int p_what);
	void _integrate_forces(Variant variant);

	void apply_accel(float128 accel_x, float128 accel_y);

	void set_x(real_t value);
	real_t get_x() const;

	void set_y(real_t value);
	real_t get_y() const;

	void set_vel_x(real_t value);
	real_t get_vel_x() const;

	void set_vel_y(real_t value);
	real_t get_vel_y() const;

	void set_timescale(real_t value);

	void set_gbody_mass(real_t value);
	real_t get_gbody_mass() const;

	void set_significance(bool value);
	bool get_significance() const;
};

#endif
