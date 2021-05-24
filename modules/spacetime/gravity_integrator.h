#ifndef GRAVITY_INTEGRATOR_H
#define GRAVITY_INTEGRATOR_H

#include "core/reference.h"

typedef long double float128;

class GravityIntegrator : public Reference {
	GDCLASS(GravityIntegrator, Reference);

private:
	float128 G;

protected:
	static void _bind_methods();

public:
	GravityIntegrator();
	~GravityIntegrator();

	void integrate(real_t delta, real_t timescale, Variant variant);

	void set_G(real_t value);
	real_t get_G();
};

#endif
