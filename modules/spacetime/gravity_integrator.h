#ifndef GRAVITY_INTEGRATOR_H
#define GRAVITY_INTEGRATOR_H

#include "core/reference.h"
#include "scene/main/scene_tree.h"

typedef long double float128;

class GravityBody;

class GravityIntegrator : public Reference {
	GDCLASS(GravityIntegrator, Reference);

private:
	float128 G;

	struct PredictionState {
		GravityBody *parent;
		float128 x;
		float128 y;
		float128 vel_x;
		float128 vel_y;
		float128 accel_x;
		float128 accel_y;
	};

	PredictionState *prediction_states;
	int ps_size;

	real_t pred_max_err;
	int pred_max_steps;
	int pred_saved_steps;

protected:
	static void _bind_methods();

public:
	GravityIntegrator();
	~GravityIntegrator();

	void integrate(Variant p_scene_tree, real_t delta, real_t timescale);
	void predict(Variant p_scene_tree, Variant p_relative, real_t duration, real_t timestep);

	void set_G(real_t value);
	real_t get_G();

	void set_pred_max_err(real_t value);
	real_t get_pred_max_err();

	void set_pred_max_steps(int value);
	int get_pred_max_steps();

	void set_pred_saved_steps(int value);
	int get_pred_saved_steps();
};

#endif
