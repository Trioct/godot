#ifndef GRAVITY_INTEGRATOR_H
#define GRAVITY_INTEGRATOR_H

#include "core/reference.h"
#include "scene/main/scene_tree.h"
#include <functional>

typedef long double float128;

class GravityBody;
class PrecisePosition {
public:
	float128 precise_x;
	float128 precise_y;

	float128 linear_vel_x;
	float128 linear_vel_y;

	float128 accel_x;
	float128 accel_y;
};

class IntegrationStats : public Reference {
	GDCLASS(IntegrationStats, Reference);

protected:
	static void _bind_methods();

public:
	IntegrationStats();
	IntegrationStats(IntegrationStats &stats) :
			time_passed(stats.time_passed), iterations(stats.iterations) {}

	real_t time_passed;
	int iterations;

	void set_time_passed(real_t value) { time_passed = value; }
	real_t get_time_passed() { return time_passed; }

	void set_iterations(int value) { iterations = value; }
	int get_iterations() { return iterations; }
};

class GravityIntegrator : public Reference {
	GDCLASS(GravityIntegrator, Reference);

private:
	float128 G;

	class PredictionState : public PrecisePosition {
	public:
		GravityBody *parent;
	};

	PredictionState *prediction_states;
	int ps_size;

	real_t int_max_err;
	int int_max_steps;

	real_t pred_max_err;
	int pred_max_steps;
	int pred_saved_steps;

	template <typename T>
	real_t integrate_inner(List<Node *> &s_bodies,
			List<Node *> &i_bodies,
			const real_t delta,
			const real_t max_error,
			std::function<T *(List<Node *> &body_set, int index)> get_state,
			std::function<void(T *, T *, float128, float128)> accel_f,
			std::function<void(T *, float128)> move_f);

protected:
	static void _bind_methods();

public:
	GravityIntegrator();
	~GravityIntegrator();

	Ref<IntegrationStats> integrate(Variant p_scene_tree, real_t delta, real_t timescale);
	void predict(Variant p_scene_tree, Variant p_relative, real_t duration, real_t timescale);

	void set_G(real_t value) { G = value; }
	real_t get_G() { return G; }

	void set_int_max_err(real_t value) { int_max_err = value; }
	real_t get_int_max_err() { return int_max_err; }

	void set_int_max_steps(int value) { int_max_steps = value; }
	int get_int_max_steps() { return int_max_steps; }

	void set_pred_max_err(real_t value) { pred_max_err = value; }
	real_t get_pred_max_err() { return pred_max_err; }

	void set_pred_max_steps(int value) { pred_max_steps = value; }
	int get_pred_max_steps() { return pred_max_steps; }

	void set_pred_saved_steps(int value) { pred_saved_steps = value; }
	int get_pred_saved_steps() { return pred_saved_steps; }
};

#endif
