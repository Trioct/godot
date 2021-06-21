#include "gravity_integrator.h"

#include "gravity_body.h"
#include "scene/main/scene_tree.h"

template <typename T>
T *from_variant(Variant &variant) {
	Object *attempt = variant.operator Object *();
	if (attempt == nullptr) {
		print_error("Expected " + T::get_class_static());
	}
	return static_cast<T *>(attempt);
}

void IntegrationStats::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_time_passed", "value"), &IntegrationStats::set_time_passed);
	ClassDB::bind_method(D_METHOD("get_time_passed"), &IntegrationStats::get_time_passed);
	ClassDB::bind_method(D_METHOD("set_iterations", "value"), &IntegrationStats::set_iterations);
	ClassDB::bind_method(D_METHOD("get_iterations"), &IntegrationStats::get_iterations);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "time_passed"), "set_time_passed", "get_time_passed");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "iterations"), "set_iterations", "get_iterations");
}

IntegrationStats::IntegrationStats() {
	time_passed = 0.0;
	iterations = 0;
}

void GravityIntegrator::_bind_methods() {
	ClassDB::bind_method(D_METHOD("integrate", "scene_tree", "delta", "timescale"), &GravityIntegrator::integrate);
	ClassDB::bind_method(D_METHOD("predict", "scene_tree", "rel_body", "duration", "timescale"), &GravityIntegrator::predict);

	ClassDB::bind_method(D_METHOD("set_G", "value"), &GravityIntegrator::set_G);
	ClassDB::bind_method(D_METHOD("get_G"), &GravityIntegrator::get_G);
	ClassDB::bind_method(D_METHOD("set_int_max_err", "value"), &GravityIntegrator::set_int_max_err);
	ClassDB::bind_method(D_METHOD("get_int_max_err"), &GravityIntegrator::get_int_max_err);
	ClassDB::bind_method(D_METHOD("set_int_max_steps", "value"), &GravityIntegrator::set_int_max_steps);
	ClassDB::bind_method(D_METHOD("get_int_max_steps"), &GravityIntegrator::get_int_max_steps);
	ClassDB::bind_method(D_METHOD("set_pred_max_err", "value"), &GravityIntegrator::set_pred_max_err);
	ClassDB::bind_method(D_METHOD("get_pred_max_err"), &GravityIntegrator::get_pred_max_err);
	ClassDB::bind_method(D_METHOD("set_pred_max_steps", "value"), &GravityIntegrator::set_pred_max_steps);
	ClassDB::bind_method(D_METHOD("get_pred_max_steps"), &GravityIntegrator::get_pred_max_steps);
	ClassDB::bind_method(D_METHOD("set_pred_saved_steps", "value"), &GravityIntegrator::set_pred_saved_steps);
	ClassDB::bind_method(D_METHOD("get_pred_saved_steps"), &GravityIntegrator::get_pred_saved_steps);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "G"), "set_G", "get_G");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "int_max_err"), "set_int_max_err", "get_int_max_err");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "int_max_steps"), "set_int_max_steps", "get_int_max_steps");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "pred_max_err"), "set_pred_max_err", "get_pred_max_err");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "pred_max_steps"), "set_pred_max_steps", "get_pred_max_steps");
}

GravityIntegrator::GravityIntegrator() {
	G = 0.00000000006674;
	prediction_states = (PredictionState *)std::malloc(sizeof(PredictionState) * 1024);
	ps_size = 1024;
	int_max_err = 0.05;
	int_max_steps = 1024;
	pred_max_err = 0.05;
	pred_max_steps = 1024;
	pred_saved_steps = 128;
}

GravityIntegrator::~GravityIntegrator() {
	free(prediction_states);
}

std::pair<float128, float128> calc_premass(float128 G, float128 x1, float128 y1, float128 x2, float128 y2) {
	float128 diff_x = x2 - x1;
	float128 diff_y = y2 - y1;
	float128 squared_distance = diff_x * diff_x + diff_y * diff_y;
	float128 distance = sqrtl(squared_distance);
	return std::pair<float128, float128>(
			G * diff_x / (squared_distance * distance),
			G * diff_y / (squared_distance * distance));
}

template <typename T>
real_t GravityIntegrator::integrate_inner(List<Node *> &s_bodies,
		List<Node *> &i_bodies,
		const real_t delta,
		const real_t max_error,
		std::function<T *(List<Node *> &body_set, int index)> get_state,
		std::function<void(T *, T *, float128, float128)> accel_f,
		std::function<void(T *, float128)> move_f) {

	float128 cur_max_err = 0.0;
	float128 warped_delta = delta;
	for (int i = 0; i < s_bodies.size(); ++i) {
		T *node_i = get_state(s_bodies, i);
		if (!node_i) {
			continue;
		}

		for (int j = 0; j < i_bodies.size(); ++j) {
			T *node_j = get_state(i_bodies, j);
			if (!node_j) {
				continue;
			}
			std::pair<float128, float128> accel_premass =
					calc_premass(G,
							node_j->precise_x, node_j->precise_y,
							node_i->precise_x, node_i->precise_y);
			accel_f(node_j, node_i, accel_premass.first, accel_premass.second);
		}

		for (int j = i + 1; j < s_bodies.size(); ++j) {
			T *node_j = get_state(s_bodies, j);
			if (!node_j) {
				continue;
			}

			std::pair<float128, float128> accel_premass =
					calc_premass(G,
							node_i->precise_x, node_i->precise_y,
							node_j->precise_x, node_j->precise_y);
			accel_f(node_i, node_j, accel_premass.first, accel_premass.second);
			accel_f(node_j, node_i, -accel_premass.first, -accel_premass.second);
		}
	}
	cur_max_err = max_error;
	for (int i = 0; i < s_bodies.size() + i_bodies.size(); ++i) {
		T *node_i;
		if (i < s_bodies.size()) {
			node_i = get_state(s_bodies, i);
		} else {
			node_i = get_state(i_bodies, i - s_bodies.size());
		}

		float128 accel = node_i->accel_x * node_i->accel_x + node_i->accel_y * node_i->accel_y;
		float128 vel = node_i->linear_vel_x * node_i->linear_vel_x + node_i->linear_vel_y * node_i->linear_vel_y;
		cur_max_err = std::max(cur_max_err, delta * delta * accel / vel);
	}
	warped_delta = delta * sqrt(max_error / cur_max_err);
	for (int i = 0; i < s_bodies.size() + i_bodies.size(); ++i) {
		T *node_i;
		if (i < s_bodies.size()) {
			node_i = get_state(s_bodies, i);
		} else {
			node_i = get_state(i_bodies, i - s_bodies.size());
		}

		move_f(node_i, warped_delta);
	}
	return warped_delta;
}

Ref<IntegrationStats> GravityIntegrator::integrate(Variant p_scene_tree, const real_t delta, const real_t timescale) {
	SceneTree *tree = from_variant<SceneTree>(p_scene_tree);
	Ref<IntegrationStats> stats(memnew(IntegrationStats));
	if (!tree) {
		return stats;
	}

	List<Node *> s_bodies;
	tree->get_nodes_in_group("significant_bodies", &s_bodies);
	List<Node *> i_bodies;
	tree->get_nodes_in_group("insignificant_bodies", &i_bodies);
	for (float128 duration = delta * timescale; duration - stats->time_passed > 0 && stats->iterations < int_max_steps;) {
		float128 passed = integrate_inner<GravityBody>(
				s_bodies,
				i_bodies,
				delta * timescale,
				pow(tan(int_max_err * 3.14159265358979 / 180), 2),
				[](List<Node *> &body_set, int index) {
					return Object::cast_to<GravityBody>(body_set[index]);
				},
				[](GravityBody *self, GravityBody *other, float128 accel_x, float128 accel_y) {
					self->apply_accel(accel_x * other->mass, accel_y * other->mass);
				},
				[&](GravityBody *node, float128 warped_delta) {
					node->timescale = timescale;
					node->move(warped_delta, true);
				});
		stats->time_passed += passed;
		++stats->iterations;
	}
	return stats;
}

void GravityIntegrator::predict(Variant p_scene_tree, Variant p_relative, real_t duration, real_t timescale) {
	SceneTree *tree = from_variant<SceneTree>(p_scene_tree);
	if (!tree) {
		return;
	}

	GravityBody *rel_body = p_relative.get_type() == Variant::NIL ? nullptr : from_variant<GravityBody>(p_relative);
	PredictionState *rel_state = nullptr;
	auto add_point = [&](GravityBody *body, const PredictionState &state) {
		float128 rel_x = rel_state ? rel_state->precise_x : 0;
		float128 rel_y = rel_state ? rel_state->precise_y : 0;
		body->prediction.append(Vector2(state.precise_x - rel_x, state.precise_y - rel_y));
	};

	List<Node *> s_bodies;
	tree->get_nodes_in_group("significant_bodies", &s_bodies);
	List<Node *> i_bodies;
	tree->get_nodes_in_group("insignificant_bodies", &i_bodies);

	bool size_changed = false;
	while (s_bodies.size() + i_bodies.size() > ps_size) {
		ps_size *= 2;
		size_changed = true;
	}

	if (size_changed) {
		prediction_states = (PredictionState *)std::realloc(prediction_states, sizeof(PredictionState) * ps_size);
	}

	for (int i = 0; i < s_bodies.size() + i_bodies.size(); ++i) {
		GravityBody *node_i;
		if (i < s_bodies.size()) {
			node_i = Object::cast_to<GravityBody>(s_bodies[i]);
		} else {
			node_i = Object::cast_to<GravityBody>(i_bodies[i - s_bodies.size()]);
		}
		PredictionState &state = prediction_states[i];
		if (!node_i) {
			continue;
		}

		if (node_i == rel_body) {
			rel_state = &state;
		}

		state.parent = node_i;
		state.precise_x = node_i->precise_x;
		state.precise_y = node_i->precise_y;
		state.linear_vel_x = node_i->linear_vel_x;
		state.linear_vel_y = node_i->linear_vel_y;
		state.accel_x = 0;
		state.accel_y = 0;

		node_i->prediction = PoolVector2Array();
	}

	int counter = 0;
	while (duration > 0 && counter < pred_max_steps) {
		float128 passed = integrate_inner<PredictionState>(
				s_bodies,
				i_bodies,
				timescale,
				pow(tan(pred_max_err * 3.14159265358979 / 180), 2),
				[this](List<Node *> &body_set, int index) {
					return &prediction_states[index];
				},
				[](PredictionState *self, PredictionState *other, float128 accel_x, float128 accel_y) {
					self->accel_x += accel_x * other->parent->mass;
					self->accel_y += accel_y * other->parent->mass;
				},
				[&](PredictionState *node, float128 warped_delta) {
					node->linear_vel_x += node->accel_x * warped_delta;
					node->linear_vel_y += node->accel_y * warped_delta;
					node->precise_x += node->linear_vel_x * warped_delta;
					node->precise_y += node->linear_vel_y * warped_delta;

					node->accel_x = 0;
					node->accel_y = 0;
				});

		duration -= passed;
		if (counter % (pred_max_steps / pred_saved_steps) == 0) {
			for (int j = 0; j < s_bodies.size() + i_bodies.size(); ++j) {
				PredictionState &state = prediction_states[j];
				add_point(state.parent, state);
			}
		}

		++counter;
	}
}
