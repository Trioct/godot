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

void GravityIntegrator::_bind_methods() {
	ClassDB::bind_method(D_METHOD("integrate", "scene_tree", "delta", "timescale"), &GravityIntegrator::integrate);
	ClassDB::bind_method(D_METHOD("predict", "rel_body", "scene_tree", "duration", "timestep"), &GravityIntegrator::predict);

	ClassDB::bind_method(D_METHOD("set_G", "value"), &GravityIntegrator::set_G);
	ClassDB::bind_method(D_METHOD("get_G"), &GravityIntegrator::get_G);
	ClassDB::bind_method(D_METHOD("set_pred_max_err", "value"), &GravityIntegrator::set_pred_max_err);
	ClassDB::bind_method(D_METHOD("get_pred_max_err"), &GravityIntegrator::get_pred_max_err);
	ClassDB::bind_method(D_METHOD("set_pred_max_steps", "value"), &GravityIntegrator::set_pred_max_steps);
	ClassDB::bind_method(D_METHOD("get_pred_max_steps"), &GravityIntegrator::get_pred_max_steps);
	ClassDB::bind_method(D_METHOD("set_pred_saved_steps", "value"), &GravityIntegrator::set_pred_saved_steps);
	ClassDB::bind_method(D_METHOD("get_pred_saved_steps"), &GravityIntegrator::get_pred_saved_steps);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "G"), "set_G", "get_G");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "pred_max_err"), "set_pred_max_err", "get_pred_max_err");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "pred_max_steps"), "set_pred_max_steps", "get_pred_max_steps");
}

GravityIntegrator::GravityIntegrator() {
	G = 0.00000000006674;
	prediction_states = (PredictionState *)std::malloc(sizeof(PredictionState) * 1024);
	ps_size = 1024;
	pred_max_err = 1.0;
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

void GravityIntegrator::integrate(Variant p_scene_tree, const real_t delta, const real_t timescale) {
	SceneTree *tree = from_variant<SceneTree>(p_scene_tree);
	if (!tree) {
		return;
	}

	List<Node *> s_bodies;
	tree->get_nodes_in_group("significant_bodies", &s_bodies);
	List<Node *> i_bodies;
	tree->get_nodes_in_group("insignificant_bodies", &i_bodies);
	for (int i = 0; i < s_bodies.size(); ++i) {
		GravityBody *node_i = Object::cast_to<GravityBody>(s_bodies[i]);
		if (!node_i) {
			continue;
		}
		node_i->timescale = timescale;
		for (int j = 0; j < i_bodies.size(); ++j) {
			GravityBody *node_j =
					Object::cast_to<GravityBody>(i_bodies[j]);
			if (!node_j) {
				continue;
			}
			node_j->timescale = timescale;
			std::pair<float128, float128> accel_premass =
					calc_premass(G,
							node_j->precise_x, node_j->precise_y,
							node_i->precise_x, node_i->precise_y);
			node_j->apply_accel(accel_premass.first * node_i->mass,
					accel_premass.second * node_i->mass);
		}
		for (int j = i + 1; j < s_bodies.size(); ++j) {
			GravityBody *node_j =
					Object::cast_to<GravityBody>(s_bodies[j]);
			if (!node_j) {
				continue;
			}
			std::pair<float128, float128> accel_premass =
					calc_premass(G,
							node_i->precise_x, node_i->precise_y,
							node_j->precise_x, node_j->precise_y);
			node_i->apply_accel(accel_premass.first * node_j->mass,
					accel_premass.second * node_j->mass);
			node_j->apply_accel(-accel_premass.first * node_i->mass,
					-accel_premass.second * node_i->mass);
		}
	}
	if (s_bodies.size() == 0) {
		for (int i = 0; i < i_bodies.size(); ++i) {
			GravityBody *node_i =
					Object::cast_to<GravityBody>(i_bodies[i]);
			if (!node_i) {
				continue;
			}
			node_i->timescale = timescale;
		}
	}
}

void GravityIntegrator::predict(Variant p_scene_tree, Variant p_relative, real_t duration, real_t timestep) {
	SceneTree *tree = from_variant<SceneTree>(p_scene_tree);
	if (!tree) {
		return;
	}

	GravityBody *rel_body = p_relative.get_type() == Variant::NIL ? nullptr : from_variant<GravityBody>(p_relative);
	PredictionState *rel_state = nullptr;
	auto add_point = [&](GravityBody *body, const PredictionState &state) {
		float128 rel_x = rel_state ? rel_state->x : 0;
		float128 rel_y = rel_state ? rel_state->y : 0;
		body->prediction.append(Vector2(state.x - rel_x, state.y - rel_y));
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

	// wtf was this???
	// float128 max_error = 1.0 / (1.0 / pow(sin(pred_max_err * 3.14159265358979 / 180), 2) - 1);
	const float128 max_error = pow(tan(pred_max_err * 3.14159265358979 / 180), 2);

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

		state.parent = node_i;
		state.x = node_i->precise_x;
		state.y = node_i->precise_y;
		state.vel_x = node_i->linear_vel_x;
		state.vel_y = node_i->linear_vel_y;
		state.accel_x = 0;
		state.accel_y = 0;

		node_i->prediction = PoolVector2Array();
		//node_i->prediction.append(Vector2(0, 0));
	}

	float128 max_fd_ratio = 0.0;
	for (int i = 0; i < pred_max_steps && duration > 0; ++i) {
		for (int j = 0; j < s_bodies.size(); ++j) {
			GravityBody *node_j =
					Object::cast_to<GravityBody>(s_bodies[j]);
			PredictionState &state_j = prediction_states[j];
			if (!node_j) {
				continue;
			}
			if (rel_body && !rel_state && rel_body == node_j) {
				rel_state = &state_j;
			}
			for (int k = 0; k < i_bodies.size(); ++k) {
				GravityBody *node_k =
						Object::cast_to<GravityBody>(i_bodies[k]);
				PredictionState &state_k = prediction_states[s_bodies.size() + k];
				if (!node_k) {
					continue;
				}
				std::pair<float128, float128> accel_premass =
						calc_premass(G, state_k.x, state_k.y, state_j.x, state_j.y);
				state_k.accel_x += accel_premass.first * node_j->mass;
				state_k.accel_y += accel_premass.second * node_j->mass;
			}
			for (int k = j + 1; k < s_bodies.size(); ++k) {
				GravityBody *node_k =
						Object::cast_to<GravityBody>(s_bodies[k]);
				PredictionState &state_k = prediction_states[k];
				if (!node_k) {
					continue;
				}
				std::pair<float128, float128> accel_premass =
						calc_premass(G, state_j.x, state_j.y, state_k.x, state_k.y);
				state_j.accel_x += accel_premass.first * node_k->mass;
				state_j.accel_y += accel_premass.second * node_k->mass;
				state_k.accel_x -= accel_premass.first * node_j->mass;
				state_k.accel_y -= accel_premass.second * node_j->mass;
			}
		}
		max_fd_ratio = max_error;
		for (int j = 0; j < s_bodies.size() + i_bodies.size(); ++j) {
			PredictionState &state = prediction_states[j];

			float128 accel = state.accel_x * state.accel_x + state.accel_y * state.accel_y;
			float128 distance = state.vel_x * state.vel_x + state.vel_y * state.vel_y;
			max_fd_ratio = std::max(max_fd_ratio, timestep * timestep * accel / distance);
		}
		float128 delta = timestep * sqrt(max_error / max_fd_ratio);
		duration -= delta;

		for (int j = 0; j < s_bodies.size() + i_bodies.size(); ++j) {
			PredictionState &state = prediction_states[j];

			state.vel_x += state.accel_x * delta;
			state.vel_y += state.accel_y * delta;
			state.x += state.vel_x * delta;
			state.y += state.vel_y * delta;
			if (i % (pred_max_steps / pred_saved_steps) == 0) {
				add_point(state.parent, state);
			}

			state.accel_x = 0;
			state.accel_y = 0;
		}
	}
}

void GravityIntegrator::set_G(real_t value) {
	G = value;
}

real_t GravityIntegrator::get_G() {
	return G;
}

void GravityIntegrator::set_pred_max_err(real_t value) {
	pred_max_err = value;
}

real_t GravityIntegrator::get_pred_max_err() {
	return pred_max_err;
}

void GravityIntegrator::set_pred_max_steps(int value) {
	pred_max_steps = value;
}

int GravityIntegrator::get_pred_max_steps() {
	return pred_max_steps;
}

void GravityIntegrator::set_pred_saved_steps(int value) {
	pred_saved_steps = value;
}

int GravityIntegrator::get_pred_saved_steps() {
	return pred_saved_steps;
}
