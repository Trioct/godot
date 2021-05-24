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
	ClassDB::bind_method(D_METHOD("integrate", "delta", "timescale", "scene_tree"), &GravityIntegrator::integrate);
	ClassDB::bind_method(D_METHOD("set_G", "value"), &GravityIntegrator::set_G);
	ClassDB::bind_method(D_METHOD("get_G"), &GravityIntegrator::get_G);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "G"), "set_G", "get_G");
}

GravityIntegrator::GravityIntegrator() {
	G = 0.00000000006674;
}
GravityIntegrator::~GravityIntegrator() {}

void GravityIntegrator::integrate(const real_t delta, const real_t timescale,
		Variant variant) {
	SceneTree *tree = from_variant<SceneTree>(variant);
	if (!tree) {
		return;
	}

	auto calc_premass = [this](const GravityBody &node1,
								const GravityBody &node2) {
		float128 diff_x = node2.x - node1.x;
		float128 diff_y = node2.y - node1.y;
		float128 squared_distance = diff_x * diff_x + diff_y * diff_y;
		float128 distance = sqrtl(squared_distance);
		return std::pair<float128, float128>(
				G * diff_x / (squared_distance * distance),
				G * diff_y / (squared_distance * distance));
	};

	List<Node *> significant_bodies;
	tree->get_nodes_in_group("significant_bodies", &significant_bodies);
	List<Node *> insignificant_bodies;
	tree->get_nodes_in_group("insignificant_bodies", &insignificant_bodies);
	for (int i = 0; i < significant_bodies.size(); ++i) {
		GravityBody *node_i = Object::cast_to<GravityBody>(significant_bodies[i]);
		if (!node_i) {
			continue;
		}
		node_i->set_timescale(timescale);
		for (int j = 0; j < insignificant_bodies.size(); ++j) {
			GravityBody *node_j =
					Object::cast_to<GravityBody>(insignificant_bodies[j]);
			if (!node_j) {
				continue;
			}
			node_j->set_timescale(timescale);
			std::pair<float128, float128> accel_premass =
					calc_premass(*node_j, *node_i);
			node_j->apply_accel(accel_premass.first * node_i->internal_mass,
					accel_premass.second * node_i->internal_mass);
		}
		for (int j = i + 1; j < significant_bodies.size(); ++j) {
			GravityBody *node_j =
					Object::cast_to<GravityBody>(significant_bodies[j]);
			if (!node_j) {
				continue;
			}
			std::pair<float128, float128> accel_premass =
					calc_premass(*node_i, *node_j);
			node_i->apply_accel(accel_premass.first * node_j->internal_mass,
					accel_premass.second * node_j->internal_mass);
			node_j->apply_accel(-accel_premass.first * node_i->internal_mass,
					-accel_premass.second * node_i->internal_mass);
		}
	}
}

void GravityIntegrator::set_G(real_t value) {
	G = value;
}
real_t GravityIntegrator::get_G() {
	return G;
}
