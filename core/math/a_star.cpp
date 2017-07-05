/*************************************************************************/
/*  a_star.cpp                                                           */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                    http://www.godotengine.org                         */
/*************************************************************************/
/* Copyright (c) 2007-2017 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2017 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/
#include "a_star.h"
#include "geometry.h"
#include "scene/scene_string_names.h"
#include "script_language.h"

void AStar::add_point(const Vector3 &p_point, real_t p_weight_scale) {

	ERR_FAIL_COND(points.has(p_point));
	ERR_FAIL_COND(p_weight_scale <= 0);

	Point &pt = points[p_point];
	pt.pos = p_point;
	pt.weight_scale = p_weight_scale;
}

real_t AStar::get_point_weight_scale(const Vector3 &p_point) const {

	ERR_FAIL_COND_V(!points.has(p_point), 0);
	return points[p_point].weight_scale;
}

void AStar::set_point_weight_scale(const Vector3 &p_point, real_t p_weight_scale) {

	ERR_FAIL_COND(!points.has(p_point));
	points[p_point].weight_scale = p_weight_scale;
}

void AStar::remove_point(const Vector3 &p_point) {

	ERR_FAIL_COND(!points.has(p_point));
	points.erase(p_point);
}

void AStar::connect_points(const Vector3 &p_from_point, const Vector3 &p_to_point, bool bidirectional) {

	ERR_FAIL_COND(!points.has(p_from_point));
	ERR_FAIL_COND(!points.has(p_to_point));
	ERR_FAIL_COND(p_from_point == p_to_point);

	Point &a = points[p_from_point];
	Point &b = points[p_to_point];
	a.neighbours.push_back(&b);

	if (bidirectional)
		b.neighbours.push_back(&a);
}

void AStar::disconnect_points(const Vector3 &p_from_point, const Vector3 &p_to_point) {

	ERR_FAIL_COND(!points.has(p_from_point));
	ERR_FAIL_COND(!points.has(p_to_point));

	Point &a = points[p_from_point];
	Point &b = points[p_to_point];
	a.neighbours.erase(&b);
	b.neighbours.erase(&a);
}

bool AStar::are_points_connected(const Vector3 &p_from_point, const Vector3 &p_to_point) const {

	ERR_FAIL_COND_V(!points.has(p_from_point), false);
	ERR_FAIL_COND_V(!points.has(p_to_point), false);

	const Point &a = points[p_from_point];
	const Point &b = points[p_to_point];
	return a.neighbours.find(&b, 0) != -1 || b.neighbours.find(&a, 0) != -1; // TODO Optimize
}

void AStar::clear() {

	points.clear();
}

float AStar::_dist_between(const Vector3 &p_from_point, const Vector3 &p_to_point) const {
	if (get_script_instance() && get_script_instance()->has_method(SceneStringNames::get_singleton()->_estimate_cost))
		return get_script_instance()->call(SceneStringNames::get_singleton()->_estimate_cost, p_from_point, p_to_point);

	return p_from_point.distance_squared_to(p_to_point);
}

float AStar::_heuristic_cost(const Vector3 &p_from_point, const Vector3 &p_end_point) const  {
	if (get_script_instance() && get_script_instance()->has_method(SceneStringNames::get_singleton()->_compute_cost))
		return get_script_instance()->call(SceneStringNames::get_singleton()->_compute_cost, p_from_point, p_end_point);

	//_compute_cost(p->id, e->id) * e->weight_scale + p->distance;
	return p_from_point.distance_squared_to(p_end_point);
}

PoolVector<Vector3> AStar::find_path(const Vector3 &p_from_point, const Vector3 &p_to_point) const {

	ERR_FAIL_COND_V(!points.has(p_from_point), PoolVector<Vector3>());
	ERR_FAIL_COND_V(!points.has(p_to_point), PoolVector<Vector3>());
	ERR_FAIL_COND_V(p_from_point == p_to_point, PoolVector<Vector3>());

	const Point &begin_point = points[p_from_point];
	const Point &end_point = points[p_to_point];

	Set<Vector3> open_set;
	Set<Vector3> closed_set;

	HashMap<Vector3, Vector3> came_from;
	HashMap<Vector3, real_t> f_score, g_score;

	g_score[p_from_point] = 0;
	f_score[p_from_point] = _heuristic_cost(p_from_point, p_to_point);

	bool success = false;

	open_set.insert(p_from_point);
	while (open_set.size() > 0) {

		const Vector3 &cur_pos = open_set.front()->get();	
		const Point &cur_point = points[cur_pos];

		success = cur_pos == p_to_point;
		if (success)
			break;

		open_set.erase(cur_pos);
		closed_set.insert(cur_pos);

		for (int i = 0; i < cur_point.neighbours.size(); ++i) {

			const Point &neighbor = *cur_point.neighbours[i];

			// Have already explored?
			if (closed_set.has(neighbor.pos))
				continue;
			
			// Add nodes that aren't already being evaluated
			if (!open_set.has(neighbor.pos))
				open_set.insert(neighbor.pos);

			// Check if it is a better path
			real_t tentative_g_score = g_score[cur_pos] + _dist_between(cur_pos, neighbor.pos);
			if (tentative_g_score >= g_score[neighbor.pos])
				continue; 
			
			// Best known path; record it
			came_from[neighbor.pos] = cur_pos;
			g_score[neighbor.pos] = tentative_g_score;
			f_score[neighbor.pos] = g_score[neighbor.pos] + _heuristic_cost(neighbor.pos, p_to_point);
		}
	}

	print_line(success ? "True" : "false");
	if (!success)
		return PoolVector<Vector3>();
	
	// Count number of points
	int points_count = 0;
	for (Vector3 p = came_from[p_to_point]; p != p_from_point; p = came_from[p]) {
		++points_count;
		print_line(p);
	}

	print_line(Variant(points_count));
	PoolVector<Vector3> path;
	path.resize(points_count);

	// Build the path in reverse
	PoolVector<Vector3>::Write w = path.write();
	for (Vector3 p = came_from[p_to_point]; p != p_from_point; p = came_from[p])
		w[--points_count] = p;
	w[0] = p_from_point;

	return path;
}

void AStar::_bind_methods() {

	ClassDB::bind_method(D_METHOD("add_point", "point", "weight_scale"), &AStar::add_point, DEFVAL(1.0));
	ClassDB::bind_method(D_METHOD("remove_point", "point"), &AStar::remove_point);

	ClassDB::bind_method(D_METHOD("get_point_weight_scale", "point"), &AStar::get_point_weight_scale);
	ClassDB::bind_method(D_METHOD("set_point_weight_scale", "point", "weight_scale"), &AStar::set_point_weight_scale);

	ClassDB::bind_method(D_METHOD("connect_points", "from_point", "to_point", "bidirectional"), &AStar::connect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("disconnect_points", "from_point", "to_point"), &AStar::disconnect_points);
	ClassDB::bind_method(D_METHOD("are_points_connected", "from_point", "to_point"), &AStar::are_points_connected);

	ClassDB::bind_method(D_METHOD("clear"), &AStar::clear);

	ClassDB::bind_method(D_METHOD("find_path", "from_point", "to_point"), &AStar::find_path);

	BIND_VMETHOD(MethodInfo("_dist_between", PropertyInfo(Variant::VECTOR3, "from_point"), PropertyInfo(Variant::VECTOR3, "to_point")));
	BIND_VMETHOD(MethodInfo("_heuristic_cost", PropertyInfo(Variant::VECTOR3, "from_point"), PropertyInfo(Variant::VECTOR3, "to_point")));
}