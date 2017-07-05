/*************************************************************************/
/*  a_star.h                                                        */
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
#ifndef ASTAR_H
#define ASTAR_H

#include "reference.h"
#include "self_list.h"
/**
	@author Juan Linietsky <reduzio@gmail.com>
*/

class AStar : public Reference {

	GDCLASS(AStar, Reference)

	struct Point {

		Vector3 pos;
		real_t weight_scale;
		Vector<const Point *> neighbours;
	};

	HashMap<Vector3, Point> points;

protected:
	static void _bind_methods();

	virtual float _dist_between(const Vector3 &p_from_pos, const Vector3 &p_to_pos) const;
	virtual float _heuristic_cost(const Vector3 &p_from_pos, const Vector3 &p_to_pos) const;

public:
	void add_point(const Vector3 &p_pos, real_t p_weight_scale = 1);
	void remove_point(const Vector3 &p_pos);

	real_t get_point_weight_scale(const Vector3 &p_pos) const;
	void set_point_weight_scale(const Vector3 &p_pos, real_t p_weight_scale);

	void connect_points(const Vector3 &p_from_pos, const Vector3 &p_to_pos, bool bidirectional = true);
	void disconnect_points(const Vector3 &p_from_pos, const Vector3 &p_to_pos);
	bool are_points_connected(const Vector3 &p_from_pos, const Vector3 &p_to_pos) const;

	void clear();

	PoolVector<Vector3> find_path(const Vector3 &p_from_pos, const Vector3 &p_to_pos) const;
};

#endif // ASTAR_H
