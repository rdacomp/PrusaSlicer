#include "PerimeterGenerator.hpp"
#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "ShortestPath.hpp"

#include <cmath>
#include <cassert>

namespace Slic3r {

static ExtrusionPaths thick_polyline_to_extrusion_paths(const ThickPolyline &thick_polyline, ExtrusionRole role, Flow &flow, const float tolerance)
{
    ExtrusionPaths paths;
    ExtrusionPath path(role);
    ThickLines lines = thick_polyline.thicklines();
    
    for (int i = 0; i < (int)lines.size(); ++i) {
        const ThickLine& line = lines[i];
        
        const coordf_t line_len = line.length();
        if (line_len < SCALED_EPSILON) continue;
        
        double thickness_delta = fabs(line.a_width - line.b_width);
        if (thickness_delta > tolerance) {
            const unsigned int segments = (unsigned int)ceil(thickness_delta / tolerance);
            const coordf_t seg_len = line_len / segments;
            Points pp;
            std::vector<coordf_t> width;
            {
                pp.push_back(line.a);
                width.push_back(line.a_width);
                for (size_t j = 1; j < segments; ++j) {
                    pp.push_back((line.a.cast<double>() + (line.b - line.a).cast<double>().normalized() * (j * seg_len)).cast<coord_t>());
                    
                    coordf_t w = line.a_width + (j*seg_len) * (line.b_width-line.a_width) / line_len;
                    width.push_back(w);
                    width.push_back(w);
                }
                pp.push_back(line.b);
                width.push_back(line.b_width);
                
                assert(pp.size() == segments + 1u);
                assert(width.size() == segments*2);
            }
            
            // delete this line and insert new ones
            lines.erase(lines.begin() + i);
            for (size_t j = 0; j < segments; ++j) {
                ThickLine new_line(pp[j], pp[j+1]);
                new_line.a_width = width[2*j];
                new_line.b_width = width[2*j+1];
                lines.insert(lines.begin() + i + j, new_line);
            }
            
            -- i;
            continue;
        }
        
        const double w = fmax(line.a_width, line.b_width);
        if (path.polyline.points.empty()) {
            path.polyline.append(line.a);
            path.polyline.append(line.b);
            // Convert from spacing to extrusion width based on the extrusion model
            // of a square extrusion ended with semi circles.
            flow.width = unscale<float>(w) + flow.height * float(1. - 0.25 * PI);
            #ifdef SLIC3R_DEBUG
            printf("  filling %f gap\n", flow.width);
            #endif
            path.mm3_per_mm  = flow.mm3_per_mm();
            path.width       = flow.width;
            path.height      = flow.height;
        } else {
            thickness_delta = fabs(scale_(flow.width) - w);
            if (thickness_delta <= tolerance) {
                // the width difference between this line and the current flow width is 
                // within the accepted tolerance
                path.polyline.append(line.b);
            } else {
                // we need to initialize a new line
                paths.emplace_back(std::move(path));
                path = ExtrusionPath(role);
                -- i;
            }
        }
    }
    if (path.polyline.is_valid())
        paths.emplace_back(std::move(path));
    return paths;
}

static void variable_width(const ThickPolylines& polylines, ExtrusionRole role, Flow flow, std::vector<ExtrusionEntity*> &out)
{
	// This value determines granularity of adaptive width, as G-code does not allow
	// variable extrusion within a single move; this value shall only affect the amount
	// of segments, and any pruning shall be performed before we apply this tolerance.
	const float tolerance = float(scale_(0.05));
	for (const ThickPolyline &p : polylines) {
		ExtrusionPaths paths = thick_polyline_to_extrusion_paths(p, role, flow, tolerance);
		// Append paths to collection.
		if (! paths.empty()) {
			if (paths.front().first_point() == paths.back().last_point())
				out.emplace_back(new ExtrusionLoop(std::move(paths)));
			else {
				for (ExtrusionPath &path : paths)
					out.emplace_back(new ExtrusionPath(std::move(path)));
			}
		}
	}
}

// Hierarchy of perimeters.
class PerimeterGeneratorLoop {
public:
    // Polygon of this contour.
    Polygon polygon;
    // Is it a contour or a hole?
    // Contours are CCW oriented, holes are CW oriented.
    bool is_contour;
    // Depth in the hierarchy. External perimeter has depth = 0. An external perimeter could be both a contour and a hole.
    unsigned short depth;
    // Children contour, may be both CCW and CW oriented (outer contours or holes).
    std::vector<PerimeterGeneratorLoop> children;
    
    PerimeterGeneratorLoop(Polygon polygon, unsigned short depth, bool is_contour) : 
        polygon(polygon), is_contour(is_contour), depth(depth) {}
    // External perimeter. It may be CCW or CW oriented (outer contour or hole contour).
    bool is_external() const { return this->depth == 0; }
    // An island, which may have holes, but it does not have another internal island.
    bool is_internal_contour() const {
	    // An internal contour is a contour containing no other contours
	    if (! this->is_contour)
	        return false;
	    for (const PerimeterGeneratorLoop &loop : this->children)
	        if (loop.is_contour)
	            return false;
	    return true;
	}
};

typedef std::vector<PerimeterGeneratorLoop> PerimeterGeneratorLoops;

// Nest contours and holes in the order from outmost to the inner most.
static inline PerimeterGeneratorLoops nest_loops(std::vector<PerimeterGeneratorLoops> &&contours, std::vector<PerimeterGeneratorLoops> &&holes, const int num_loops)
{
	PerimeterGeneratorLoops out;

	if (! contours.empty()) 
	{
	    // nest loops: holes first
	    for (int depth = 0; depth < num_loops; ++ depth) {
	        PerimeterGeneratorLoops &holes_d = holes[depth];
	        // loop through all holes having depth == depth
	        for (int i = 0; i < (int)holes_d.size(); ++ i) {
	            PerimeterGeneratorLoop &loop = holes_d[i];
	            // find the hole loop that contains this one, if any
	            for (int t = depth + 1; t < num_loops; ++ t)
	                for (PerimeterGeneratorLoop &candidate_parent : holes[t])
	                    if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
	                        candidate_parent.children.emplace_back(std::move(loop));
	                        holes_d[i] = std::move(holes_d.back());
	                        holes_d.pop_back();
	                        -- i;
	                        goto end_loop;
	                    }
	            // if no hole contains this hole, find the contour loop that contains it
	            for (int t = num_loops - 1; t >= 0; -- t)
	                for (PerimeterGeneratorLoop &candidate_parent : contours[t])
	                    if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
	                        candidate_parent.children.emplace_back(std::move(loop));
	                        holes_d.pop_back();
	                        -- i;
	                        goto end_loop;
	                    }
	        end_loop:
	        	;
	        }
	    }
	    // nest contour loops
	    for (int depth = num_loops - 1; depth >= 1; -- depth) {
	        PerimeterGeneratorLoops &contours_d = contours[depth];
	        // loop through all contours having depth == depth
	        for (int i = 0; i < (int)contours_d.size(); ++ i) {
	            const PerimeterGeneratorLoop &loop = contours_d[i];
	            // find the contour loop that contains it
	            for (int t = depth - 1; t >= 0; -- t)
	                for (PerimeterGeneratorLoop &candidate_parent : contours[t])
	                    if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
	                        candidate_parent.children.emplace_back(std::move(loop));
	                        contours_d.pop_back();
	                        -- i;
	                        goto end_loop2;
	                    }
	        end_loop2:
	        	;
	        }
	    }
    	out = std::move(contours.front());
    }

    return out;
}

static ExtrusionEntityCollection traverse_loops(const PerimeterGenerator &perimeter_generator, const PerimeterGeneratorLoops &loops, ThickPolylines &thin_walls)
{
    // loops is an arrayref of ::Loop objects
    // turn each one into an ExtrusionLoop object
    ExtrusionEntityCollection coll;
    for (const PerimeterGeneratorLoop &loop : loops) {
        bool is_external = loop.is_external();
        
        ExtrusionRole role;
        ExtrusionLoopRole loop_role;
        role = is_external ? erExternalPerimeter : erPerimeter;
        if (loop.is_internal_contour()) {
            // Note that we set loop role to ContourInternalPerimeter
            // also when loop is both internal and external (i.e.
            // there's only one contour loop).
            loop_role = elrContourInternalPerimeter;
        } else {
            loop_role = elrDefault;
        }
        
        // detect overhanging/bridging perimeters
        ExtrusionPaths paths;
        if (perimeter_generator.config->overhangs && perimeter_generator.layer_id > 0
            && !(perimeter_generator.object_config->support_material && perimeter_generator.object_config->support_material_contact_distance.value == 0)) {
            // get non-overhang paths by intersecting this loop with the grown lower slices
            extrusion_paths_append(
                paths,
                intersection_pl(loop.polygon, perimeter_generator.lower_slices_polygons()),
                role,
                is_external ? perimeter_generator.ext_mm3_per_mm()          : perimeter_generator.mm3_per_mm(),
                is_external ? perimeter_generator.ext_perimeter_flow.width  : perimeter_generator.perimeter_flow.width,
                (float)perimeter_generator.layer_height);
            
            // get overhang paths by checking what parts of this loop fall 
            // outside the grown lower slices (thus where the distance between
            // the loop centerline and original lower slices is >= half nozzle diameter
            extrusion_paths_append(
                paths,
                diff_pl(loop.polygon, perimeter_generator.lower_slices_polygons()),
                erOverhangPerimeter,
                perimeter_generator.mm3_per_mm_overhang(),
                perimeter_generator.overhang_flow.width,
                perimeter_generator.overhang_flow.height);
            
            // Reapply the nearest point search for starting point.
            // We allow polyline reversal because Clipper may have randomly reversed polylines during clipping.
            chain_and_reorder_extrusion_paths(paths, &paths.front().first_point());
        } else {
            ExtrusionPath path(role);
            path.polyline   = loop.polygon.split_at_first_point();
            path.mm3_per_mm = is_external ? perimeter_generator.ext_mm3_per_mm()          : perimeter_generator.mm3_per_mm();
            path.width      = is_external ? perimeter_generator.ext_perimeter_flow.width  : perimeter_generator.perimeter_flow.width;
            path.height     = (float)perimeter_generator.layer_height;
            paths.push_back(path);
        }
        
        coll.append(ExtrusionLoop(std::move(paths), loop_role));
    }
    
    // Append thin walls to the nearest-neighbor search (only for first iteration)
    if (! thin_walls.empty()) {
        variable_width(thin_walls, erExternalPerimeter, perimeter_generator.ext_perimeter_flow, coll.entities);
        thin_walls.clear();
    }
    
    // Traverse children and build the final collection.
	Point zero_point(0, 0);
	std::vector<std::pair<size_t, bool>> chain = chain_extrusion_entities(coll.entities, &zero_point);
    ExtrusionEntityCollection out;
    for (const std::pair<size_t, bool> &idx : chain) {
		assert(coll.entities[idx.first] != nullptr);
        if (idx.first >= loops.size()) {
            // This is a thin wall.
			out.entities.reserve(out.entities.size() + 1);
            out.entities.emplace_back(coll.entities[idx.first]);
			coll.entities[idx.first] = nullptr;
            if (idx.second)
				out.entities.back()->reverse();
        } else {
            const PerimeterGeneratorLoop &loop = loops[idx.first];
            assert(thin_walls.empty());
            ExtrusionEntityCollection children = traverse_loops(perimeter_generator, loop.children, thin_walls);
            out.entities.reserve(out.entities.size() + children.entities.size() + 1);
            ExtrusionLoop *eloop = static_cast<ExtrusionLoop*>(coll.entities[idx.first]);
            coll.entities[idx.first] = nullptr;
            if (loop.is_contour) {
                eloop->make_counter_clockwise();
                out.append(std::move(children.entities));
                out.entities.emplace_back(eloop);
            } else {
                eloop->make_clockwise();
                out.entities.emplace_back(eloop);
                out.append(std::move(children.entities));
            }
        }
    }
    return out;
}

#if 1
static ClipperLib::Paths offset_contour_miter(const ClipperLib::Path &input, const double delta, const double miter_limit)
{
    ClipperLib::ClipperOffset co;
    co.MiterLimit = miter_limit;
    co.ShortestEdgeLength = double(std::abs(delta * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR));
    co.AddPath(input, jtMiter, ClipperLib::etClosedPolygon);
    ClipperLib::Paths out;
    co.Execute(out, delta);
    return out;
}

static ClipperLib::Paths offset_contours_miter(const ClipperLib::Paths &input_paths, const double delta, const double miter_limit)
{
    ClipperLib::Paths out;
    for (const ClipperLib::Path& input_path : input_paths)
        append(out, offset_contour_miter(input_path, delta, miter_limit));
    return out;
}

static ClipperLib::Paths unite_contours(ClipperLib::Paths &&input)
{
    if (input.size() > 1) {
    	// Trim the holes one by the other.
        ClipperLib::Clipper clipper;
        clipper.AddPaths(input, ClipperLib::ptSubject, true);
        ClipperLib::Paths out;
        clipper.Execute(ClipperLib::ctUnion, out, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        input = std::move(out);
    }
    return input;
}

static ClipperLib::Paths subtract_holes(ClipperLib::Paths &&input_contours, ClipperLib::Paths &input_holes)
{
    if (input_holes.size() > 1) {
    	// Trim the holes one by the other.
        ClipperLib::Clipper clipper;
        clipper.AddPaths(input_contours, ClipperLib::ptSubject, true);
        clipper.AddPaths(input_holes, ClipperLib::ptClip, true);
        ClipperLib::Paths out;
        clipper.Execute(ClipperLib::ctDifference, out, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        input_contours = std::move(out);
    }
    return input_contours;
}

static ClipperLib::Paths subtract_holes(const ClipperLib::Paths &input_contours, ClipperLib::Paths &input_holes)
{
    ClipperLib::Paths out = input_contours;
    return subtract_holes(std::move(out), input_holes);
}

ClipperLib::PolyTree clip_holes(const ClipperLib::Paths &input_contours, const ClipperLib::Paths &input_holes)
{
    ClipperLib::Clipper clipper;
    clipper.AddPaths(input_holes, 	 ClipperLib::ptSubject, true);
    clipper.AddPaths(input_contours, ClipperLib::ptClip,    true);
    ClipperLib::PolyTree retval;
    clipper.Execute(ClipperLib::ctDifference, retval, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return retval;
}

// Perimeter tree.
// Outmost Perimeter contains the source contours (non-printable) and possibly the gaps between the 1st perimeter and the source contours
// (aka thin walls, if the thin walls are enabled).
struct PerimeterNesting
{
	// Outer contour of this region. It may be an extrusion centerline or a region (extrusion) separator.
	// Both the contours and holes are CCW oriented, but the holes are marked with is_hole.
	ClipperLib::Path 				contour;
	// Depth of this contour. Even - separation contours & gap fills, odd: perimeters and open lines.
	unsigned int 					depth = 0;
	// Is this contour an outer contour or a hole? Both contours and holes are CCW oriented.
	bool 							is_hole = false;
	// Childrens of this region, that is both children contours and holes.
	std::vector<PerimeterNesting> 	children;
	// Open lines created as centerlines of holes trimmed by the inner boundary of the outer contour extrusion.
	ClipperLib::Paths 				open_lines;
	// Gap fill between the parent perimeters and this perimeters.
	// Gap fill shall only be filled in for even depth.
	ThickPolylines 					gap_fill;

    // External perimeter. Both the outer contours and holes are CCW oriented.
    bool is_external_perimeter() const {
    	// Call it on centerline contours (aka perimeters) only, not on contours separating regions.
    	assert(this->depth & 1);
    	return this->depth == 1;
    }
    // A contour, which may have holes, but it does not have another internal island.
    bool is_innermost_perimeter() const {
    	assert(this->depth & 1);
	    // An internal contour is a contour containing no other contours
	    if (this->is_hole)
	        return false;
	    for (const PerimeterNesting &child : this->children)
	        if (!child.is_hole)
	            return false;
	    return true;
	}

#ifndef NDEBUG
	bool validate();
#endif /* NDEBUG */
private:
	bool validate_recursive_even(const PerimeterNesting* const parent);
};

#ifndef NDEBUG
bool PerimeterNesting::validate()
{
	assert(this->depth == 0);
	assert(this->validate_recursive_even(nullptr));
	// Everything is fine.
	return true;
}
bool PerimeterNesting::validate_recursive_even(const PerimeterNesting* const parent)
{
	// The contour separating perimeter regions is non-empty.
	assert(! this->contour.empty());
	if (parent != nullptr) {
		if (parent->is_hole) {
			assert(this->is_hole);
		} else {
			// This contour is a contour or a hole, parent is not a hole.
//			assert(parent->);
		}
	}
	// Everything is fine.
	return true;
}
#endif /* NDEBUG */

using PerimetersNesting = std::vector<PerimeterNesting>;

// Convert the expolygons into the outmost Perimeters.
PerimetersNesting expolygons_to_perimeters(const ExPolygons &expolygons)
{
	PerimetersNesting out;
	out.reserve(expolygons.size());
	for (const ExPolygon &expoly : expolygons) {
		out.emplace_back(PerimeterNesting{ Slic3rMultiPoint_to_ClipperPath_scaled(expoly.contour), 0, false });
        PerimeterNesting &p = out.back();
        p.children.reserve(expoly.holes.size());
        for (const Polygon &hole_src : expoly.holes)
            p.children.emplace_back(PerimeterNesting{ Slic3rMultiPoint_to_ClipperPath_scaled_reversed(hole_src), 0, true});
    }
    return out;
}

template<class InputIt, class OutputIt, class UnaryPredicate>
inline OutputIt move_if_clear(InputIt first, InputIt last, OutputIt d_first, UnaryPredicate pred)
{
    while (first != last) {
        if (pred(*first)) {
            *d_first++ = std::move(*first);
            first->clear();
        }
        first++;
    }
    return d_first;
}

ThickPolylines gap_fill(ClipperLib::Paths in, const double perimeter_width, const double perimeter_spacing)
{
    double min = 0.2 * perimeter_width * (1 - INSET_OVERLAP_TOLERANCE);
    double max = 2. * perimeter_spacing;
#if 0
    // collapse 
    ExPolygons gaps_ex = diff_ex(
        //FIXME offset2 would be enough and cheaper.
        offset2_ex(gaps, - float(min / 2.), float(min / 2.)),
        offset2_ex(gaps, - float(max / 2.), float(max / 2.)),
        true);
#endif

    ThickPolylines out;
#if 0
    unscaleClipperPolygons(in);
    for (const ExPolygon &ex : ClipperPaths_to_Slic3rExPolygons(in))
        ex.medial_axis(max, min, &out);
#endif
   	return out;
}

// Offsetting an input region inside by step1 (centerline) and step2 (line thickness).
// Returns the inner contours (regions) with the extruded paths removed.
void single_perimeter_step(PerimeterNesting &parent, float step1, float step2)
{
	double miter_limit = 3.;
    // Offset the outer contour by step1. This offset contour will be the outer perimeter.
    const float step1_scaled = step1 * float(CLIPPER_OFFSET_SCALE);
    ClipperLib::Paths contours = offset_contour_miter(parent.contour, - step1_scaled, miter_limit);
    // Offset the holes one by one by step1, unite the holes. These holes will be the candidates for new perimeters, 
    // but they need to be clipped yet by outer contours.
    ClipperLib::Paths holes;
    ClipperLib::Paths centerlines_outer;
    {
	    ClipperLib::Paths holes_initial;
        holes.reserve(parent.children.size());
        for (const PerimeterNesting &hole_src : parent.children)
        	if (hole_src.is_hole && hole_src.depth == parent.depth) {
	            append(holes, 		  offset_contour_miter(hole_src.contour, step1_scaled,		  miter_limit));
//	            append(holes_initial, offset_contour_miter(hole_src.contour, step1_scaled / 10.f, miter_limit));
	            holes_initial.emplace_back(hole_src.contour);
        	}
		holes = unite_contours(std::move(holes));
	    // Clip the outer centerlines by the holes offsetted
		centerlines_outer = subtract_holes(contours, std::move(holes_initial));
	    // Calculate outer gap fill areas.
	    parent.gap_fill = gap_fill(subtract_holes(contours, offset_contour_miter(parent.contour, step1_scaled + 10, miter_limit)), 2. * step1, 2. * step1);
	    append(parent.gap_fill, gap_fill(subtract_holes(offset_contour_miter(parent.contour, - step1_scaled - 10, miter_limit), holes_initial), 2. * step1, 2. * step1));
    }

    // Offset the contours one by one by step2.
    const float step2_scaled = step2 * float(CLIPPER_OFFSET_SCALE);
    ClipperLib::Paths contours2 = offset_contours_miter(contours, - step2_scaled, miter_limit);
    // Offset the holes one by one by step2.
    ClipperLib::Paths holes2 = offset_contours_miter(holes, step2_scaled, miter_limit);

    // Clip the holes with contours2. This may create some open contours.
    ClipperLib::PolyTree centerlines_inner = clip_holes(contours2, holes);

    // Subtract final holes from the final contours. This is the area covered by the extrusions.
    ClipperLib::Paths final_boundaries_contours = subtract_holes(std::move(contours2), std::move(holes2));
    // Partition final_boundaries_contours to final_boundaries_contours / final_boundaries_holes based on the contour orientation.
    ClipperLib::Paths final_boundaries_holes;
    final_boundaries_holes.reserve(final_boundaries_contours.size());
	move_if_clear(final_boundaries_contours.begin(), final_boundaries_contours.end(),
	    std::back_inserter(final_boundaries_holes), [](const ClipperLib::Path &path){ return ClipperLib::Area(path) < 0.; });
	final_boundaries_contours.erase(
		std::remove_if(final_boundaries_contours.begin(), final_boundaries_contours.end(), [](const ClipperLib::Path &path){ return path.empty(); }),
        final_boundaries_contours.end());
	for (ClipperLib::Path &hole : final_boundaries_holes)
		// Orient the holes CCW.
		std::reverse(hole.begin(), hole.end());

    // Sort out the centerlines_outer / final_boundaries / centerlines_inner into a nested graph.
    PerimetersNesting src_perimeters_holes = std::move(parent.children);
    parent.children.clear();
    parent.children.reserve(centerlines_outer.size());
    for (ClipperLib::Path &centerline_outer_src : centerlines_outer)
    	if (! centerline_outer_src.empty()) {
    		// Add the centerline to the output.
    		parent.children.emplace_back(PerimeterNesting{ std::move(centerline_outer_src), parent.depth + 1, false });
    		// Distribute the final extrusion boundary contours among the centerlines.
    		PerimeterNesting	&centerline_outer      = parent.children.back();
	    	ClipperBoundingBox   centerline_outer_bbox = get_extents(centerline_outer.contour);
	    	auto                 inside_centerline     = [&centerline_outer, &centerline_outer_bbox](const ClipperLib::Path &path) {
    			return ! path.empty() && centerline_outer_bbox.contains(Vec2i64(path.front().X, path.front().Y)) && ClipperLib::PointInPolygon(path.front(), centerline_outer.contour);
	    	};
	    	for (ClipperLib::Path &final_boundary_contour_src : final_boundaries_contours)
    			if (inside_centerline(final_boundary_contour_src)) {
    				centerline_outer.children.emplace_back(PerimeterNesting{ std::move(final_boundary_contour_src), parent.depth + 2, false });
    				final_boundary_contour_src.clear();
    				// Distribute the final_boundary_holes accross the final_boundaries_contours.
                    PerimeterNesting    &final_boundary_contour        = centerline_outer.children.back();
			    	ClipperBoundingBox   final_boundary_contour_bbox   = get_extents(final_boundary_contour.contour);
			    	auto                 inside_final_boundary_contour = [&final_boundary_contour, &final_boundary_contour_bbox](const ClipperLib::Path &path) {
		    			return ! path.empty() && final_boundary_contour_bbox.contains(Vec2i64(path.front().X, path.front().Y)) && ClipperLib::PointInPolygon(path.front(), final_boundary_contour.contour);
			    	};
			    	for (ClipperLib::Path &final_boundary_hole_src : final_boundaries_holes)
		    			if (inside_final_boundary_contour(final_boundary_hole_src)) {
		    				final_boundary_contour.children.emplace_back(PerimeterNesting{ std::move(final_boundary_hole_src), parent.depth + 2, true });
		    				final_boundary_hole_src.clear();
		    				// Distribute the final_boundary_holes accross the final_boundaries_contours.
                            PerimeterNesting    &final_boundary_hole        = final_boundary_contour.children.back();
					    	ClipperBoundingBox   final_boundary_hole_bbox   = get_extents(final_boundary_contour.contour);
					    	auto                 inside_final_boundary_hole = [&final_boundary_hole, &final_boundary_hole_bbox](const ClipperLib::Path &path) {
				    			return ! path.empty() && final_boundary_hole_bbox.contains(Vec2i64(path.front().X, path.front().Y)) && ClipperLib::PointInPolygon(path.front(), final_boundary_hole.contour);
					    	};
					    	for (size_t i = 0; i < centerlines_inner.Childs.size(); ++ i) {
				    			ClipperLib::Path &hole = centerlines_inner.Childs[i]->Contour;
				    			if (! hole.empty() && ! centerlines_inner.Childs[i]->IsOpen() && inside_final_boundary_hole(hole)) {
				    				final_boundary_hole.children.emplace_back(PerimeterNesting{ std::move(hole), parent.depth + 1, true });
				    				hole.clear();
				    				// Distribute the source holes accross the final_boundaries_contours.
                                    PerimeterNesting    &centerline_inner        = final_boundary_hole.children.back();
							    	ClipperBoundingBox   centerline_inner_bbox   = get_extents(centerline_inner.contour);
							    	auto                 inside_centerline_inner = [&centerline_inner, &centerline_inner_bbox](const ClipperLib::Path &path) {
						    			return ! path.empty() && centerline_inner_bbox.contains(Vec2i64(path.front().X, path.front().Y)) && ClipperLib::PointInPolygon(path.front(), centerline_inner.contour);
							    	};
    						    	for (PerimeterNesting &src_perimeter_hole : src_perimeters_holes)
    						    		if (inside_centerline_inner(src_perimeter_hole.contour)) {
						    				centerline_inner.children.emplace_back(std::move(src_perimeter_hole));
						    				centerline_inner.contour.clear();
    						    		}
				    			}
				    		}
		    			}
    			}
	    	for (PerimeterNesting &src_perimeter_hole : src_perimeters_holes)
	    		if (inside_centerline(src_perimeter_hole.contour)) {
    				centerline_outer.children.emplace_back(std::move(src_perimeter_hole));
    				src_perimeter_hole.contour.clear();
	    		}
    	}
	for (size_t i = 0; i < centerlines_inner.Childs.size(); ++ i) {
		ClipperLib::Path &open_line = centerlines_inner.Childs[i]->Contour;
		if (! open_line.empty() && centerlines_inner.Childs[i]->IsOpen()) {
			parent.open_lines.emplace_back(std::move(open_line));
            open_line.clear();
		}
	}
	for (PerimeterNesting &src_perimeter_hole : src_perimeters_holes)
		if (! src_perimeter_hole.contour.empty()) {
			parent.children.emplace_back(std::move(src_perimeter_hole));
			src_perimeter_hole.contour.clear();
		}

    // Unscale the output.
//    unscaleClipperPolygons(final);
//    return ClipperPaths_to_Slic3rExPolygons(final);
}

void single_perimeter_step_recursive(PerimeterNesting &parent, float perimeter_spacing, size_t depth)
{
	single_perimeter_step(parent, perimeter_spacing * 0.5f, perimeter_spacing * 0.5f);
	if (-- depth == 0)
		return;
	for (PerimeterNesting &nested : parent.children)
		if (! nested.is_hole)
			for (PerimeterNesting &nested2 : nested.children)
				if (! nested2.is_hole)
					single_perimeter_step_recursive(nested2, perimeter_spacing, depth);
}

PerimetersNesting create_perimeters(const ExPolygons &expolygons, 
	const float external_perimeter_width, 
	const float external_perimeter_spacing, 
	const float perimeter_width,
	const float perimeter_spacing,
	const size_t num_perimeters)
{
	PerimetersNesting topmost = expolygons_to_perimeters(expolygons);
	if (num_perimeters > 0) {
		for (PerimeterNesting &parent : topmost) {
			single_perimeter_step(parent, external_perimeter_width * 0.5f, external_perimeter_spacing * 0.5f);
			if (num_perimeters > 1) {
				for (PerimeterNesting &nested : parent.children)
					if (! nested.is_hole)
						for (PerimeterNesting &nested2 : nested.children)
							if (! nested2.is_hole)
								single_perimeter_step_recursive(nested2, perimeter_spacing, num_perimeters - 1);
			}
		}
	}
	return topmost;
}

void collect_infill_areas_recursive(const PerimeterNesting &parent, ExPolygons &out)
{
	assert(! parent.is_hole);
	if (parent.open_lines.empty() && parent.gap_fill.empty() && ! parent.contour.empty()) {
		bool all_holes = true;
		for (const PerimeterNesting &child : parent.children)
			if (! child.is_hole || child.depth != parent.depth) {
				all_holes = false;
				break;
			}
		if (all_holes) {
			ExPolygon expoly;
			ClipperLib::Path p = parent.contour;
			unscaleClipperPolygon(p);
			expoly.contour = ClipperPath_to_Slic3rPolygon(p);
			expoly.holes.reserve(parent.children.size());
			for (const PerimeterNesting &child : parent.children) {
				ClipperLib::Path p = child.contour;
				std::reverse(p.begin(), p.end());
				unscaleClipperPolygon(p);
				expoly.holes.emplace_back(ClipperPath_to_Slic3rPolygon(p));
			}
			return;
		}
	}
	for (const PerimeterNesting &child : parent.children)
		if (! child.is_hole)
			for (const PerimeterNesting &child2 : child.children)
				if (! child2.is_hole)
					collect_infill_areas_recursive(child2, out);
}

ExPolygons collect_infill_areas(const PerimetersNesting &outmost)
{
	ExPolygons out;
	for (const PerimeterNesting &parent : outmost) {
		assert(! parent.is_hole);
		collect_infill_areas_recursive(parent, out);
	}
	return out;
}

static ExtrusionEntityCollection collect_extrusions(const PerimeterGenerator &perimeter_generator, const PerimetersNesting &input_regions)
{
    ExtrusionEntityCollection 			 extrusion_entity_collection;
    std::vector<const PerimeterNesting*> extrusion_entity_collection_source;

    auto extrude_perimeter = [&extrusion_entity_collection, &extrusion_entity_collection_source, &perimeter_generator](const PerimeterNesting &child)
    {
        bool 				is_external = child.is_external_perimeter();
        ExtrusionRole 		role        = is_external ? erExternalPerimeter : erPerimeter;
        ExtrusionLoopRole 	loop_role   = child.is_innermost_perimeter() ? 
            // Note that we set loop role to ContourInternalPerimeter
            // also when loop is both internal and external (i.e.
            // there's only one contour loop).
            elrContourInternalPerimeter :
			elrDefault;
	    Polygon perimeter = ClipperPath_to_Slic3rPolygon_unscale(child.contour);
	    // detect overhanging/bridging perimeters
        ExtrusionPaths paths;
        if (perimeter_generator.config->overhangs && perimeter_generator.layer_id > 0
            && !(perimeter_generator.object_config->support_material && perimeter_generator.object_config->support_material_contact_distance.value == 0)) {
            // get non-overhang paths by intersecting this loop with the grown lower slices
            extrusion_paths_append(
                paths,
                intersection_pl(perimeter, perimeter_generator.lower_slices_polygons()),
                role,
                is_external ? perimeter_generator.ext_mm3_per_mm()          : perimeter_generator.mm3_per_mm(),
                is_external ? perimeter_generator.ext_perimeter_flow.width  : perimeter_generator.perimeter_flow.width,
                (float)perimeter_generator.layer_height);
            
            // get overhang paths by checking what parts of this loop fall 
            // outside the grown lower slices (thus where the distance between
            // the loop centerline and original lower slices is >= half nozzle diameter
            extrusion_paths_append(
                paths,
                diff_pl(perimeter, perimeter_generator.lower_slices_polygons()),
                erOverhangPerimeter,
                perimeter_generator.mm3_per_mm_overhang(),
                perimeter_generator.overhang_flow.width,
                perimeter_generator.overhang_flow.height);
            
            // Reapply the nearest point search for starting point.
            // We allow polyline reversal because Clipper may have randomly reversed polylines during clipping.
            chain_and_reorder_extrusion_paths(paths, &paths.front().first_point());
        } else {
            ExtrusionPath path(role);
            path.polyline   = perimeter.split_at_first_point();
            path.mm3_per_mm = is_external ? perimeter_generator.ext_mm3_per_mm()          : perimeter_generator.mm3_per_mm();
            path.width      = is_external ? perimeter_generator.ext_perimeter_flow.width  : perimeter_generator.perimeter_flow.width;
            path.height     = (float)perimeter_generator.layer_height;
            paths.push_back(path);
        }
        
        extrusion_entity_collection.append(ExtrusionLoop(std::move(paths), loop_role));
        extrusion_entity_collection_source.emplace_back(&child);
    };

    auto extrude_open_lines = [&extrusion_entity_collection, &extrusion_entity_collection_source, &perimeter_generator](const PerimeterNesting &parent)
    {
        bool detect_overhangs = perimeter_generator.config->overhangs && perimeter_generator.layer_id > 0 && 
            !(perimeter_generator.object_config->support_material && perimeter_generator.object_config->support_material_contact_distance.value == 0);
        for (Polyline &open_line : ClipperPaths_to_Slic3rPolylines_unscale(parent.open_lines)) {
	        // detect overhanging/bridging perimeters
            ExtrusionPaths paths;
            if (detect_overhangs) {
                // get non-overhang paths by intersecting this loop with the grown lower slices
                extrusion_paths_append(
                    paths,
                    intersection_pl(open_line, perimeter_generator.lower_slices_polygons()),
                    erPerimeter,
                    perimeter_generator.mm3_per_mm(),
                    perimeter_generator.perimeter_flow.width,
                    (float)perimeter_generator.layer_height);
            
                // get overhang paths by checking what parts of this loop fall 
                // outside the grown lower slices (thus where the distance between
                // the loop centerline and original lower slices is >= half nozzle diameter
                extrusion_paths_append(
                    paths,
                    diff_pl(open_line, perimeter_generator.lower_slices_polygons()),
                    erOverhangPerimeter,
                    perimeter_generator.mm3_per_mm_overhang(),
                    perimeter_generator.overhang_flow.width,
                    perimeter_generator.overhang_flow.height);
            
                // Reapply the nearest point search for starting point.
                // We allow polyline reversal because Clipper may have randomly reversed polylines during clipping.
                chain_and_reorder_extrusion_paths(paths, &paths.front().first_point());
            } else {
                ExtrusionPath path(erPerimeter);
                path.polyline   = std::move(open_line);
                path.mm3_per_mm = perimeter_generator.mm3_per_mm();
                path.width      = perimeter_generator.perimeter_flow.width;
                path.height     = (float)perimeter_generator.layer_height;
                paths.emplace_back(std::move(path));
            }
            if (paths.size() == 1)
                extrusion_entity_collection.append(ExtrusionPath(std::move(paths.front())));
            else
                extrusion_entity_collection.append(ExtrusionMultiPath(std::move(paths)));
            extrusion_entity_collection_source.emplace_back(nullptr);
        }
    };

    for (const PerimeterNesting &parent : input_regions) {
	    if (parent.is_hole) {
	    	// Hole may only contain holes, never outer contours.
		    for (const PerimeterNesting &child : parent.children) {
		    	assert(child.is_hole);
	    		assert(! child.contour.empty());
		    	assert((parent.depth & 1) == 0);
		    	assert(parent.depth > 0);
		    	// Child has depth one lower than its parent.
		    	assert(parent.depth + 1 == child.depth);
	    		// This must be a separation contour.
	    		assert((child.depth & 1) == 0);
	    		assert(child.depth < parent.depth);
			    // Append thin walls between the hole perimetes and this area to the nearest-neighbor search.
			    if (! child.gap_fill.empty()) {
			        variable_width(child.gap_fill, erPerimeter, perimeter_generator.perimeter_flow, extrusion_entity_collection.entities);
			        extrusion_entity_collection_source.resize(extrusion_entity_collection.entities.size(), nullptr);
			    }
	    		for (const PerimeterNesting &child2 : child.children) {
		    		assert(child2.depth & 1);
		    		assert(child2.depth + 1 == child.depth);
	    		    extrude_perimeter(child);
                }
		    }
	    } else {
	    	// An outer contour may contain outer perimeters and hole boundaries.
		    for (const PerimeterNesting &child : parent.children) {
		    	assert((parent.depth & 1) == 0);
		    	if (child.is_hole) {
		    		assert(! child.contour.empty());
		    		// This must be a separation contour.
		    		assert((child.depth & 1) == 0);
		    		assert(child.depth <= parent.depth);
				    // Append thin walls between the hole perimetes and this area to the nearest-neighbor search.
				    if (! child.gap_fill.empty()) {
				        variable_width(child.gap_fill, erPerimeter, perimeter_generator.perimeter_flow, extrusion_entity_collection.entities);
				        extrusion_entity_collection_source.resize(extrusion_entity_collection.entities.size(), nullptr);
				    }
		    		for (const PerimeterNesting &child2 : child.children) {
			    		assert(child2.depth & 1);
			    		assert(child2.depth + 1 == child.depth);
		    			extrude_perimeter(child2);
		    		}
		    	} else {
		    		// This must be a perimeter contour.
		    		assert(child.depth & 1);
		    		assert(child.depth == parent.depth + 1);
		    		extrude_perimeter(child);
		    	}
		    }
	    }
	    // Append thin walls to the nearest-neighbor search.
	    if (! parent.gap_fill.empty()) {
	        variable_width(parent.gap_fill, erExternalPerimeter, perimeter_generator.ext_perimeter_flow, extrusion_entity_collection.entities);
	        extrusion_entity_collection_source.resize(extrusion_entity_collection.entities.size(), nullptr);
	    }
        extrude_open_lines(parent);
    }

    // Traverse children and build the final collection.
	Point zero_point(0, 0);
	std::vector<std::pair<size_t, bool>> chain = chain_extrusion_entities(extrusion_entity_collection.entities, &zero_point);
    ExtrusionEntityCollection out;
    for (const std::pair<size_t, bool> &idx : chain) {
		assert(extrusion_entity_collection.entities[idx.first] != nullptr);
        const PerimeterNesting *perimeter = extrusion_entity_collection_source[idx.first];
        if (perimeter == nullptr) {
            // This is a thin wall.
			out.entities.reserve(out.entities.size() + 1);
            out.entities.emplace_back(extrusion_entity_collection.entities[idx.first]);
			extrusion_entity_collection.entities[idx.first] = nullptr;
            if (idx.second)
				out.entities.back()->reverse();
        } else {
            ExtrusionEntityCollection children = collect_extrusions(perimeter_generator, perimeter->children);
            out.entities.reserve(out.entities.size() + children.entities.size() + 1);
            ExtrusionLoop *eloop = static_cast<ExtrusionLoop*>(extrusion_entity_collection.entities[idx.first]);
            extrusion_entity_collection.entities[idx.first] = nullptr;
            if (perimeter->is_hole) {
                out.entities.emplace_back(eloop);
                out.append(std::move(children.entities));
            } else {
                out.append(std::move(children.entities));
                out.entities.emplace_back(eloop);
            }
            children.entities.clear();
        }
    }
    return out;
}
#endif

void PerimeterGenerator::process()
{
    // other perimeters
    m_mm3_per_mm               		= this->perimeter_flow.mm3_per_mm();
    coord_t perimeter_width         = this->perimeter_flow.scaled_width();
    coord_t perimeter_spacing       = this->perimeter_flow.scaled_spacing();
    
    // external perimeters
    m_ext_mm3_per_mm           		= this->ext_perimeter_flow.mm3_per_mm();
    coord_t ext_perimeter_width     = this->ext_perimeter_flow.scaled_width();
    coord_t ext_perimeter_spacing   = this->ext_perimeter_flow.scaled_spacing();
    coord_t ext_perimeter_spacing2  = this->ext_perimeter_flow.scaled_spacing(this->perimeter_flow);
    
    // overhang perimeters
    m_mm3_per_mm_overhang      		= this->overhang_flow.mm3_per_mm();
    
    // solid infill
    coord_t solid_infill_spacing    = this->solid_infill_flow.scaled_spacing();
    
    // Calculate the minimum required spacing between two adjacent traces.
    // This should be equal to the nominal flow spacing but we experiment
    // with some tolerance in order to avoid triggering medial axis when
    // some squishing might work. Loops are still spaced by the entire
    // flow spacing; this only applies to collapsing parts.
    // For ext_min_spacing we use the ext_perimeter_spacing calculated for two adjacent
    // external loops (which is the correct way) instead of using ext_perimeter_spacing2
    // which is the spacing between external and internal, which is not correct
    // and would make the collapsing (thus the details resolution) dependent on 
    // internal flow which is unrelated.
    coord_t min_spacing         = coord_t(perimeter_spacing      * (1 - INSET_OVERLAP_TOLERANCE));
    coord_t ext_min_spacing     = coord_t(ext_perimeter_spacing  * (1 - INSET_OVERLAP_TOLERANCE));
    bool    has_gap_fill 		= this->config->gap_fill_speed.value > 0;

    // prepare grown lower layer slices for overhang detection
    if (this->lower_slices != NULL && this->config->overhangs) {
        // We consider overhang any part where the entire nozzle diameter is not supported by the
        // lower layer, so we take lower slices and offset them by half the nozzle diameter used 
        // in the current layer
        double nozzle_diameter = this->print_config->nozzle_diameter.get_at(this->config->perimeter_extruder-1);
        m_lower_slices_polygons = offset(*this->lower_slices, float(scale_(+nozzle_diameter/2)));
    }
    
    // we need to process each island separately because we might have different
    // extra perimeters for each one
    for (const Surface &surface : this->slices->surfaces) {
        // detect how many perimeters must be generated for this island
        int        loop_number = this->config->perimeters + surface.extra_perimeters - 1;  // 0-indexed loops
        ExPolygons last        = union_ex(surface.expolygon.simplify_p(SCALED_RESOLUTION));

#if 0
        ExPolygons gaps;
        if (loop_number >= 0) {
            // In case no perimeters are to be generated, loop_number will equal to -1.
            std::vector<PerimeterGeneratorLoops> contours(loop_number+1);    // depth => loops
            std::vector<PerimeterGeneratorLoops> holes(loop_number+1);       // depth => loops
            ThickPolylines thin_walls;
            // we loop one time more than needed in order to find gaps after the last perimeter was applied
            for (int i = 0;; ++ i) {  // outer loop is 0
                // Calculate next onion shell of perimeters.
                ExPolygons offsets;
                if (i == 0) {
                    // the minimum thickness of a single loop is:
                    // ext_width/2 + ext_spacing/2 + spacing/2 + width/2
                    offsets = this->config->thin_walls ? 
                        offset2_ex(
                            last,
                            - float(ext_perimeter_width / 2. + ext_min_spacing / 2. - 1),
                            + float(ext_min_spacing / 2. - 1)) :
                        offset_ex(last, - float(ext_perimeter_width / 2.));
                    // look for thin walls
                    if (this->config->thin_walls) {
                        // the following offset2 ensures almost nothing in @thin_walls is narrower than $min_width
                        // (actually, something larger than that still may exist due to mitering or other causes)
                        coord_t min_width = coord_t(scale_(this->ext_perimeter_flow.nozzle_diameter / 3));
                        ExPolygons expp = offset2_ex(
                            // medial axis requires non-overlapping geometry
                            diff_ex(to_polygons(last),
                                    offset(offsets, float(ext_perimeter_width / 2.)),
                                    true),
                            - float(min_width / 2.), float(min_width / 2.));
                        // the maximum thickness of our thin wall area is equal to the minimum thickness of a single loop
                        for (ExPolygon &ex : expp)
                            ex.medial_axis(ext_perimeter_width + ext_perimeter_spacing2, min_width, &thin_walls);
                    }
                    if (print_config->spiral_vase && offsets.size() > 1) {
                    	// Remove all but the largest area polygon.
                    	keep_largest_contour_only(offsets);
                    }
                } else {
                    //FIXME Is this offset correct if the line width of the inner perimeters differs
                    // from the line width of the infill?
                    coord_t distance = (i == 1) ? ext_perimeter_spacing2 : perimeter_spacing;
                    offsets = this->config->thin_walls ?
                        // This path will ensure, that the perimeters do not overfill, as in 
                        // prusa3d/Slic3r GH #32, but with the cost of rounding the perimeters
                        // excessively, creating gaps, which then need to be filled in by the not very 
                        // reliable gap fill algorithm.
                        // Also the offset2(perimeter, -x, x) may sometimes lead to a perimeter, which is larger than
                        // the original.
                        offset2_ex(last,
                                - float(distance + min_spacing / 2. - 1.),
                                float(min_spacing / 2. - 1.)) :
                        // If "detect thin walls" is not enabled, this paths will be entered, which 
                        // leads to overflows, as in prusa3d/Slic3r GH #32
                        offset_ex(last, - float(distance));
                    // look for gaps
                    if (has_gap_fill)
                        // not using safety offset here would "detect" very narrow gaps
                        // (but still long enough to escape the area threshold) that gap fill
                        // won't be able to fill but we'd still remove from infill area
                        append(gaps, diff_ex(
                            offset(last,    - float(0.5 * distance)),
                            offset(offsets,   float(0.5 * distance + 10))));  // safety offset
                }
                if (offsets.empty()) {
                    // Store the number of loops actually generated.
                    loop_number = i - 1;
                    // No region left to be filled in.
                    last.clear();
                    break;
                } else if (i > loop_number) {
                    // If i > loop_number, we were looking just for gaps.
                    break;
                }
                for (const ExPolygon &expolygon : offsets) {
	                // Outer contour may overlap with an inner contour,
	                // inner contour may overlap with another inner contour,
	                // outer contour may overlap with itself.
	                //FIXME evaluate the overlaps, annotate each point with an overlap depth,
	                // compensate for the depth of intersection.
                    contours[i].emplace_back(PerimeterGeneratorLoop(expolygon.contour, i, true));
                    if (! expolygon.holes.empty()) {
                        holes[i].reserve(holes[i].size() + expolygon.holes.size());
                        for (const Polygon &hole : expolygon.holes)
                            holes[i].emplace_back(PerimeterGeneratorLoop(hole, i, false));
                    }
                }
                last = std::move(offsets);
                if (i == loop_number && (! has_gap_fill || this->config->fill_density.value == 0)) {
                	// The last run of this loop is executed to collect gaps for gap fill.
                	// As the gap fill is either disabled or not 
                	break;
                }
            }
            ExtrusionEntityCollection entities = traverse_loops(*this, nest_loops(std::move(contours), std::move(holes), loop_number + 1), thin_walls);
            // if brim will be printed, reverse the order of perimeters so that
            // we continue inwards after having finished the brim
            // TODO: add test for perimeter order
            if (this->config->external_perimeters_first || 
                (this->layer_id == 0 && this->print_config->brim_width.value > 0))
                entities.reverse();
            // append perimeters for this slice as a collection
            if (! entities.empty())
                this->loops->append(entities);
        } // for each loop of an island

        // fill gaps
        if (! gaps.empty()) {
            // collapse 
            double min = 0.2 * perimeter_width * (1 - INSET_OVERLAP_TOLERANCE);
            double max = 2. * perimeter_spacing;
            ExPolygons gaps_ex = diff_ex(
                //FIXME offset2 would be enough and cheaper.
                offset2_ex(gaps, - float(min / 2.), float(min / 2.)),
                offset2_ex(gaps, - float(max / 2.), float(max / 2.)),
                true);
            ThickPolylines polylines;
            for (const ExPolygon &ex : gaps_ex)
                ex.medial_axis(max, min, &polylines);
            if (! polylines.empty()) {
				ExtrusionEntityCollection gap_fill;
				variable_width(polylines, erGapFill, this->solid_infill_flow, gap_fill.entities);
                /*  Make sure we don't infill narrow parts that are already gap-filled
                    (we only consider this surface's gaps to reduce the diff() complexity).
                    Growing actual extrusions ensures that gaps not filled by medial axis
                    are not subtracted from fill surfaces (they might be too short gaps
                    that medial axis skips but infill might join with other infill regions
                    and use zigzag).  */
                //FIXME Vojtech: This grows by a rounded extrusion width, not by line spacing,
                // therefore it may cover the area, but no the volume.
                last = diff_ex(to_polygons(last), gap_fill.polygons_covered_by_width(10.f));
				this->gap_fill->append(std::move(gap_fill.entities));
			}
        }

        // create one more offset to be used as boundary for fill
        // we offset by half the perimeter spacing (to get to the actual infill boundary)
        // and then we offset back and forth by half the infill spacing to only consider the
        // non-collapsing regions
        coord_t inset = 
            (loop_number < 0) ? 0 :
            (loop_number == 0) ?
                // one loop
                ext_perimeter_spacing / 2 :
                // two or more loops?
                perimeter_spacing / 2;
        // only apply infill overlap if we actually have one perimeter
        if (inset > 0)
            inset -= coord_t(scale_(this->config->get_abs_value("infill_overlap", unscale<double>(inset + solid_infill_spacing / 2))));
        // simplify infill contours according to resolution
        Polygons pp;
        for (ExPolygon &ex : last)
            ex.simplify_p(SCALED_RESOLUTION, &pp);
        // collapse too narrow infill areas
        coord_t min_perimeter_infill_spacing = coord_t(solid_infill_spacing * (1. - INSET_OVERLAP_TOLERANCE));
        // append infill areas to fill_surfaces
        this->fill_surfaces->append(
            offset2_ex(
                union_ex(pp),
                float(- inset - min_perimeter_infill_spacing / 2.),
                float(min_perimeter_infill_spacing / 2.)),
            stInternal);
#else
        PerimetersNesting perimeters = create_perimeters(
        	last, float(ext_perimeter_width), float(ext_perimeter_spacing), float(perimeter_width), float(perimeter_spacing), loop_number + 1);

        // Collect the infill regions at depth loop_number + 1.
		last = collect_infill_areas(perimeters);

        // create one more offset to be used as boundary for fill
        // we offset by half the perimeter spacing (to get to the actual infill boundary)
        // and then we offset back and forth by half the infill spacing to only consider the
        // non-collapsing regions
        coord_t inset = 
            (loop_number < 0) ? 0 :
            (loop_number == 0) ?
                // one loop
                ext_perimeter_spacing / 2 :
                // two or more loops?
                perimeter_spacing / 2;
        // only apply infill overlap if we actually have one perimeter
        if (inset > 0)
            inset -= coord_t(scale_(this->config->get_abs_value("infill_overlap", unscale<double>(inset + solid_infill_spacing / 2))));
        // simplify infill contours according to resolution
        Polygons pp;
        for (ExPolygon &ex : last)
            ex.simplify_p(SCALED_RESOLUTION, &pp);
        // collapse too narrow infill areas
        coord_t min_perimeter_infill_spacing = coord_t(solid_infill_spacing * (1. - INSET_OVERLAP_TOLERANCE));
        // append infill areas to fill_surfaces
        this->fill_surfaces->append(
            offset2_ex(
                union_ex(pp),
                float(- inset - min_perimeter_infill_spacing / 2.),
                float(min_perimeter_infill_spacing / 2.)),
            stInternal);
		
		ExtrusionEntityCollection extrusion_entities = collect_extrusions(*this, perimeters);
        // if brim will be printed, reverse the order of perimeters so that
        // we continue inwards after having finished the brim
        // TODO: add test for perimeter order
        if (this->config->external_perimeters_first || 
            (this->layer_id == 0 && this->print_config->brim_width.value > 0))
            extrusion_entities.reverse();
        // append perimeters for this slice as a collection
        if (! extrusion_entities.empty())
            this->loops->append(extrusion_entities);
#endif
    } // for each island
}

}
