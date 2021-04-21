#ifndef slic3r_SLA_SuppotstIslands_SampleConfig_hpp_
#define slic3r_SLA_SuppotstIslands_SampleConfig_hpp_

#include <libslic3r/libslic3r.h>

namespace Slic3r::sla {
/// <summary>
/// Configuration DTO 
/// Define where is neccessary to put support point on island
/// Mainly created by SampleConfigFactory
/// </summary>
struct SampleConfig
{
    // Every point on island has at least one support point in maximum distance
    // MUST be bigger than zero
    coord_t max_distance  = 2; 
    coord_t half_distance = 1; // has to be half od max_distance

    // Support point head radius
    // MUST be bigger than zero
    coord_t head_radius = 1; // [nano meter]

    // When it is possible, there will be this minimal distance from outline.
    // zero when head should be on outline
    coord_t minimal_distance_from_outline = 0; // [nano meter]

    // Distinguish when to add support point on VD outline point(center line sample)
    // MUST be bigger than minimal_distance_from_outline
    coord_t minimal_support_distance = 0;

    // minimal length of side branch to be sampled
    // it is used for sampling in center only
    coord_t min_side_branch_length = 0;

    // Maximal length of longest path in voronoi diagram to be island
    // supported only by one single support point this point will be in center of path.
    coord_t max_length_for_one_support_point = 1.;

    // Maximal length of island supported by 2 points
    coord_t max_length_for_two_support_points = 1.;

    // Maximal width of line island supported in the middle of line
    // Must be greater or equal to min_width_for_outline_support
    coord_t max_width_for_center_support_line = 1.;

    // Minimal width to be supported by outline
    // Must be smaller or equal to max_width_for_center_support_line
    coord_t min_width_for_outline_support = 1.;

    // Term criteria for end of alignment
    // Minimal change in manhatn move of support position before termination
    coord_t minimal_move = 1000; // in nanometers, devide from print resolution to quater pixel

    // Maximal count of align iteration
    size_t count_iteration = 100;
    
    // Sample outline of Field by this value
    // Less than max_distance
    coord_t outline_sample_distance = 2; 

    // Maximal distance over Voronoi diagram edges to find closest point during aligning Support point
    coord_t max_align_distance = 0.;
};
} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleConfig_hpp_
