#ifndef slic3r_SLA_SuppotstIslands_SampleConfig_hpp_
#define slic3r_SLA_SuppotstIslands_SampleConfig_hpp_

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
    double max_distance = 1.; 

    // Support point head radius
    // MUST be bigger than zero
    double head_radius = 1;

    // When it is possible, there will be this minimal distance from outline.
    // zero when head should be on outline
    double minimal_distance_from_outline = 0.; 

    // Maximal length of longest path in voronoi diagram to be island
    // supported only by one single support point this point will be in center of path.
    double max_length_for_one_support_point = 1.;

    // Maximal length of island supported by 2 points
    double max_length_for_two_support_points = 1.;

    // Maximal width of line island supported in the middle of line
    double max_width_for_center_supportr_line = 1.;
    // Maximal width of line island supported by zig zag
    double max_width_for_zig_zag_supportr_line = 1.;

    // Term criteria for end of alignment
    // Minimal change in manhatn move of support position before termination
    coord_t minimal_move     = 1;
    // Maximal count of align iteration
    size_t  count_iteration = 100;
};
} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleConfig_hpp_
