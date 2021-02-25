#ifndef slic3r_SLA_SuppotstIslands_SampleConfig_hpp_
#define slic3r_SLA_SuppotstIslands_SampleConfig_hpp_

namespace Slic3r::sla {

/// <summary>
/// Configuration fro sampling voronoi diagram for support point generator
/// </summary>
struct SampleConfig
{
    // Maximal distance from edge
    double max_distance = 1.; // must be bigger than zero
    // Maximal distance between samples on skeleton
    double sample_size = 1.; // must be bigger than zero
    // distance from edge of skeleton
    double start_distance = 0; // support head diameter

    // maximal length of longest path in voronoi diagram to be island
    // supported only by one single support point this point will be in center
    // of path suggestion: smaller than 2* SampleConfig.start_distance
    double max_length_for_one_support_point = 1.;

    // each curve is sampled by this value to test distance to edge of island
    double curve_sample = 1.; // must be bigger than zero
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleConfig_hpp_
