#ifndef slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_
#define slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_

#include <libslic3r/sla/SupportPointGenerator.hpp>

namespace Slic3r::sla {

/// <summary>
/// Factory to create configuration
/// </summary>
class SampleConfigFactory
{
public:
    SampleConfigFactory() = delete;

    // factory method to iniciate config
    static SampleConfig create(const SupportPointGenerator::Config &config)
    {
        // TODO: find valid params !!!!
        SampleConfig result;
        result.max_distance                  = 100 * config.head_diameter;
        result.half_distance                 = result.max_distance / 2;
        result.head_radius                   = config.head_diameter / 2;
        result.minimal_distance_from_outline = config.head_diameter / 2.;
        result.minimal_support_distance = result.minimal_distance_from_outline +
                                          result.half_distance;

        result.max_length_for_one_support_point =
            2 * result.minimal_distance_from_outline + 
            config.head_diameter;
        coord_t max_length_for_one_support_point = 
            2 * result.max_distance +
            config.head_diameter +
            2 * result.minimal_distance_from_outline;
        if (result.max_length_for_one_support_point > max_length_for_one_support_point)
            result.max_length_for_one_support_point = max_length_for_one_support_point;
        coord_t min_length_for_one_support_point =
            2 * config.head_diameter +
            2 * result.minimal_distance_from_outline;
        if (result.max_length_for_one_support_point < min_length_for_one_support_point)
            result.max_length_for_one_support_point = min_length_for_one_support_point;

        result.max_length_for_two_support_points =
            2 * result.max_distance + 2 * config.head_diameter +
            2 * result.minimal_distance_from_outline;
        coord_t max_length_for_two_support_points =
            2 * result.max_distance + 
            2 * config.head_diameter +
            2 * result.minimal_distance_from_outline;
        if (result.max_length_for_two_support_points > max_length_for_two_support_points)
            result.max_length_for_two_support_points = max_length_for_two_support_points;
        assert(result.max_length_for_two_support_points < result.max_length_for_one_support_point);

        result.max_width_for_center_support_line = 2 * config.head_diameter;
        coord_t min_width_for_center_support_line =
            config.head_diameter + 2 * result.minimal_distance_from_outline;
        if (result.max_width_for_center_support_line < min_width_for_center_support_line)
            result.max_width_for_center_support_line = min_width_for_center_support_line;
        coord_t max_width_for_center_support_line = 2 * result.max_distance + config.head_diameter;
        if (result.max_width_for_center_support_line > max_width_for_center_support_line)
            result.max_width_for_center_support_line = max_width_for_center_support_line;

        result.min_width_for_outline_support = result.max_width_for_center_support_line - 2 * config.head_diameter;
        assert(result.min_width_for_outline_support >= result.max_width_for_center_support_line);

        // Align support points
        // TODO: propagate print resolution
        result.minimal_move = 1000; // [in nanometers], devide from print resolution to quater pixel
        result.count_iteration = 100; // speed VS precission
        result.max_align_distance = result.max_distance / 2;

        return result;
    }
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_
