#ifndef slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_
#define slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_

#include "../SupportPointGenerator.hpp"

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
        coord_t head_diameter = scale_(config.head_diameter);
        coord_t min_distance  = scale_(config.minimal_distance);
        coord_t max_distance  = 3 * min_distance;
        coord_t sample_multiplicator = 10; // allign is made by selecting from samples


        // TODO: find valid params !!!!
        SampleConfig result;
        result.max_distance                  = max_distance / sample_multiplicator;
        result.half_distance                 = result.max_distance / 2;
        result.head_radius                   = head_diameter / 2;
        result.minimal_distance_from_outline = result.head_radius;
        result.maximal_distance_from_outline = result.max_distance/4;
        assert(result.minimal_distance_from_outline < result.maximal_distance_from_outline);
        result.minimal_support_distance = result.minimal_distance_from_outline +
                                          result.half_distance;

        result.min_side_branch_length = 2 * result.minimal_distance_from_outline;

        result.max_length_for_one_support_point =
            2 * result.minimal_distance_from_outline + 
            head_diameter;
        coord_t max_length_for_one_support_point = 
            2 * max_distance +
            head_diameter +
            2 * result.minimal_distance_from_outline;
        if (result.max_length_for_one_support_point > max_length_for_one_support_point)
            result.max_length_for_one_support_point = max_length_for_one_support_point;
        coord_t min_length_for_one_support_point =
            2 * head_diameter +
            2 * result.minimal_distance_from_outline;
        if (result.max_length_for_one_support_point < min_length_for_one_support_point)
            result.max_length_for_one_support_point = min_length_for_one_support_point;

        result.max_length_for_two_support_points =
            2 * max_distance + 2 * head_diameter +
            2 * result.minimal_distance_from_outline;
        coord_t max_length_for_two_support_points =
            2 * max_distance + 
            2 * head_diameter +
            2 * result.minimal_distance_from_outline;
        if (result.max_length_for_two_support_points > max_length_for_two_support_points)
            result.max_length_for_two_support_points = max_length_for_two_support_points;
        assert(result.max_length_for_two_support_points > result.max_length_for_one_support_point);

        result.max_width_for_center_support_line =
            2 * head_diameter + 2 * result.minimal_distance_from_outline +
            max_distance / 2;
        coord_t min_width_for_center_support_line = head_diameter + 2 * result.minimal_distance_from_outline;
        if (result.max_width_for_center_support_line < min_width_for_center_support_line)
            result.max_width_for_center_support_line = min_width_for_center_support_line;
        coord_t max_width_for_center_support_line = 2 * max_distance + head_diameter;
        if (result.max_width_for_center_support_line > max_width_for_center_support_line)
            result.max_width_for_center_support_line = max_width_for_center_support_line;

        result.min_width_for_outline_support = result.max_width_for_center_support_line - 2 * head_diameter;
        assert(result.min_width_for_outline_support <= result.max_width_for_center_support_line);

        result.outline_sample_distance = result.max_distance/20;

        // Align support points
        // TODO: propagate print resolution
        result.minimal_move = 10000.;// [in nanometers --> 0.01mm ], devide from print resolution to quater pixel
        result.count_iteration = 100; // speed VS precission
        result.max_align_distance = result.max_distance / 2;

        return result;
    }
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleConfigFactory_hpp_
