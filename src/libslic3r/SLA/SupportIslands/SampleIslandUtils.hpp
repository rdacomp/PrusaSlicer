#ifndef slic3r_SLA_SuppotstIslands_SampleIslandUtils_hpp_
#define slic3r_SLA_SuppotstIslands_SampleIslandUtils_hpp_

#include <libslic3r/Geometry.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/SVG.hpp>

#include "VoronoiGraph.hpp"
#include "Parabola.hpp"
#include "SampleConfig.hpp"
#include "SupportIslandPoint.hpp"

namespace Slic3r::sla {

/// <summary>
/// Utils class with only static function
/// Function for sampling island by Voronoi Graph.
/// </summary>
class SampleIslandUtils
{
public:
    SampleIslandUtils() = delete;

    /// <summary>
    /// Find point lay on path with distance from first point on path
    /// </summary>
    /// <param name="path">Neighbor connected Nodes</param>
    /// <param name="distance">Distance to final point</param>
    /// <returns>Points with distance to first node</returns>
    static Point get_point_on_path(const VoronoiGraph::Nodes &path,
                                   double                     distance);

    /// <summary>
    /// Find point lay in center of path
    /// Distance from this point to front of path
    /// is same as distance to back of path
    /// </summary>
    /// <param name="path">Queue of neighbor nodes.(must be neighbor)</param>
    /// <param name="path_length">length of path</param>
    /// <returns>Point laying on voronoi diagram</returns>
    static Point get_center_of_path(const VoronoiGraph::Nodes &path,
                                    double                     path_length)
    {
        return get_point_on_path(path, path_length / 2);
    }

    // create 2 points on both ends of path with side distance from border
    static SupportIslandPoints create_side_points(const VoronoiGraph::Nodes &path, double side_distance);

    // DTO with data for sampling line in center
    struct CenterLineConfiguration
    {
        const VoronoiGraph::ExPath::SideBranchesMap &branches_map;
        double min_length;
        double max_sample_distance;
        double side_distance;
        CenterLineConfiguration(
            const VoronoiGraph::ExPath::SideBranchesMap &branches_map,
            double                                       min_length,
            double                                       max_sample_distance,
            double                                       side_distance)
            : branches_map(branches_map)
            , min_length(min_length)
            , max_sample_distance(max_sample_distance)
            , side_distance(side_distance)
        {}
    };
    // create points along path and its side branches(recursively)
    static SupportIslandPoints sample_side_branch(
        const VoronoiGraph::Node *     first_node,
        const VoronoiGraph::Path       side_path,
        double                         start_offset,
        const CenterLineConfiguration &cfg);    
    static SupportIslandPoints sample_side_branches(
        const VoronoiGraph::ExPath::SideBranchesMap::const_iterator&  side_branches_iterator,
        double                         start_offset,
        const CenterLineConfiguration &cfg);
    // create points along path and its side branches(recursively) + colve circles
    static SupportIslandPoints sample_center_line(const VoronoiGraph::ExPath &path, const CenterLineConfiguration &cfg);
    // create point along multi circles in path
    static SupportIslandPoints sample_center_circles(const VoronoiGraph::ExPath &path, const CenterLineConfiguration &cfg);
    static SupportIslandPoints sample_center_circle(
        const std::set<const VoronoiGraph::Node *> &circle_set,
        const VoronoiGraph::Nodes &                 path_nodes,
        const CenterLineConfiguration &             cfg);

    /// <summary>
    /// Decide how to sample path
    /// </summary>
    /// <param name="path">Path inside voronoi diagram with side branches and circles</param>
    /// <param name="config">Definition how to sample</param>
    /// <returns>Support points for island</returns>
    static SupportIslandPoints sample_expath(
        const VoronoiGraph::ExPath &path, 
        const SampleConfig &config
    );

    /// <summary>
    /// Sample voronoi skeleton
    /// </summary>
    /// <param name="graph">Inside skeleton of island</param>
    /// <param name="config">Params for sampling</param>
    /// <param name="longest_path">OUTPUT: longest path in graph</param>
    /// <returns>Vector of sampled points or Empty when distance from edge is
    /// bigger than max_distance</returns>
    static SupportIslandPoints sample_voronoi_graph(
        const VoronoiGraph &  graph,
        const SampleConfig &  config,
        VoronoiGraph::ExPath &longest_path);

    static void draw(SVG &                      svg,
                     const SupportIslandPoints &supportIslandPoints,
                     double      size,
                     const char *color = "lightgreen",
                     bool write_type    = true);
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleIslandUtils_hpp_
