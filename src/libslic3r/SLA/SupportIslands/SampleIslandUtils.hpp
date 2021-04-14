#ifndef slic3r_SLA_SuppotstIslands_SampleIslandUtils_hpp_
#define slic3r_SLA_SuppotstIslands_SampleIslandUtils_hpp_

#include <vector>
#include <optional>

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
    /// Create support point on edge defined by neighbor
    /// </summary>
    /// <param name="neighbor">Source edge</param>
    /// <param name="ratio">Source distance ratio for position on edge</param>
    /// <param name="type">Type of point</param>
    /// <returns>created support island point</returns>
    static SupportIslandPointPtr create_point(
        const VoronoiGraph::Node::Neighbor *neighbor,
        double                              ratio,
        SupportIslandPoint::Type type = SupportIslandPoint::Type::undefined);

    /// <summary>
    /// Find point lay on path with distance from first point on path
    /// </summary>
    /// <param name="path">Neighbor connected Nodes</param>
    /// <param name="distance">Distance to final point</param>
    /// <returns>Points with distance to first node</returns>
    static SupportIslandPointPtr create_point_on_path(
        const VoronoiGraph::Nodes &path,
        double                     distance,
        SupportIslandPoint::Type type = SupportIslandPoint::Type::undefined);

    /// <summary>
    /// Find point lay in center of path
    /// Distance from this point to front of path
    /// is same as distance to back of path
    /// </summary>
    /// <param name="path">Queue of neighbor nodes.(must be neighbor)</param>
    /// <param name="path_length">length of path</param>
    /// <returns>Point laying on voronoi diagram</returns>
    static SupportIslandPointPtr create_middle_path_point(
        const VoronoiGraph::Path &path,
        SupportIslandPoint::Type  type = SupportIslandPoint::Type::undefined);    

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
    static void sample_center_circles(const VoronoiGraph::ExPath &   path,
                                      const CenterLineConfiguration &cfg,
                                      SupportIslandPoints &          result);
    static SupportIslandPoints sample_center_circle(
        const std::set<const VoronoiGraph::Node *> &        circle_set,
        std::map<const VoronoiGraph::Node *, double> &path_distances,
        const CenterLineConfiguration &             cfg);
    static void sample_center_circle_end(
        const VoronoiGraph::Node::Neighbor &neighbor,
        double &                            neighbor_distance,
        const VoronoiGraph::Nodes &         done_nodes,
        double &                            node_distance,
        const CenterLineConfiguration &     cfg,
        SupportIslandPoints &               result);

    /// <summary>
    /// Sample voronoi skeleton
    /// </summary>
    /// <param name="graph">Inside skeleton of island</param>
    /// <param name="lines">Source lines for VG --> outline of island.</param>
    /// <param name="config">Params for sampling</param>
    /// <param name="longest_path">OUTPUT: longest path in graph</param>
    /// <returns>Vector of sampled points or Empty when distance from edge is
    /// bigger than max_distance</returns>
    static SupportIslandPoints sample_voronoi_graph(
        const VoronoiGraph &  graph,
        const Lines & lines,
        const SampleConfig &  config,
        VoronoiGraph::ExPath &longest_path);

    /// <summary>
    /// Decide how to sample path
    /// </summary>
    /// <param name="path">Path inside voronoi diagram with side branches and circles</param>
    /// <param name="lines">Source lines for VG --> outline of island.</param>
    /// <param name="config">Definition how to sample</param>
    /// <returns>Support points for island</returns>
    static SupportIslandPoints sample_expath(const VoronoiGraph::ExPath &path,
                                             const Lines &       lines,
                                             const SampleConfig &config);


    /// <summary>
    /// Transform support point to slicer points
    /// </summary>
    static Slic3r::Points to_points(const SupportIslandPoints &support_points);
    static std::vector<Vec2f> to_points_f(const SupportIslandPoints &support_points);
    
    /// <summary>
    /// keep same distances between support points
    /// call once align
    /// </summary>
    /// <param name="samples">In/Out support points to be alligned(min 3 points)</param>
    /// <param name="island">Area for sampling, border for position of samples</param>
    /// <param name="config"> Sampling configuration
    /// Maximal distance between neighbor points + 
    /// Term criteria for align: Minimal sample move and Maximal count of iteration</param>
    static void align_samples(SupportIslandPoints &samples,
                              const ExPolygon &    island,
                              const SampleConfig &config);
    
    /// <summary>
    /// once align
    /// </summary>
    /// <param name="samples">In/Out support points to be alligned(min 3 points)</param>
    /// <param name="island">Area for sampling, border for position of samples</param>
    /// <param name="config"> Sampling configuration
    /// Maximal distance between neighbor points + 
    /// Term criteria for align: Minimal sample move and Maximal count of iteration</param>
    static coord_t align_once(SupportIslandPoints &samples,
                              const ExPolygon &    island,
                              const SampleConfig & config);

    /// <summary>
    /// Align support point the closest to Wanted point
    /// </summary>
    /// <param name="support">In/Out support point, contain restriction for move</param>
    /// <param name="center">Wanted position point</param>
    /// <param name="max_distance">Maximal search distance on VD for closest point</param>
    /// <returns>Distance move of original point</returns>
    static coord_t align_support(SupportIslandPoint &support, const Point &wanted, double max_distance);

    /// <summary>
    /// DTO hold information for start sampling VD in center
    /// </summary>
    struct CenterStart
    {
        // Start of ceneter sampled line segment
        const VoronoiGraph::Node::Neighbor *neighbor;

        // distance to nearest support point
        coord_t support_in; // [nano meters]

        VoronoiGraph::Nodes path; // to sample in distance from border

        CenterStart(const VoronoiGraph::Node::Neighbor *neighbor,
                    coord_t                             support_in,
                    VoronoiGraph::Nodes                 path = {})
            : neighbor(neighbor), support_in(support_in), path(path)
        {}
    };
    using CenterStarts = std::queue<CenterStart>;

    /// <summary>
    /// Sample VG in center --> sample tiny part of island
    /// Detection of wide neighbor which start field
    /// Creating of new starts (from VG cross -> multi neighbors)
    /// </summary>
    /// <param name="start">Start to sample</param>
    /// <param name="config">Parameters for sampling</param>
    /// <param name="done">Already done nodes</param>
    /// <param name="results">Result of sampling</param>
    /// <param name="liness">Source line for VD. To decide position of change from tiny to wide part</param>
    /// <param name="field_start">Output: Wide neighbor, start of field</param>
    /// <returns>New start of sampling</returns>
    static std::vector<CenterStart> sample_center(
        const CenterStart &                   start,
        const SampleConfig &                  config,
        std::set<const VoronoiGraph::Node *> &done,
        SupportIslandPoints &                 results,
        const Lines &lines,
        std::optional<VoronoiGraph::Position> &field_start);

    /// <summary>
    /// DTO represents Wide parts of island to sample
    /// extend polygon with information about source lines
    /// </summary>
    struct Field
    {
        // border of field created by source lines and closing of tiny island 
        ExPolygon border;

        // same size as polygon.points.size()
        // indexes to source island lines 
        // in case (index > lines.size()) it means fill the gap from tiny part of island
        std::vector<size_t> source_indexes;
        // value for source index of change from wide to tiny part of island
        size_t              source_indexe_for_change;
    };

    /// <summary>
    /// Create & sample field -- wide part of island
    /// </summary>
    /// <param name="field_start">Start neighbor, first occur of wide neighbor.</param>
    /// <param name="tiny_starts">Append new founded tiny parts of island.</param>
    /// <param name="tiny_done">Already sampled node sets. Filled only node inside field imediate after change</param>
    /// <param name="lines">Source lines for VG --> outline of island.</param>
    /// <param name="config">Containe Minimal width in field and sample distance for center line</param>
    static void sample_field(VoronoiGraph::Position &field_start,
                             SupportIslandPoints &   points,
                             CenterStarts &          center_starts,
                             std::set<const VoronoiGraph::Node *> &done,
                             const Lines &                         lines,
                             const SampleConfig &                  config);

    /// <summary>
    /// Create field from input neighbor
    /// </summary>
    /// <param name="field_start">Start position, change from tiny to wide.</param>
    /// <param name="tiny_starts">Append new founded tiny parts of island.</param>
    /// <param name="tiny_done">Already sampled node sets.</param>
    /// <param name="lines">Source lines for VG --> outline of island.</param>
    /// <param name="config">Containe Minimal width in field and sample distance for center line</param>
    /// <returns>New created field</returns>
    static Field create_field(const VoronoiGraph::Position &field_start,
                              CenterStarts &    tiny_starts,
                              std::set<const VoronoiGraph::Node *> &tiny_done,
                              const Lines &     lines,
                              const SampleConfig &config);

    /// <summary>
    /// create support points on border of field
    /// </summary>
    /// <param name="field">Input field</param>
    /// <param name="config">Parameters for sampling.</param>
    /// <returns>support for outline</returns>
    static SupportIslandPoints sample_outline(const Field &       field,
                                              const SampleConfig &config);

    // debug draw functions
    static void draw(SVG &        svg,
                     const Field &field,
                     bool         draw_border_line_indexes  = false,
                     bool         draw_field_source_indexes = true);

    static void draw(SVG &                      svg,
                     const SupportIslandPoints &supportIslandPoints,
                     double      size,
                     const char *color = "lightgreen",
                     bool write_type    = true);
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SampleIslandUtils_hpp_
