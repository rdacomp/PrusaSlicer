#ifndef slic3r_SLA_SuppotstIslands_VoronoiGraphUtils_hpp_
#define slic3r_SLA_SuppotstIslands_VoronoiGraphUtils_hpp_

#include <optional>
#include <map>
#include <set>
#include <libslic3r/Geometry.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/SVG.hpp>

#include "VoronoiGraph.hpp"
#include "Parabola.hpp"
#include "SampleConfig.hpp"

namespace Slic3r::sla {

/// <summary>
/// Class which contain collection of static function
/// for work with Voronoi Graph.
/// </summary>
class VoronoiGraphUtils
{
    using VD = Slic3r::Geometry::VoronoiDiagram;

public:
    VoronoiGraphUtils() = delete;

    // return node from graph by vertex, when no exists create one
    static VoronoiGraph::Node *getNode(VoronoiGraph &         graph,
                                       const VD::vertex_type *vertex,
                                       const VD::edge_type *  edge,
                                       const Lines &          lines);

    /// <summary>
    /// extract parabola focus point from lines belongs to cell
    /// </summary>
    /// <param name="lines">source of Voronoi diagram</param>
    /// <param name="cell">cell inside of Voronoi diagram</param>
    /// <returns>Point from lines which create parabola.</returns>
    static Point retrieve_point(const Lines &lines, const VD::cell_type &cell);
    static Slic3r::Point get_parabola_point(const VD::edge_type &parabola, const Slic3r::Lines &lines);
    static Slic3r::Line get_parabola_line(const VD::edge_type &parabola, const Lines &lines);
    static Parabola get_parabola(const VD::edge_type &edge, const Lines &lines);

    /// <summary>
    /// Calculate length
    /// </summary>
    /// <param name="edge">curved edge</param>
    /// <returns>edge length</returns>
    static double calculate_length_of_parabola(
        const VD::edge_type &                edge,
        const Lines &                        lines);

    /// <summary>
    /// Calculate length of edge line segment or curve - parabola.
    /// </summary>
    /// <param name="edge">Input edge to calcuate length</param>
    /// <param name="lines">Source for Voronoi diagram. It contains parabola parameters</param>
    /// <returns>The length of edge</returns>
    static double calculate_length(const VD::edge_type &edge,
                                   const Lines &        lines);

    /// <summary>
    /// Calculate maximal distance to outline and multiply by two(must be similar on both side)
    /// </summary>
    /// <param name="edge">Input edge.</param>
    /// <param name="lines">Source for Voronoi diagram. It contains parabola parameters</param>
    /// <returns>Maximal island width along edge</returns>
    static double calculate_max_width(const VD::edge_type &edge,
                                      const Lines &        lines);

    /// <summary>
    /// calculate distances to border of island and length on skeleton
    /// </summary>
    /// <param name="voronoi_diagram">Input anotated voronoi diagram
    /// (use function Slic3r::Voronoi::annotate_inside_outside)</param>
    /// <param name="lines">Source lines for voronoi diagram</param>
    /// <returns>Extended voronoi graph by distances and length</returns>
    static VoronoiGraph getSkeleton(const VD &vd, const Lines &lines);

    /// <summary>
    /// For generating support point in distance from node
    /// </summary>
    /// <param name="node">Node lay on outline with only one neighbor</param>
    /// <param name="padding">Distance from outline</param>
    /// <returns></returns>
    static Slic3r::Point get_offseted_point(const VoronoiGraph::Node &node,
                                            double padding);

    /// <summary>
    /// find neighbor and return distance between nodes
    /// </summary>
    /// <param name="from">source of neighborse</param>
    /// <param name="to">neighbor node</param>
    /// <returns>When neighbor return distance between neighbor Nodes
    /// otherwise no value</returns>
    static const VoronoiGraph::Node::Neighbor *get_neighbor(
        const VoronoiGraph::Node *from, const VoronoiGraph::Node *to);

    /// <summary>
    /// use function get_neighbor
    /// when not neighbor assert
    /// </summary>
    /// <param name="from">source Node</param>
    /// <param name="to">destination Node</param>
    /// <returns>distance between Nodes or Assert when not neighbor</returns>
    static double get_neighbor_distance(const VoronoiGraph::Node *from,
                                        const VoronoiGraph::Node *to);

    /// <summary>
    /// Create longest node path over circle together with side branches
    /// </summary>
    /// <param name="circle">Source circle, can't be connected with another
    /// circle</param> <param name="side_branches">Circle side branches from
    /// nodes of circle</param> <param name="start_path">Path before circle -
    /// defince input point to circle</param> <returns>Longest nodes path and
    /// its length</returns>
    static VoronoiGraph::Path find_longest_path_on_circle(
        const VoronoiGraph::Circle &                 circle,
        const VoronoiGraph::ExPath::SideBranchesMap &side_branches);

    /// <summary>
    /// Serach longest path from input_node throw Nodes in connected circles,
    /// when circle is alone call find_longest_path_on_circle.
    /// </summary>
    /// <param name="input_node">Node on circle</param>
    /// <param name="finished_circle_index">index of circle with input
    /// node</param> <param name="ex_path">Hold Circles, connection of circles
    /// and Side branches</param> <returns>Longest path from input
    /// node</returns>
    static VoronoiGraph::Path find_longest_path_on_circles(
        const VoronoiGraph::Node &  input_node,
        size_t                      finished_circle_index,
        const VoronoiGraph::ExPath &ex_path);

    /// <summary>
    /// Function for detection circle in passed path.
    /// </summary>
    /// <param name="path">Already passed path in Graph</param>
    /// <param name="neighbor">Actual neighbor possible created circle</param>
    /// <returns>Circle when exists</returns>
    static std::optional<VoronoiGraph::Circle> create_circle(
        const VoronoiGraph::Path &          path,
        const VoronoiGraph::Node::Neighbor &neighbor);

    /// <summary>
    /// Move source connected circles into destination
    /// </summary>
    /// <param name="dst">In/Out param</param>
    /// <param name="src">Input possible modified, do not use it after this
    /// function</param> <param name="dst_circle_count">Count of destination
    /// circles before merge Source circle are append afted destination, therfore
    /// all src indexes must be increased by destination circle count</param>
    static void merge_connected_circle(
        VoronoiGraph::ExPath::ConnectedCircles &dst,
        VoronoiGraph::ExPath::ConnectedCircles &src,
        size_t                                  dst_circle_count);

    /// <summary>
    /// move data from source to destination
    /// side_branches + circles + connected_circle
    /// </summary>
    /// <param name="dst">destination extended path - append data from
    /// source</param> <param name="src">source extended path - data will be
    /// moved to dst</param>
    static void append_neighbor_branch(VoronoiGraph::ExPath &dst,
                                       VoronoiGraph::ExPath &src);

    /// <summary>
    /// Heal starting from random point.
    /// Compare length of all starting path with side branches
    /// when side branch is longer than swap with start path
    /// </summary>
    /// <param name="path">IN/OUT path to be fixed after creating longest path
    /// from one point</param>
    static void reshape_longest_path(VoronoiGraph::ExPath &path);

    /// <summary>
    /// Extract the longest path from voronoi graph
    /// by own function call stack(IStackFunction).
    /// Restructuralize path by branch created from random point.
    /// </summary>
    /// <param name="start_node">Random point from outline.</param>
    static VoronoiGraph::ExPath create_longest_path(
        const VoronoiGraph::Node *start_node);

    /// <summary>
    /// Create point on edge defined by neighbor
    /// in distance defined by edge length ratio
    /// </summary>
    static Point get_edge_point(const VD::edge_type *edge, double ratio);

    /// <summary>
    /// Find point lay on path with distance from first point on path
    /// </summary>
    /// <param name="path">Neighbor connected Nodes</param>
    /// <param name="distance">Distance to final point</param>
    /// <returns>Points with distance to first node</returns>
    static Point get_point_on_path(const VoronoiGraph::Nodes &path, double distance);

    /// <summary>
    /// Find point lay in center of path
    /// Distance from this point to front of path
    /// is same as distance to back of path
    /// </summary>
    /// <param name="path">Queue of neighbor nodes.(must be neighbor)</param>
    /// <param name="path_length">length of path</param>
    /// <returns>Point laying on voronoi diagram</returns>
    static Point get_center_of_path(const VoronoiGraph::Nodes &path, double path_length)
    { return get_point_on_path(path, path_length / 2); }

    /// <summary>
    /// decide how to sample longest path
    /// </summary>
    /// <param name="longest_path">Path inside voronoi diagram with all side branches and circles</param>
    /// <param name="config">Definition how to sample</param>
    /// <returns>Support points for island</returns>
    static std::vector<Point> sample_longest_path(
        const VoronoiGraph::ExPath &longest_path, 
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
    static std::vector<Point> sample_voronoi_graph(
        const VoronoiGraph &  graph,
        const SampleConfig &  config,
        VoronoiGraph::ExPath &longest_path);

public: // draw function for debug
    static void draw(SVG &svg, const VoronoiGraph &graph, coord_t width);
    static void draw(SVG &                      svg,
                     const VoronoiGraph::Nodes &path,
                     coord_t                    width,
                     const char *               color,
                     bool                       finish = false);
    static void draw(SVG &                       svg,
                     const VoronoiGraph::ExPath &path,
                     coord_t                     width);
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_VoronoiGraphUtils_hpp_
