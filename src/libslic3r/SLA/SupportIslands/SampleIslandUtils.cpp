#include "SampleIslandUtils.hpp"

#include <cmath>
#include <optional>
#include <libslic3r/VoronoiOffset.hpp>
#include "IStackFunction.hpp"
#include "EvaluateNeighbor.hpp"
#include "ParabolaUtils.hpp"
#include "VoronoiGraphUtils.hpp"
#include "VectorUtils.hpp"
#include "LineUtils.hpp"
#include "PointUtils.hpp"

#include <libslic3r/VoronoiVisualUtils.hpp>

#include <libslic3r/ClipperUtils.hpp> // allign

#include "libslic3r/SLA/SupportPointGenerator.hpp"

// comment definition of NDEBUG to enable assert()
//#define NDEBUG

//#define SLA_SAMPLE_ISLAND_UTILS_STORE_VORONOI_GRAPH_TO_SVG
//#define SLA_SAMPLE_ISLAND_UTILS_STORE_INITIAL_SAMPLE_POSITION_TO_SVG
//#define SLA_SAMPLE_ISLAND_UTILS_STORE_FIELD_TO_SVG
//#define SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGNE_VD_TO_SVG
//#define SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
//#define SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGNED_TO_SVG

#include <cassert>

using namespace Slic3r::sla;

std::vector<Slic3r::Vec2f> SampleIslandUtils::sample_expolygon(
    const ExPolygon &expoly, float samples_per_mm2)
{
    static const float mm2_area = static_cast<float>(scale_(1) * scale_(1));
    // Equilateral triangle area = (side * height) / 2
    float triangle_area = mm2_area / samples_per_mm2;
    // Triangle area = sqrt(3) / 4 * "triangle side"
    static const float coef1         = sqrt(3.) / 4.;
    coord_t            triangle_side = static_cast<coord_t>(
        std::round(sqrt(triangle_area * coef1)));

    Points points = sample_expolygon(expoly, triangle_side);

    std::vector<Vec2f> result;
    result.reserve(points.size());
    std::transform(points.begin(), points.end(), std::back_inserter(result), 
        [](const Point &p) { return unscale(p).cast<float>(); });

    return result;
}

Slic3r::Points SampleIslandUtils::sample_expolygon(const ExPolygon &expoly,
                                                   coord_t triangle_side)
{
    const Points &points = expoly.contour.points;
    assert(!points.empty());
    // get y range
    coord_t min_y = points.front().y();
    coord_t max_y = min_y;
    for (const Point &point : points) {
        if (min_y > point.y())
            min_y = point.y();
        else if (max_y < point.y())
            max_y = point.y();
    }

    coord_t triangle_side_2 = triangle_side / 2;
    static const float coef2           = sqrt(3.) / 2.;
    coord_t triangle_height = static_cast<coord_t>(std::round(triangle_side * coef2));

    // IMPROVE: use line end y
    Lines lines = to_lines(expoly);
    // remove lines paralel with axe x
    lines.erase(std::remove_if(lines.begin(), lines.end(),
                               [](const Line &l) {
                                   return l.a.y() == l.b.y();
                               }), lines.end());

    // change line direction from top to bottom
    for (Line &line : lines)
        if (line.a.y() > line.b.y()) std::swap(line.a, line.b);
    
    // sort by a.y()
    std::sort(lines.begin(), lines.end(),
              [](const Line &l1, const Line &l2) -> bool {
                  return l1.a.y() < l2.a.y();
              });
    // IMPROVE: guess size and reserve points
    Points result;
    size_t start_index = 0;
    bool is_odd = false;
    for (coord_t y = min_y + triangle_height / 2; y < max_y;
         y += triangle_height) {
        is_odd = !is_odd;
        std::vector<coord_t> intersections;
        bool increase_start_index = true;
        for (auto line = std::begin(lines)+start_index; line != std::end(lines); ++line) {
            const Point &b = line->b;
            if (b.y() <= y) {
                // removing lines is slow, start index is faster
                // line = lines.erase(line); 
                if (increase_start_index) ++start_index;
                continue;
            }
            increase_start_index = false;
            const Point &a = line->a;
            if (a.y() >= y) break;
            float   y_range      = static_cast<float>(b.y() - a.y());
            float   x_range      = static_cast<float>(b.x() - a.x());
            float   ratio        = (y - a.y()) / y_range;
            coord_t intersection = a.x() +
                                   static_cast<coord_t>(x_range * ratio);
            intersections.push_back(intersection);
        }
        assert(intersections.size() % 2 == 0);
        std::sort(intersections.begin(), intersections.end());
        for (size_t index = 0; index + 1 < intersections.size(); index += 2) {
            coord_t start_x = intersections[index];
            coord_t end_x   = intersections[index + 1];
            if (is_odd) start_x += triangle_side_2;
            coord_t div = start_x / triangle_side;
            if (start_x > 0) div += 1;
            coord_t x = div * triangle_side;
            if (is_odd) x -= triangle_side_2;
            while (x < end_x) {
                result.emplace_back(x, y);
                x += triangle_side;
            }
        }
    }
    return result;
}

SupportIslandPointPtr SampleIslandUtils::create_no_move_point(
    const VoronoiGraph::Node::Neighbor *neighbor,
    double                              ratio,
    SupportIslandPoint::Type            type)
{
    VoronoiGraph::Position position(neighbor, ratio);
    return create_no_move_point(position, type);
}

SupportIslandPointPtr SampleIslandUtils::create_no_move_point(
    const VoronoiGraph::Position &position,
    SupportIslandPoint::Type      type)
{
    Point point = VoronoiGraphUtils::create_edge_point(position);
    return std::make_unique<SupportIslandNoMovePoint>(point, type);
}

 std::optional<VoronoiGraph::Position> SampleIslandUtils::create_position_on_path(
    const VoronoiGraph::Nodes &path,
    double                     distance)
{
    const VoronoiGraph::Node *prev_node       = nullptr;
    double                    actual_distance = 0.;
    for (const VoronoiGraph::Node *node : path) {
        if (prev_node == nullptr) { // first call
            prev_node = node;
            continue;
        }
        const VoronoiGraph::Node::Neighbor *neighbor =
            VoronoiGraphUtils::get_neighbor(prev_node, node);
        actual_distance += neighbor->length();
        if (actual_distance >= distance) {
            // over half point is on
            double behind_position = actual_distance - distance;
            double ratio           = 1. - behind_position / neighbor->length();
            return VoronoiGraph::Position(neighbor, ratio);
        }
        prev_node = node;
    }

    // distance must be inside path
    // this means bad input params
    assert(false);
    return {}; // unreachable
}

std::optional<VoronoiGraph::Position>
SampleIslandUtils::create_position_on_path(const VoronoiGraph::Nodes &path,
                                           const Lines &              lines,
                                           coord_t                    width,
                                           coord_t &max_distance)
{
    const VoronoiGraph::Node *prev_node = nullptr;
    coord_t  actual_distance = 0;
    for (const VoronoiGraph::Node *node : path) {
        if (prev_node == nullptr) { // first call
            prev_node = node;
            continue;
        }
        const VoronoiGraph::Node::Neighbor *neighbor =
            VoronoiGraphUtils::get_neighbor(prev_node, node);

        if (width <= neighbor->max_width()) {
            VoronoiGraph::Position position = VoronoiGraphUtils::get_position_with_width(neighbor, width, lines);
            // set max distance to actual distance
            coord_t rest_distance = position.calc_distance();
            coord_t distance      = actual_distance + rest_distance;
            if (max_distance > distance) {
                max_distance = distance;
                return position;
            }
        }

        actual_distance += static_cast<coord_t>(neighbor->length());
        if (actual_distance >= max_distance) {
            // over half point is on
            coord_t behind_position = actual_distance - max_distance;
            double ratio = 1. - behind_position / neighbor->length();
            return VoronoiGraph::Position(neighbor, ratio);
        }
        prev_node = node;
    }

    // distance must be inside path
    // this means bad input params
    assert(false);
    return {}; // unreachable
}

SupportIslandPointPtr SampleIslandUtils::create_center_island_point(
    const VoronoiGraph::Nodes &path,
    double                     distance,
    const SampleConfig &       config,
    SupportIslandPoint::Type   type)
{
    auto position_opt = create_position_on_path(path, distance);
    if (!position_opt.has_value()) return nullptr;
    return std::make_unique<SupportCenterIslandPoint>(*position_opt, &config, type);
}

SupportIslandPointPtr SampleIslandUtils::create_middle_path_point(
    const VoronoiGraph::Path &path, SupportIslandPoint::Type  type)
{
    auto position_opt = create_position_on_path(path.nodes, path.length / 2);
    if (!position_opt.has_value()) return nullptr;
    return create_no_move_point(*position_opt, type);
}

SupportIslandPoints SampleIslandUtils::create_side_points(
    const VoronoiGraph::Nodes &path, 
    const Lines& lines,
    coord_t width,
    coord_t max_side_distance)
{
    VoronoiGraph::Nodes reverse_path = path; // copy
    std::reverse(reverse_path.begin(), reverse_path.end());
    coord_t distance2 = max_side_distance; // copy
    auto pos1 = create_position_on_path(path, lines, width, max_side_distance);
    auto pos2 = create_position_on_path(reverse_path, lines, width, distance2);
    assert(pos1.has_value());
    assert(pos2.has_value());
    SupportIslandPoint::Type type = SupportIslandPoint::Type::two_points;
    SupportIslandPoints      result;
    result.reserve(2);
    result.push_back(create_no_move_point(*pos1, type));
    result.push_back(create_no_move_point(*pos2, type));
    return result;
}

SupportIslandPoints SampleIslandUtils::sample_side_branch(
    const VoronoiGraph::Node *     first_node,
    const VoronoiGraph::Path       side_path,
    double                         start_offset,
    const CenterLineConfiguration &cfg)
{
    const double max_distance = cfg.sample_config.max_distance;
    assert(max_distance > start_offset);
    double distance = max_distance - start_offset;
    const double side_distance = cfg.sample_config.minimal_distance_from_outline;
    double length   = side_path.length - side_distance - distance;
    if (length < 0.) {
        VoronoiGraph::Nodes reverse_path = side_path.nodes;
        std::reverse(reverse_path.begin(), reverse_path.end());
        reverse_path.push_back(first_node);
        SupportIslandPoints result;
        result.push_back(
            create_center_island_point(reverse_path, side_distance, cfg.sample_config,
                                 SupportIslandPoint::Type::center_line_end));
        return result;
    }
    // count of segment between points on main path
    size_t segment_count = static_cast<size_t>(
        std::ceil(length / max_distance));
    double                     sample_distance = length / segment_count;
    SupportIslandPoints result;
    result.reserve(segment_count + 1);
    const VoronoiGraph::Node *prev_node = first_node;
    for (const VoronoiGraph::Node *node : side_path.nodes) {
        const VoronoiGraph::Node::Neighbor *neighbor =
            VoronoiGraphUtils::get_neighbor(prev_node, node);
        auto side_item = cfg.branches_map.find(node);
        if (side_item != cfg.branches_map.end()) {
            double start_offset = (distance < sample_distance / 2.) ?
                                      distance :
                                      (sample_distance - distance);

            if (side_item->second.top().length > cfg.sample_config.min_side_branch_length) {
                auto side_samples = sample_side_branches(side_item,
                                                         start_offset, cfg);
                result.insert(result.end(), std::move_iterator(side_samples.begin()),
                              std::move_iterator(side_samples.end()));
            }
        }
        while (distance < neighbor->length()) {
            double edge_ratio = distance / neighbor->length();
            result.push_back(
                create_no_move_point(neighbor, edge_ratio,
                                     SupportIslandPoint::Type::center_line)
            );
            distance += sample_distance;
        }
        distance -= neighbor->length();
        prev_node = node;
    }
    //if (cfg.side_distance > sample_distance) {
    //    int count = static_cast<int>(std::floor(cfg.side_distance / sample_distance));
    //    for (int i = 0; i < count; ++i) { 
    //        distance += sample_distance;
    //        result.pop_back();
    //    }
    //}
    //assert(fabs(distance - (sample_distance - cfg.side_distance)) < 1e-5);
    result.back()->type = SupportIslandPoint::Type::center_line_end;
    return result;
}

SupportIslandPoints SampleIslandUtils::sample_side_branches(
    const VoronoiGraph::ExPath::SideBranchesMap::const_iterator
        &                          side_branches_iterator,
    double                         start_offset,
    const CenterLineConfiguration &cfg)
{
    const VoronoiGraph::ExPath::SideBranches &side_branches =
        side_branches_iterator->second;
    const VoronoiGraph::Node *first_node = side_branches_iterator->first;
    if (side_branches.size() == 1)
        return sample_side_branch(first_node, side_branches.top(),
                                  start_offset, cfg);

    SupportIslandPoints         result;
    VoronoiGraph::ExPath::SideBranches side_branches_cpy = side_branches;
    while (side_branches_cpy.top().length >
           cfg.sample_config.min_side_branch_length) {
        auto samples = sample_side_branch(first_node, side_branches_cpy.top(),
                                          start_offset, cfg);
        result.insert(result.end(), 
            std::move_iterator(samples.begin()), 
            std::move_iterator(samples.end()));
        side_branches_cpy.pop();
    }
    return result;
}

std::vector<std::set<const VoronoiGraph::Node *>> create_circles_sets(
    const std::vector<VoronoiGraph::Circle> &     circles,
    const VoronoiGraph::ExPath::ConnectedCircles &connected_circle)
{
    std::vector<std::set<const VoronoiGraph::Node *>> result;
    std::vector<bool> done_circle(circles.size(), false);
    for (size_t circle_index = 0; circle_index < circles.size();
         ++circle_index) {
        if (done_circle[circle_index]) continue;
        done_circle[circle_index] = true;
        std::set<const VoronoiGraph::Node *> circle_nodes;
        const VoronoiGraph::Circle &         circle = circles[circle_index];
        for (const VoronoiGraph::Node *node : circle.nodes)
            circle_nodes.insert(node);

        circle_nodes.insert(circle.nodes.begin(), circle.nodes.end());
        auto cc = connected_circle.find(circle_index);
        if (cc != connected_circle.end()) {
            for (const size_t &cc_index : cc->second) {
                done_circle[cc_index]              = true;
                const VoronoiGraph::Circle &circle = circles[cc_index];
                circle_nodes.insert(circle.nodes.begin(), circle.nodes.end());
            }
        }
        result.push_back(circle_nodes);
    }
    return result;
}

Slic3r::Points SampleIslandUtils::to_points(const SupportIslandPoints &support_points)
{ 
    std::function<Point(const std::unique_ptr<SupportIslandPoint> &)> transform_func = &SupportIslandPoint::point;
    return VectorUtils::transform(support_points, transform_func);
}

std::vector<Slic3r::Vec2f> SampleIslandUtils::to_points_f(const SupportIslandPoints &support_points)
{
    std::function<Vec2f(const std::unique_ptr<SupportIslandPoint> &)> transform_func =
        [](const std::unique_ptr<SupportIslandPoint> &p) {
            return p->point.cast<float>();
        };
    return VectorUtils::transform(support_points, transform_func);
}

void SampleIslandUtils::align_samples(SupportIslandPoints &samples,
                                      const ExPolygon &    island,
                                      const SampleConfig & config)
{
    bool exist_moveable = false;
    for (const auto &sample : samples) {
        if (sample->can_move()) {
            exist_moveable = true;
            break;
        }
    }
    if (!exist_moveable) return;

    size_t count_iteration = config.count_iteration; // copy
    coord_t max_move        = 0;
    while (--count_iteration > 1) {
        max_move = align_once(samples, island, config);        
        if (max_move < config.minimal_move) break;
    }

#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGNED_TO_SVG
    static int  counter = 0;
    SVG svg(("aligned_" + std::to_string(counter++) + ".svg").c_str(),BoundingBox(island));
    svg.draw(island);
    draw(svg, samples, config.head_radius);
    svg.Close();
    std::cout << "Align use " << config.count_iteration - count_iteration
            << " iteration and finish with precision " << unscale(max_move,0)[0] <<
            " mm" << std::endl;
#endif
    
}

bool is_points_in_distance(const Slic3r::Point & p,
                           const Slic3r::Points &points,
                           double                max_distance)
{
    for (const auto &p2 : points) {
        double d = (p - p2).cast<double>().norm();
        if (d > max_distance) return false;
    }
    return true;
}

coord_t SampleIslandUtils::align_once(SupportIslandPoints &samples,
                                      const ExPolygon &    island,
                                      const SampleConfig & config)
{
    using VD = Slic3r::Geometry::VoronoiDiagram;
    VD             vd;
    Slic3r::Points points = SampleIslandUtils::to_points(samples);
    coord_t max_move = 0;

#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
    std::string color_of_island = "#FF8080"; // LightRed. Should not be visible - cell color should overlap
    std::string color_point_cell = "lightgray"; // bigger than island but NOT self overlap
    std::string color_island_cell_intersection = "gray"; // Should full overlap island !!
    std::string color_old_point = "lightblue"; // Center of island cell intersection
    std::string color_wanted_point = "darkblue"; // Center of island cell intersection
    std::string color_new_point = "blue"; // Center of island cell intersection
    std::string color_static_point = "black";
    static int  counter = 0;
    BoundingBox bbox(island);
    
    SVG svg(("align_" + std::to_string(counter++) + ".svg").c_str(), bbox);
    svg.draw(island, color_of_island );
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG

    // create voronoi diagram with points
    construct_voronoi(points.begin(), points.end(), &vd);
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGNE_VD_TO_SVG
    static int vd_counter = 0;
    BoundingBox bbox(island);
    std::string name = "align_VD_" + std::to_string(vd_counter++) + ".svg";
    SVG svg(name.c_str(), bbox);
    svg.draw(island);
    for (const Point &point : points) {
        size_t index = &point - &points.front();
        svg.draw(point, "black", config.head_radius);
        svg.draw_text(point + Point(config.head_radius,0), std::to_string(index).c_str(), "black");
    }
    Lines island_lines = to_lines(island);
    svg.draw(island_lines, "blue");
    for (const auto &edge: vd.edges()) {
        std::optional<Line> line =
            VoronoiGraphUtils::to_line(edge, points, config.max_distance);
        if (!line.has_value()) continue;
        svg.draw(*line, "green", 1e6);
    }
    svg.Close();
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGNE_VD_TO_SVG
    size_t max_move_index = -1;
    for (const VD::cell_type &cell : vd.cells()) {
        SupportIslandPointPtr &sample = samples[cell.source_index()];

#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
        if (!sample->can_move()) {
            svg.draw(sample->point, color_static_point, config.head_radius);
            svg.draw_text(sample->point+Point(config.head_radius,0), SupportIslandPoint::to_string(sample->type).c_str(), color_static_point.c_str());
        }
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
        if (!sample->can_move()) continue;
        Polygon cell_polygon = VoronoiGraphUtils::to_polygon(cell, points, config.max_distance);
        Polygons intersections = Slic3r::intersection(island, ExPolygon(cell_polygon));
        const Polygon *island_cell   = nullptr;
        for (const Polygon &intersection : intersections) {
            if (intersection.contains(sample->point)) {
                island_cell = &intersection;
                break;
            }
        }
        assert(island_cell != nullptr);        
        Point center = island_cell->centroid();        
        /*{
            SVG cell_svg("island_cell.svg", island_cell->points);
            cell_svg.draw(cell_polygon, "lightgray");
            cell_svg.draw(points, "darkgray", config.head_radius);
            cell_svg.draw(*island_cell, "gray");
            cell_svg.draw(sample->point, "green", config.head_radius);
            cell_svg.draw(center, "black", config.head_radius);
        }*/
        assert(is_points_in_distance(center, island_cell->points, config.max_distance));
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
        svg.draw(cell_polygon, color_point_cell);
        svg.draw(*island_cell, color_island_cell_intersection);
        svg.draw(Line(sample->point, center), color_wanted_point, config.head_radius / 5);
        svg.draw(sample->point, color_old_point, config.head_radius);
        svg.draw(center, color_wanted_point, config.head_radius); // wanted position
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
        coord_t act_move = sample->move(center);
        if (max_move < act_move) {
            max_move = act_move; 
            max_move_index = cell.source_index();
        }
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
        svg.draw(sample->point, color_new_point, config.head_radius);        
        svg.draw_text(sample->point+Point(config.head_radius,0), SupportIslandPoint::to_string(sample->type).c_str(), color_new_point.c_str());
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
    }
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
    svg.Close();
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
    return max_move;
}

SupportIslandPoints SampleIslandUtils::sample_center_line(
    const VoronoiGraph::ExPath &path, const CenterLineConfiguration &cfg)
{
    const VoronoiGraph::Nodes &nodes = path.nodes;
    // like side branch separate first node from path
    VoronoiGraph::Path main_path({nodes.begin() + 1, nodes.end()},
                                 path.length);
    double start_offset        = cfg.sample_config.max_distance - cfg.sample_config.min_side_branch_length;
    SupportIslandPoints result = sample_side_branch(nodes.front(), main_path,
                                                    start_offset, cfg);

    if (path.circles.empty()) return result;
    sample_center_circles(path, cfg, result);
    
    return result;
}

void SampleIslandUtils::sample_center_circle_end(
    const VoronoiGraph::Node::Neighbor &neighbor,
    double &                            neighbor_distance,
    const VoronoiGraph::Nodes &         done_nodes,
    double &                            node_distance,
    const CenterLineConfiguration &     cfg,
    SupportIslandPoints &               result)
{
    double distance = neighbor_distance + node_distance + neighbor.length();
    double max_sample_distance = cfg.sample_config.max_distance;
    if (distance < max_sample_distance) { // no need add support point
        if (neighbor_distance > node_distance + neighbor.length())
            neighbor_distance = node_distance + neighbor.length();
        if (node_distance > neighbor_distance + neighbor.length())
            node_distance = neighbor_distance + neighbor.length();
        return;
    }
    size_t count_supports = static_cast<size_t>(
        std::floor(distance / max_sample_distance));
    // distance between support points
    double distance_between = distance / (count_supports + 1);
    if (distance_between < neighbor_distance) {
        // point is calculated to be in done path, SP will be on edge point
        result.push_back(create_no_move_point(
            &neighbor, 1., SupportIslandPoint::Type::center_circle_end));
        neighbor_distance = 0.;
        count_supports -= 1;
        if (count_supports == 0) {
            if (node_distance > neighbor.length())
                node_distance = neighbor.length();
            return;
        }
        distance         = node_distance + neighbor.length();
        distance_between = distance / (count_supports + 1);
    }
    VoronoiGraph::Nodes nodes = done_nodes; // copy, could be more neighbor
    nodes.insert(nodes.begin(), neighbor.node);
    for (size_t i = 1; i <= count_supports; ++i) {
        double distance_from_neighbor = i * distance_between - neighbor_distance;
        auto position = create_position_on_path(nodes, distance_from_neighbor);
        assert(position.has_value());
        result.push_back(
            create_no_move_point(*position, SupportIslandPoint::Type::center_circle_end2));
        double distance_support_to_node = fabs(neighbor.length() -
                                               distance_from_neighbor);
        if (node_distance > distance_support_to_node)
            node_distance = distance_support_to_node;
    }
}

// DTO store information about distance to nearest support point
// and path from start point
struct NodeDistance
{
    VoronoiGraph::Nodes nodes; // from act node to start
    double              distance_from_support_point;
    NodeDistance(const VoronoiGraph::Node *node,
                 double                    distance_from_support_point)
        : nodes({node})
        , distance_from_support_point(distance_from_support_point)
    {}
};

using SupportDistanceMap = std::map<const VoronoiGraph::Node*, double>;
double get_distance_to_support_point(const VoronoiGraph::Node *node,
                                     const SupportDistanceMap &       support_distance_map,
                                     double                    maximal_search)
{
    auto distance_item = support_distance_map.find(node);
    if (distance_item != support_distance_map.end())
        return distance_item->second;

    // wide search for nearest support point by neighbors
    struct Item
    {
        const VoronoiGraph::Node *prev_node;
        const VoronoiGraph::Node *node;
        double                    act_distance;
        bool                      exist_support_point;
        Item(const VoronoiGraph::Node *prev_node,
             const VoronoiGraph::Node *node,
             double                    act_distance,
             bool                      exist_support_point = false)
            : prev_node(prev_node)
            , node(node)
            , act_distance(act_distance)
            , exist_support_point(exist_support_point)
        {}
    };
    struct OrderDistanceFromNearest
    {
        bool operator()(const Item &first, const Item &second)
        {
            return first.act_distance > second.act_distance;
        }
    };
    std::priority_queue<Item, std::vector<Item>, OrderDistanceFromNearest> process;
    for (const VoronoiGraph::Node::Neighbor &neighbor : node->neighbors)
        process.emplace(node, neighbor.node, neighbor.length());
    
    while (!process.empty()) { 
        Item i = process.top();
        if (i.exist_support_point) return i.act_distance;
        process.pop();
        auto distance_item = support_distance_map.find(i.node);
        if (distance_item != support_distance_map.end()) {
            double distance = i.act_distance + distance_item->second;
            if (distance > maximal_search) continue;
            process.emplace(i.prev_node, i.node, distance, true);
            continue;
        }
        for (const VoronoiGraph::Node::Neighbor &neighbor :i.node->neighbors) {
            if (neighbor.node == i.prev_node) continue;
            double distance = i.act_distance + neighbor.length();
            if (distance > maximal_search) continue;
            process.emplace(i.node, neighbor.node, distance);
        }
    }
    return maximal_search;
}

SupportDistanceMap create_path_distances(
    const std::set<const VoronoiGraph::Node *> &circle_set,
    const std::set<const VoronoiGraph::Node *> &path_set,
    const SupportDistanceMap & support_distance_map,
    double                                      maximal_search)
{
    SupportDistanceMap path_distances;
    for (const VoronoiGraph::Node *node : circle_set) {        
        if (path_set.find(node) == path_set.end()) continue; // lay out of path
        path_distances[node] = get_distance_to_support_point(
            node, support_distance_map, maximal_search);
    }
    return path_distances;
}

// do not use
SupportDistanceMap create_support_distance_map(const SupportIslandPoints &support_points)
{
    SupportDistanceMap support_distance_map;
    for (const SupportIslandPointPtr &support_point : support_points) {
        auto ptr = dynamic_cast<SupportCenterIslandPoint*>(support_point.get()); // bad use
        const VoronoiGraph::Position &position = ptr->position;
        const VoronoiGraph::Node *node = position.neighbor->node; 
        const VoronoiGraph::Node *twin_node = VoronoiGraphUtils::get_twin_node(*position.neighbor);
        double distance = (1 - position.ratio) * position.neighbor->length();
        double twin_distance = position.ratio * position.neighbor->length();

        auto item = support_distance_map.find(node);
        if (item == support_distance_map.end()) { 
            support_distance_map[node] = distance;
        } else if (item->second > distance)
            item->second = distance;

        auto twin_item = support_distance_map.find(twin_node);
        if (twin_item == support_distance_map.end()) {
            support_distance_map[twin_node] = twin_distance;
        } else if (twin_item->second > twin_distance)
            twin_item->second = twin_distance;
    }

    return support_distance_map;
}

template<class T, class S, class C>
const S &get_container_ref(const std::priority_queue<T, S, C> &q)
{
    struct HackedQueue : private std::priority_queue<T, S, C>
    {
        static const S &Container(const std::priority_queue<T, S, C> &q)
        {
            return q.*&HackedQueue::c;
        }
    };
    return HackedQueue::Container(q);
}

std::set<const VoronoiGraph::Node *> create_path_set(
    const VoronoiGraph::ExPath &path)
{
    std::queue<const VoronoiGraph::Node *> side_branch_nodes;
    std::set<const VoronoiGraph::Node *> path_set;
    for (const VoronoiGraph::Node *node : path.nodes) {
        path_set.insert(node);
        auto side_branch_item = path.side_branches.find(node);
        if (side_branch_item == path.side_branches.end()) continue;
        side_branch_nodes.push(node);
    }
    while (!side_branch_nodes.empty()) {
        const VoronoiGraph::Node *node = side_branch_nodes.front();
        side_branch_nodes.pop();
        auto side_branch_item = path.side_branches.find(node);
        const std::vector<VoronoiGraph::Path> &side_branches =
            get_container_ref(side_branch_item->second);
        for (const VoronoiGraph::Path& side_branch : side_branches)
            for (const VoronoiGraph::Node *node : side_branch.nodes) {
                path_set.insert(node);
                auto side_branch_item = path.side_branches.find(node);
                if (side_branch_item == path.side_branches.end()) continue;
                side_branch_nodes.push(node);
            }
    }
    return path_set;
}

void SampleIslandUtils::sample_center_circles(
    const VoronoiGraph::ExPath &   path,
    const CenterLineConfiguration &cfg,
    SupportIslandPoints &          result)
{
    // vector of connected circle points
    // for detection path from circle
    std::vector<std::set<const VoronoiGraph::Node *>> circles_sets =
        create_circles_sets(path.circles, path.connected_circle);
    std::set<const VoronoiGraph::Node *> path_set = create_path_set(path);
    SupportDistanceMap support_distance_map = create_support_distance_map(result);
    double half_sample_distance = cfg.sample_config.max_distance / 2.;
    for (const auto &circle_set : circles_sets) {
        SupportDistanceMap path_distances =
            create_path_distances(circle_set, path_set, support_distance_map,
                                  half_sample_distance);
        SupportIslandPoints circle_result = sample_center_circle(circle_set, path_distances, cfg);
        result.insert(result.end(), 
            std::make_move_iterator(circle_result.begin()),
            std::make_move_iterator(circle_result.end()));
    }
}

SupportIslandPoints SampleIslandUtils::sample_center_circle(
    const std::set<const VoronoiGraph::Node *> &  circle_set,
    std::map<const VoronoiGraph::Node *, double> &path_distances,
    const CenterLineConfiguration &               cfg)
{
    SupportIslandPoints result;
    // depth search
    std::stack<NodeDistance> process;

    // path_nodes are already sampled
    for (const auto &path_distanc : path_distances) {
        process.push(NodeDistance(path_distanc.first, path_distanc.second));
    }

    // when node is sampled in all side branches.
    // Value is distance to nearest support point
    std::map<const VoronoiGraph::Node *, double> dones;
    while (!process.empty()) {
        NodeDistance nd = process.top(); // copy
        process.pop();
        const VoronoiGraph::Node *node               = nd.nodes.front();
        const VoronoiGraph::Node *prev_node          = (nd.nodes.size() > 1) ?
                                                           nd.nodes[1] :
                                                           nullptr;
        auto                      done_distance_item = dones.find(node);
        if (done_distance_item != dones.end()) {
            if (done_distance_item->second > nd.distance_from_support_point)
                done_distance_item->second = nd.distance_from_support_point;
            continue;
        }
        // sign node as done with distance to nearest support
        dones[node]                = nd.distance_from_support_point;
        double &node_distance      = dones[node]; // append to done node
        auto    path_distance_item = path_distances.find(node);
        bool is_node_on_path = (path_distance_item != path_distances.end());
        if (is_node_on_path && node_distance > path_distance_item->second)
            node_distance = path_distance_item->second;
        for (const auto &neighbor : node->neighbors) {
            if (neighbor.node == prev_node) continue;
            if (circle_set.find(neighbor.node) == circle_set.end())
                continue; // out of circle points
            auto path_distance_item  = path_distances.find(neighbor.node);
            bool is_neighbor_on_path = (path_distance_item !=
                                        path_distances.end());
            if (is_node_on_path && is_neighbor_on_path)
                continue; // already sampled

            auto neighbor_done_item = dones.find(neighbor.node);
            bool is_neighbor_done   = neighbor_done_item != dones.end();
            if (is_neighbor_done || is_neighbor_on_path) {
                double &neighbor_distance = (is_neighbor_done) ?
                                                neighbor_done_item->second :
                                                path_distance_item->second;
                sample_center_circle_end(neighbor, neighbor_distance,
                                         nd.nodes, node_distance, cfg,
                                         result);
                continue;
            }

            NodeDistance next_nd = nd; // copy
            next_nd.nodes.insert(next_nd.nodes.begin(), neighbor.node);
            next_nd.distance_from_support_point += neighbor.length();
            // exist place for sample:
            double max_sample_distance = cfg.sample_config.max_distance;
            while (next_nd.distance_from_support_point > max_sample_distance) {
                double distance_from_node = next_nd
                                                .distance_from_support_point -
                                            nd.distance_from_support_point;
                double ratio = distance_from_node / neighbor.length();              
                result.push_back(std::make_unique<SupportCenterIslandPoint>(
                    VoronoiGraph::Position(&neighbor, ratio), 
                    &cfg.sample_config,
                    SupportIslandPoint::Type::center_circle));
                next_nd.distance_from_support_point -= max_sample_distance;
            }
            process.push(next_nd);
        }
    }
    return result;
}

SupportIslandPoints SampleIslandUtils::sample_voronoi_graph(
    const VoronoiGraph &  graph,
    const Lines &         lines,
    const SampleConfig &  config,
    VoronoiGraph::ExPath &longest_path)
{
    const VoronoiGraph::Node *start_node =
        VoronoiGraphUtils::getFirstContourNode(graph);
    // every island has to have a point on contour
    assert(start_node != nullptr);
    longest_path = VoronoiGraphUtils::create_longest_path(start_node);
    // longest_path = create_longest_path_recursive(start_node);

#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_VORONOI_GRAPH_TO_SVG
    {
        static int counter = 0;
        SVG        svg("voronoiGraph" + std::to_string(counter++) + ".svg",
                LineUtils::create_bounding_box(lines));
        VoronoiGraphUtils::draw(svg, graph, lines, 1e6, true);
    }
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_VORONOI_GRAPH_TO_SVG

    SupportIslandPoints points = sample_expath(longest_path, lines, config);

#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_INITIAL_SAMPLE_POSITION_TO_SVG
    {
        static int counter = 0;
        SVG svg("initial_sample_positions" + std::to_string(counter++) + ".svg",
                LineUtils::create_bounding_box(lines));
        svg.draw(lines, "gray", config.head_radius/ 10);
        draw(svg, points, config.head_radius, "black", true);
    }
#endif // SLA_SAMPLE_ISLAND_UTILS_STORE_INITIAL_SAMPLE_POSITION_TO_SVG
    return points;
}

SupportIslandPoints SampleIslandUtils::sample_expath(
    const VoronoiGraph::ExPath &path,
    const Lines &               lines,
    const SampleConfig &        config)
{
    // 1) One support point
    if (path.length < config.max_length_for_one_support_point) {
        // create only one point in center
        SupportIslandPoints result;
        result.push_back(create_middle_path_point(
            path, SupportIslandPoint::Type::one_center_point));
        return result;
    }

    double max_width = VoronoiGraphUtils::get_max_width(path);
    if (max_width < config.max_width_for_center_support_line) {
        // 2) Two support points
        if (path.length < config.max_length_for_two_support_points) {
            coord_t max_distance =
                std::min(config.half_distance, static_cast<coord_t>(path.length / 2));
            return create_side_points(path.nodes, lines,
                                      2 * config.minimal_distance_from_outline,
                                      max_distance);
        }

        // othewise sample path
        /*CenterLineConfiguration
            centerLineConfiguration(path.side_branches, config);
        SupportIslandPoints samples = sample_center_line(path, centerLineConfiguration);
        samples.front()->type = SupportIslandPoint::Type::center_line_end2;
        return samples;*/
    }

    // TODO: 3) Triangle of points
    // eval outline and find three point create almost equilateral triangle

    // IMPROVE: Erase continous sampling: Extract ExPath and than sample uniformly whole ExPath
    CenterStarts center_starts;
    const VoronoiGraph::Node *start_node = path.nodes.front();
    // CHECK> Front of path is outline node
    assert(start_node->neighbors.size() == 1);
    const VoronoiGraph::Node::Neighbor *neighbor = &start_node->neighbors.front();
    std::set<const VoronoiGraph::Node *> done; // already done nodes
    SupportIslandPoints points; // result
    if (neighbor->max_width() < config.max_width_for_center_support_line) {
        // start sample center
        coord_t width = 2 * config.minimal_distance_from_outline;
        coord_t distance = config.maximal_distance_from_outline;
        auto position = create_position_on_path(path.nodes, lines, width, distance);
        if (position.has_value()) {
            points.push_back(create_no_move_point(
                *position, SupportIslandPoint::Type::center_line_start));
            // move nodes to done set 
            VoronoiGraph::Nodes start_path;
            for (const auto &node : path.nodes) {
                if (node == position->neighbor->node) break;
                done.insert(node);
                start_path.push_back(node);
            }
            coord_t already_supported = position->calc_distance();
            coord_t support_in = config.max_distance + already_supported;                                 
            center_starts.emplace_back(position->neighbor, support_in, start_path);
        } else {
            assert(position.has_value());
            done.insert(start_node);
            coord_t support_in = config.minimal_distance_from_outline;
            center_starts.emplace_back(neighbor, support_in);
        }
        // IMPROVE: check side branches on start path
    } else {
        // start sample field
        VoronoiGraph::Position field_start =
            VoronoiGraphUtils::get_position_with_width(
                neighbor, config.min_width_for_outline_support, lines);
        sample_field(field_start, points, center_starts, done, lines, config);
    }

    // Main loop of sampling
    std::optional<VoronoiGraph::Position> field_start = 
        sample_center(center_starts, done, points, lines, config);
    while (field_start.has_value()) {
        sample_field(*field_start, points, center_starts, done, lines, config);
        field_start = sample_center(center_starts, done, points, lines, config);
    }
    return points;
}

void SampleIslandUtils::sample_field(VoronoiGraph::Position &field_start,
                                     SupportIslandPoints &   points,
                                     CenterStarts &          center_starts,
                                     std::set<const VoronoiGraph::Node *> &done,
                                     const Lines &       lines,
                                     const SampleConfig &config)
{
    auto field = create_field(field_start, center_starts, done, lines, config);
    SupportIslandPoints outline_support = sample_outline(field, config);
    points.insert(points.end(), std::move_iterator(outline_support.begin()),
                  std::move_iterator(outline_support.end()));

    // Erode island to not sampled island around border,
    // minimal value must be -config.minimal_distance_from_outline
    Polygons polygons = offset(field.border,
                               -2.f * config.minimal_distance_from_outline,
                               ClipperLib::jtSquare);
    if (polygons.empty()) return;
    auto inner = std::make_shared<ExPolygon>(field.inner);    
    Points inner_points = sample_expolygon(*inner, config.max_distance);    
    std::transform(inner_points.begin(), inner_points.end(), std::back_inserter(points), 
        [&](const Point &point) { 
            return std::make_unique<SupportIslandInnerPoint>(
                           point, inner, SupportIslandPoint::Type::inner);
        });
}

std::optional<VoronoiGraph::Position> SampleIslandUtils::sample_center(
    CenterStarts &                        new_starts,
    std::set<const VoronoiGraph::Node *> &done,
    SupportIslandPoints &                 results,
    const Lines &                         lines,
    const SampleConfig &                  config)
{
    const VoronoiGraph::Node::Neighbor *neighbor = nullptr;
    VoronoiGraph::Nodes                 path;
    coord_t                             support_in;
    bool use_new_start = true;
    bool is_continous = false;

    while (use_new_start || neighbor->max_width() <= config.max_width_for_center_support_line) {
        // !! do not check max width for new start, it could be wide to tiny change
        if (use_new_start) { 
            use_new_start = false;
            // skip done starts
            if (new_starts.empty()) return {}; // no start
            while (done.find(new_starts.back().neighbor->node) != done.end()) {
                new_starts.pop_back();
                if (new_starts.empty()) return {};
            }
            // fill new start
            const CenterStart & new_start = new_starts.back();
            neighbor = new_start.neighbor;
            path = new_start.path; // copy
            support_in = new_start.support_in;
            new_starts.pop_back();
            is_continous = false;
        }

        // add support on actual neighbor edge
        coord_t edge_length = static_cast<coord_t>(neighbor->length());
        while (edge_length >= support_in) {
            double ratio = support_in / neighbor->length();
            VoronoiGraph::Position position(neighbor, ratio);
            results.push_back(std::make_unique<SupportCenterIslandPoint>(
                position, &config, SupportIslandPoint::Type::center_line1));
            support_in += config.max_distance;
            is_continous = true;
        }
        support_in -= edge_length;

        const VoronoiGraph::Node *node = neighbor->node;
        done.insert(node);
        // IMPROVE: A) limit length of path to config.minimal_support_distance
        // IMPROVE: B) store node in reverse order
        path.push_back(node);
        const VoronoiGraph::Node::Neighbor *next_neighbor = nullptr;
        for (const auto &node_neighbor : node->neighbors) {
            if (done.find(node_neighbor.node) != done.end()) continue;
            if (next_neighbor == nullptr) {
                next_neighbor = &node_neighbor;
                continue;
            }
            new_starts.emplace_back(&node_neighbor, support_in, path); // search in side branch
        }

        if (next_neighbor == nullptr) {
            if (neighbor->min_width() != 0) {
                std::reverse(path.begin(), path.end());
                auto position_opt = create_position_on_path(path, support_in / 2);
                if (position_opt.has_value()) {
                    results.push_back(
                        std::make_unique<SupportCenterIslandPoint>(
                            *position_opt, &config,
                            SupportIslandPoint::Type::center_line3));                    
                }
            } else {
                // no neighbor to continue
                create_sample_center_end(*neighbor, is_continous, path,
                                         support_in, lines, results,
                                         new_starts, config);
            }
            use_new_start = true;
        } else {
            neighbor = next_neighbor;
        }
    }

    // create field start
    auto result = VoronoiGraphUtils::get_position_with_width(
            neighbor, config.min_width_for_outline_support, lines);

    // sample rest of neighbor before field
    double edge_length = neighbor->length();
    double sample_length = edge_length * result.ratio;
    while (sample_length > support_in) {
        double ratio = support_in / edge_length;
        VoronoiGraph::Position position(neighbor, ratio);
        results.push_back(std::make_unique<SupportCenterIslandPoint>(
            position, &config, SupportIslandPoint::Type::center_line2));
        support_in += config.max_distance;
    }
    return result;
}

void SampleIslandUtils::create_sample_center_end(
    const VoronoiGraph::Node::Neighbor &neighbor,
    bool                                is_continous,
    const VoronoiGraph::Nodes &         path,
    coord_t                             support_in,
    const Lines &                       lines,
    SupportIslandPoints &               results,
    CenterStarts &                      new_starts,
    const SampleConfig &                config)
{
    // last neighbor?
    if (neighbor.min_width() != 0) return;

    // sharp corner?
    double angle = VoronoiGraphUtils::outline_angle(neighbor, lines);
    if (angle > config.max_interesting_angle) return;

    // exist place for support?
    VoronoiGraph::Nodes path_reverse = path; // copy
    std::reverse(path_reverse.begin(), path_reverse.end());
    coord_t width        = 2 * config.minimal_distance_from_outline;
    coord_t distance     = config.maximal_distance_from_outline;
    auto    position_opt = create_position_on_path(path_reverse, lines, width, distance);
    if (!position_opt.has_value()) return;
    
    // check if exist popable result
    if (is_continous && config.max_distance < (support_in + distance)) {
        // one support point should be enough
        // when max_distance > maximal_distance_from_outline
        results.pop_back(); // remove support point
    }

    create_sample_center_end(*position_opt, results, new_starts, config);
}

void SampleIslandUtils::create_sample_center_end(
    const VoronoiGraph::Position &position,
    SupportIslandPoints &         results,
    CenterStarts &                new_starts,
    const SampleConfig &          config)
{
    const SupportIslandPoint::Type no_move_type =
        SupportIslandPoint::Type::center_line_end3;
    const coord_t minimal_support_distance = config.minimal_support_distance;
    Point         point = VoronoiGraphUtils::create_edge_point(position);
    // raw pointers are function scope ONLY
    std::vector<const SupportIslandPoint *> near_no_move;
    for (const auto &res : results) {
        if (res->type != no_move_type) continue;
        Point diff = point - res->point;
        if (abs(diff.x()) > minimal_support_distance) continue;
        if (abs(diff.y()) > minimal_support_distance) continue;
        // create raw pointer, used only in function scope
        near_no_move.push_back(&*res); 
    }

    std::map<const VoronoiGraph::Node::Neighbor *, coord_t> distances;
    std::function<void(const VoronoiGraph::Node::Neighbor &, coord_t)>
        collect_distances = [&](const auto &neighbor, coord_t act_distance) {
            distances[&neighbor] = act_distance;
        };
    VoronoiGraphUtils::for_neighbor_at_distance(position, minimal_support_distance, collect_distances);
    
    bool exist_no_move = false;
    if (!near_no_move.empty()) {
        for (const auto &item : distances) {
            const VoronoiGraph::Node::Neighbor &neighbor = *item.first;
            // TODO: create belongs for parabola, when start sampling at parabola
            Line edge(VoronoiGraphUtils::to_point(neighbor.edge->vertex0()),
                      VoronoiGraphUtils::to_point(neighbor.edge->vertex1()));
            for (const auto &support_point : near_no_move) {
                if (LineUtils::belongs(edge, support_point->point, 10000)) {
                    exist_no_move = true;
                    break;
                }
            }
            if (exist_no_move) break;
        }
    }

    if (!exist_no_move) {
        // fix value of support_in
        // for new_starts in sampled path
        // by distance to position
        for (CenterStart &new_start : new_starts) {
            auto item = distances.find(new_start.neighbor);
            if (item != distances.end()) {
                coord_t support_distance = item->second;
                coord_t new_support_in   = config.max_distance - item->second;
                new_start.support_in     = std::max(new_start.support_in,
                                                new_support_in);
            } else {
                const VoronoiGraph::Node::Neighbor *twin =
                    VoronoiGraphUtils::get_twin(*new_start.neighbor);
                auto item = distances.find(twin);
                if (item != distances.end()) {
                    coord_t support_distance = item->second + twin->length();
                    coord_t new_support_in   = config.max_distance -
                                             support_distance;
                    new_start.support_in = std::max(new_start.support_in,
                                                    new_support_in);
                }
            }
        }
        results.push_back(
            std::make_unique<SupportIslandNoMovePoint>(point, no_move_type));
    }
}


SampleIslandUtils::Field SampleIslandUtils::create_field(
    const VoronoiGraph::Position & field_start,
    CenterStarts &    tiny_starts,
    std::set<const VoronoiGraph::Node *> &tiny_done,
    const Lines &     lines,
    const SampleConfig &config)
{
    using VD = Slic3r::Geometry::VoronoiDiagram;
    const coord_t min_width = config.min_width_for_outline_support;

    // DTO represents one island change from wide to tiny part
    // it is stored inside map under source line index
    struct WideTinyChange{
        // new coordinate for line.b point
        Point new_b;
        // new coordinate for next line.a point
        Point next_new_a;
        // index to lines
        size_t next_line_index;

        WideTinyChange(Point new_b, Point next_new_a, size_t next_line_index)
            : new_b(new_b)
            , next_new_a(next_new_a)
            , next_line_index(next_line_index)
        {}

        // is used only when multi wide tiny change are on same Line
        struct SortFromAToB
        {
            LineUtils::SortFromAToB compare;
            SortFromAToB(const Line &line) : compare(line) {}            
            bool operator()(const WideTinyChange &left,
                            const WideTinyChange &right)
            {
                return compare.compare(left.new_b, right.new_b);
            }
        };
    };
    using WideTinyChanges = std::vector<WideTinyChange>;

    // store shortening of outline segments
    //   line index, vector<next line index + 2x shortening points>
    std::map<size_t, WideTinyChanges> wide_tiny_changes;

    coord_t minimal_edge_length = std::max(config.max_distance / 2, 2*config.minimal_distance_from_outline);
    // cut lines at place where neighbor has width = min_width_for_outline_support
    // neighbor must be in direction from wide part to tiny part of island
    auto add_wide_tiny_change =
        [&](const VoronoiGraph::Position &position,
            const VoronoiGraph::Node *    source_node)->bool {
        const VoronoiGraph::Node::Neighbor *neighbor = position.neighbor;

        // IMPROVE: check not only one neighbor but all path to edge
        coord_t rest_size = static_cast<coord_t>(neighbor->length() * (1. - position.ratio));
        if (VoronoiGraphUtils::is_last_neighbor(neighbor) &&
            rest_size <= minimal_edge_length)
            return false; // no change only rich outline

        // function to add sorted change from wide to tiny
        // stored uder line index or line shorten in point b
        auto add = [&](const Point &p1, const Point &p2, size_t i1,
                        size_t i2) {
            WideTinyChange change(p1, p2, i2);
            auto           item = wide_tiny_changes.find(i1);
            if (item == wide_tiny_changes.end()) {
                wide_tiny_changes[i1] = {change};
            } else {
                WideTinyChange::SortFromAToB pred(lines[i1]);
                VectorUtils::insert_sorted(item->second, change, pred);
            }
        };

        Point p1, p2;
        std::tie(p1, p2) = VoronoiGraphUtils::point_on_lines(position, lines);
        const VD::edge_type *edge = neighbor->edge;
        size_t               i1   = edge->cell()->source_index();
        size_t               i2   = edge->twin()->cell()->source_index();

        const Line &l1 = lines[i1];
        if (VoronoiGraphUtils::is_opposit_direction(edge, l1)) {
            // line1 is shorten on side line1.a --> line2 is shorten on
            // side line2.b
            add(p2, p1, i2, i1);
        } else {
            // line1 is shorten on side line1.b
            add(p1, p2, i1, i2);
        }
        coord_t support_in = neighbor->length() * position.ratio + config.max_distance/2;
        CenterStart tiny_start(neighbor, support_in, {source_node});
        tiny_starts.push_back(tiny_start);
        tiny_done.insert(source_node);
        return true;
    };

    const VoronoiGraph::Node::Neighbor *tiny_wide_neighbor = field_start.neighbor;
    const VoronoiGraph::Node::Neighbor *wide_tiny_neighbor = VoronoiGraphUtils::get_twin(*tiny_wide_neighbor);
    VoronoiGraph::Position position(wide_tiny_neighbor, 1. - field_start.ratio);
    add_wide_tiny_change(position, tiny_wide_neighbor->node);

    std::set<const VoronoiGraph::Node*> done;
    done.insert(wide_tiny_neighbor->node);
    //                                    prev node         ,           node
    using ProcessItem = std::pair<const VoronoiGraph::Node *, const VoronoiGraph::Node *>;
    // initial proccess item from tiny part to wide part of island
    ProcessItem start(wide_tiny_neighbor->node, tiny_wide_neighbor->node);
    std::queue<ProcessItem> process;
    process.push(start);

    // all lines belongs to polygon
    std::set<size_t> field_line_indexes;
    while (!process.empty()) { 
        const ProcessItem &item = process.front();
        const VoronoiGraph::Node *node = item.second;
        const VoronoiGraph::Node *prev_node = item.first;
        process.pop();
        if (done.find(node) != done.end()) continue;
        do {
            done.insert(node);
            const VoronoiGraph::Node *next_node = nullptr;
            for (const VoronoiGraph::Node::Neighbor &neighbor: node->neighbors) {
                if (neighbor.node == prev_node) continue; 
                const VD::edge_type *edge = neighbor.edge;
                size_t index1 = edge->cell()->source_index();
                size_t index2 = edge->twin()->cell()->source_index();
                field_line_indexes.insert(index1);
                field_line_indexes.insert(index2);
                if (neighbor.min_width() < min_width) {
                    VoronoiGraph::Position position =
                        VoronoiGraphUtils::get_position_with_width(
                            &neighbor, min_width, lines);
                    if(add_wide_tiny_change(position, node))
                        continue;
                }
                if (done.find(neighbor.node) != done.end()) continue; // loop back
                if (next_node == nullptr) { 
                    next_node = neighbor.node;
                } else {
                    process.push({node, neighbor.node});
                }
            }
            prev_node = node;
            node      = next_node;
        } while (node != nullptr);
    }
    
    // connection of line on island
    std::map<size_t, size_t> b_connection =
        LineUtils::create_line_connection_over_b(lines);

    std::vector<size_t> source_indexes;
    auto inser_point_b = [&](size_t& index, Points& points, std::set<size_t>& done)
    {
        const Line &line = lines[index];
        points.push_back(line.b);
        const auto &connection_item = b_connection.find(index);
        assert(connection_item != b_connection.end());
        done.insert(index);
        index = connection_item->second;
        source_indexes.push_back(index);
    };

    size_t source_indexe_for_change = lines.size();
    auto insert_changes = [&](size_t &index, Points &points, std::set<size_t> &done, size_t input_index)->bool {
        bool is_first    = points.empty();
        auto change_item = wide_tiny_changes.find(index);
        while (change_item != wide_tiny_changes.end()) {
            const WideTinyChanges &changes = change_item->second;
            assert(!changes.empty());
            size_t change_index = 0;
            if (!points.empty()) {
                const Point &           last_point = points.back();
                LineUtils::SortFromAToB pred(lines[index]);
                bool                    no_change = false;
                while (pred.compare(changes[change_index].new_b, last_point)) {
                    ++change_index;
                    if (change_index >= changes.size()) {
                        no_change = true;
                        break;
                    }
                }
                if (no_change) break;

                // Field ends with change into first index
                if (!is_first && change_item->first == input_index &&
                    change_index == 0) {
                    return false;
                } else {
                    is_first = false;
                }
            }
            const WideTinyChange &change = changes[change_index];
            // prevent double points
            if (points.empty() ||
                !PointUtils::is_equal(points.back(), change.new_b)) {
                points.push_back(change.new_b);
                source_indexes.push_back(source_indexe_for_change);
            } else {
                source_indexes.back() = source_indexe_for_change;
            }
            // prevent double points
            if (!PointUtils::is_equal(lines[change.next_line_index].b,
                                      change.next_new_a)) {
                points.push_back(change.next_new_a);
                source_indexes.push_back(change.next_line_index);
            }
            done.insert(index);
            index = change.next_line_index;
            change_item = wide_tiny_changes.find(index);
        }
        return true;
    };
    
    Points points;
    points.reserve(field_line_indexes.size());
    std::vector<size_t> outline_indexes;
    outline_indexes.reserve(field_line_indexes.size());
    size_t input_index1 = tiny_wide_neighbor->edge->cell()->source_index();
    size_t input_index2 = tiny_wide_neighbor->edge->twin()->cell()->source_index();
    size_t input_index  = std::min(input_index1, input_index2);
    size_t outline_index = input_index;
    std::set<size_t> done_indexes;
    do {
        if (!insert_changes(outline_index, points, done_indexes, input_index))
            break;        
        inser_point_b(outline_index, points, done_indexes);
    } while (outline_index != input_index);

    Field field;
    field.border.contour = Polygon(points);
    // finding holes
    if (done_indexes.size() < field_line_indexes.size()) {
        for (const size_t &index : field_line_indexes) {
            if(done_indexes.find(index) != done_indexes.end()) continue;
            // new  hole
            Points hole_points;
            size_t hole_index = index;
            do {
                inser_point_b(hole_index, hole_points, done_indexes);
            } while (hole_index != index);
            field.border.holes.emplace_back(hole_points);
        }
    }
    field.source_indexe_for_change = source_indexe_for_change;
    field.source_indexes = std::move(source_indexes);
    std::tie(field.inner, field.field_2_inner) =
        outline_offset(field.border, config.minimal_distance_from_outline);

#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_FIELD_TO_SVG
    {
        const char *source_line_color = "black";
        bool draw_source_line_indexes = true;
        bool draw_border_line_indexes = false;
        bool draw_field_source_indexes = true;
        static int  counter   = 0;
        std::string file_name = "field_" + std::to_string(counter++) + ".svg";

        SVG svg(file_name, LineUtils::create_bounding_box(lines));
        LineUtils::draw(svg, lines, source_line_color, 0., draw_source_line_indexes);
        draw(svg, field, draw_border_line_indexes, draw_field_source_indexes);
    }
#endif //SLA_SAMPLE_ISLAND_UTILS_STORE_FIELD_TO_SVG
    return field;
}

std::pair<Slic3r::ExPolygon, std::map<size_t, size_t>>
SampleIslandUtils::outline_offset(const Slic3r::ExPolygon &island,
                                  coord_t                  offset_distance)
{
    Polygons polygons = offset(island, -offset_distance, ClipperLib::jtSquare);
    if (polygons.empty()) return {}; // no place for support point
    assert(polygons.front().is_counter_clockwise());
    ExPolygon offseted(polygons.front());
    for (size_t i = 1; i < polygons.size(); ++i) {
        Polygon &hole = polygons[i];
        assert(hole.is_clockwise());
        offseted.holes.push_back(hole);
    }

    // TODO: Connect indexes for convert during creation of offset
    // !! this implementation was fast for develop BUT NOT for running !!
    const double angle_tolerace = 1e-4;
    const double distance_tolerance = 20.;
    Lines island_lines = to_lines(island);
    Lines offset_lines = to_lines(offseted);
    // Convert index map from island index to offseted index
    std::map<size_t, size_t> converter;
    for (size_t island_line_index = 0; island_line_index < island_lines.size(); ++island_line_index) {
        const Line &island_line = island_lines[island_line_index];
        Vec2d dir1 = LineUtils::direction(island_line).cast<double>();
        dir1.normalize();
        for (size_t offset_line_index = 0; offset_line_index < offset_lines.size(); ++offset_line_index) {
            const Line &offset_line = offset_lines[offset_line_index];
            Vec2d dir2 = LineUtils::direction(offset_line).cast<double>();
            dir2.normalize();
            double  angle    = acos(dir1.dot(dir2));
            // not similar direction
            
            if (fabs(angle) > angle_tolerace) continue;

            Point offset_middle = LineUtils::middle(offset_line);
            Point island_middle = LineUtils::middle(island_line);
            Point diff_middle   = offset_middle - island_middle;
            if (fabs(diff_middle.x()) > 2 * offset_distance ||
                fabs(diff_middle.y()) > 2 * offset_distance) continue;

            double distance = island_line.perp_distance_to(offset_middle);
            if (fabs(distance - offset_distance) > distance_tolerance)
                continue;

            // found offseted line
            converter[island_line_index] = offset_line_index;
            break;            
        }
    }

    return {offseted, converter};
}

SupportIslandPoints SampleIslandUtils::sample_outline(
    const Field &field, const SampleConfig &config)
{
    const ExPolygon &border        = field.border;
    const Polygon &contour = border.contour;
    assert(field.source_indexes.size() >= contour.size());
    coord_t max_align_distance = config.max_align_distance;
    coord_t sample_distance = config.outline_sample_distance;
    SupportIslandPoints result;

    using RestrictionPtr = std::shared_ptr<SupportOutlineIslandPoint::Restriction>;
    auto add_sample = [&](size_t index, const RestrictionPtr& restriction, coord_t &last_support) {
        using Position = SupportOutlineIslandPoint::Position;
        const Line &  line = restriction->lines[index];
        const double &line_length_double = restriction->lengths[index];
        coord_t line_length = static_cast<coord_t>(std::round(line_length_double));
        if (last_support + line_length > sample_distance) {
            Point direction = LineUtils::direction(line);
            do {
                double ratio = (sample_distance - last_support) / line_length_double;
                result.emplace_back(
                    std::make_unique<SupportOutlineIslandPoint>(
                        Position(index, ratio), restriction,
                        SupportIslandPoint::Type::outline)
                );
                last_support -= sample_distance;
            } while (last_support + line_length > sample_distance);
        }
        last_support += line_length;
    };
    auto add_circle_sample = [&](const Polygon& polygon) {
        // IMPROVE: find interesting points to start sampling
        Lines lines = to_lines(polygon);
        std::vector<double> lengths;
        lengths.reserve(lines.size());
        double sum_lengths = 0;
        for (const Line &line : lines) {
            double length = line.length();
            sum_lengths += length;
            lengths.push_back(length);
        }
        // no samples on this polygon

        using Restriction = SupportOutlineIslandPoint::RestrictionCircleSequence;
        auto restriction  = std::make_shared<Restriction>(lines, lengths, max_align_distance);
        coord_t last_support = std::min(static_cast<coord_t>(sum_lengths), sample_distance) / 2;
        for (size_t index = 0; index < lines.size(); ++index) {
            add_sample(index, restriction, last_support);
        }
    };

    // sample line sequence
    auto add_lines_samples = [&](const Lines &inner_lines,
                                 size_t       first_index,
                                 size_t       last_index) {
        ++last_index; // index after last item
        Lines lines;
        // is over start ?
        if (first_index > last_index) {
            size_t count = last_index + (inner_lines.size() - first_index);
            lines.reserve(count);
            std::copy(inner_lines.begin() + first_index,
                        inner_lines.end(),
                        std::back_inserter(lines));
            std::copy(inner_lines.begin(),
                      inner_lines.begin() + last_index,
                        std::back_inserter(lines));
        } else {
            size_t count = last_index - first_index;
            lines.reserve(count);
            std::copy(inner_lines.begin() + first_index,
                      inner_lines.begin() + last_index,
                      std::back_inserter(lines));
        }

        // IMPROVE: find interesting points to start sampling
        std::vector<double> lengths;
        lengths.reserve(lines.size());
        double sum_lengths = 0;
        for (const Line &line : lines) { 
            double length = line.length();
            sum_lengths += length;
            lengths.push_back(length);
        }
        
        using Restriction = SupportOutlineIslandPoint::RestrictionLineSequence;
        auto restriction = std::make_shared<Restriction>(lines, lengths, max_align_distance);
        
        // CHECK: Is correct to has always one support on outline sequence? 
        // or no sample small sequence at all?
        coord_t last_support = std::min(static_cast<coord_t>(sum_lengths), sample_distance) / 2;
        for (size_t index = 0; index < lines.size(); ++index) { 
            add_sample(index, restriction, last_support);
        }
    };

    
    // convert map from field index to inner(line index)
    const std::map<size_t, size_t>& field_2_inner = field.field_2_inner;
    const size_t& change_index = field.source_indexe_for_change;
    auto sample_polygon = [&](const Polygon &polygon,
                              const Polygon &inner_polygon,
                              size_t         index_offset) {
        // contain polygon tiny wide change?
        size_t first_change_index = polygon.size();
        for (size_t polygon_index = 0; polygon_index < polygon.size(); ++polygon_index) {
            size_t index = polygon_index + index_offset;
            assert(index < field.source_indexes.size());
            size_t source_index = field.source_indexes[index];
            if (source_index == change_index) {
                // found change from wide to tiny part
                first_change_index = polygon_index;
                break;
            }
        }

        // is polygon without change
        if (first_change_index == polygon.size()) {
            add_circle_sample(inner_polygon);
        } else { // exist change create line sequences
            // initialize with non valid values
            Lines  inner_lines   = to_lines(inner_polygon);
            size_t inner_invalid = inner_lines.size();
            // first and last index to inner lines
            size_t inner_first = inner_invalid;
            size_t inner_last  = inner_invalid;
            size_t stop_index  = first_change_index;
            if (stop_index == 0)
                stop_index = polygon.size();
            for (size_t polygon_index = first_change_index + 1;
                 polygon_index != stop_index; ++polygon_index) {
                if (polygon_index == polygon.size()) polygon_index = 0;
                size_t index = polygon_index + index_offset;
                assert(index < field.source_indexes.size());
                size_t source_index = field.source_indexes[index];
                if (source_index == change_index) {
                    if (inner_first == inner_invalid) continue;
                    // create Restriction object
                    add_lines_samples(inner_lines, inner_first, inner_last);
                    inner_first = inner_invalid;
                    inner_last  = inner_invalid;
                    continue;
                }
                auto convert_index_item = field_2_inner.find(index);
                // check if exist inner line
                if (convert_index_item == field_2_inner.end()) continue;
                inner_last = convert_index_item->second - index_offset;
                // initialize first index
                if (inner_first == inner_invalid) inner_first = inner_last;
            }
            add_lines_samples(inner_lines, inner_first, inner_last);
        }
    };

    size_t index_offset = 0;
    sample_polygon(contour, field.inner.contour, index_offset);
    index_offset = contour.size();    
    for (size_t hole_index = 0; hole_index < border.holes.size(); ++hole_index) {
        const Polygon &hole = border.holes[hole_index];
        sample_polygon(hole, field.inner.holes[hole_index], index_offset);
        index_offset += hole.size();
    }
    return result;
}

void SampleIslandUtils::draw(SVG &        svg,
                             const Field &field,
                             bool         draw_border_line_indexes,
                             bool         draw_field_source_indexes)
{
    const char *field_color               = "red";
    const char *border_line_color       = "blue";
    const char *inner_line_color        = "green";
    const char *source_index_text_color   = "blue";
    svg.draw(field.border, field_color);
    Lines border_lines = to_lines(field.border);
    LineUtils::draw(svg, border_lines, border_line_color, 0.,
                    draw_border_line_indexes);
    if (draw_field_source_indexes)
        for (auto &line : border_lines) {
            size_t index = &line - &border_lines.front();
            // start of holes
            if (index >= field.source_indexes.size()) break;
            Point       middle_point = LineUtils::middle(line);
            std::string text = std::to_string(field.source_indexes[index]);
            auto        item = field.field_2_inner.find(index);
            if (item != field.field_2_inner.end()) {
                text += " inner " + std::to_string(item->second);
            }
            svg.draw_text(middle_point, text.c_str(), source_index_text_color);
        }

    // draw inner
    Lines inner_lines = to_lines(field.inner);
    LineUtils::draw(svg, inner_lines, inner_line_color, 0.,
                    draw_border_line_indexes);
    if (draw_field_source_indexes)
        for (auto &line : inner_lines) {
            size_t index = &line - &inner_lines.front();
            Point       middle_point = LineUtils::middle(line);
            std::string text = std::to_string(index);
            svg.draw_text(middle_point, text.c_str(), inner_line_color);
        }

}

void SampleIslandUtils::draw(SVG &                      svg,
                             const SupportIslandPoints &supportIslandPoints,
                             double                     size,
                             const char *               color,
                             bool                       write_type)
{
    for (const auto &p : supportIslandPoints) {
        svg.draw(p->point, color, size);
        if (write_type && p->type != SupportIslandPoint::Type::undefined) {
            auto type_name = SupportIslandPoint::to_string(p->type);
            Point start     = p->point + Point(size, 0.);
            svg.draw_text(start, std::string(type_name).c_str(), color);
        }
    }
}

bool SampleIslandUtils::is_visualization_disabled()
{
#ifndef NDEBUG
    return false;
#endif
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_VORONOI_GRAPH_TO_SVG
    return false;
#endif
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_INITIAL_SAMPLE_POSITION_TO_SVG
    return false;
#endif
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGNE_VD_TO_SVG
    return false;
#endif
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_FIELD_TO_SVG
    return false;
#endif
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGN_ONCE_TO_SVG
    return false;
#endif
#ifdef SLA_SAMPLE_ISLAND_UTILS_STORE_ALIGNED_TO_SVG
    return false;
#endif
    return true;
}
