#include "VoronoiGraphUtils.hpp"

#include <cmath>
#include <libslic3r/VoronoiOffset.hpp>
#include "IStackFunction.hpp"
#include "EvaluateNeighbor.hpp"
#include "ParabolaUtils.hpp"
#include "LineUtils.hpp"
#include "PointUtils.hpp"

#include <libslic3r/VoronoiVisualUtils.hpp>

using namespace Slic3r::sla;

Slic3r::Line VoronoiGraphUtils::to_line(const VD::edge_type &edge) {
    assert(edge.is_linear());
    assert(edge.is_finite());
    return Line(Point(edge.vertex0()->x(), edge.vertex0()->y()),
                Point(edge.vertex1()->x(), edge.vertex1()->y()));
}

// create line segment between (in the middle) points. With size depend on their distance
Slic3r::Line VoronoiGraphUtils::to_line(Point point1,
                                        Point point2,
                                        double        maximal_distance)
{
    Point middle = (point1 + point2) / 2;
    Point diff = point1 - point2;
    double distance_2 = diff.x() * static_cast<double>(diff.x()) +
                        diff.y() * static_cast<double>(diff.y());
    double half_distance = sqrt(distance_2) / 2.;
    double half_distance_2 = distance_2 / 4;
    double size = sqrt(maximal_distance * maximal_distance - half_distance_2);
    // normalized direction to side multiplied by size/2
    double scale = size / half_distance / 2;
    Point  side_dir(-diff.y() * scale, diff.x() * scale);
    return Line(middle - side_dir, middle + side_dir);
}

bool is_oposit_direction(const Slic3r::Point &p1, const Slic3r::Point &p2) {
    if (abs(p1.x()) > abs(p1.y())) { 
        return (p1.x() > 0) != (p2.x() > 0);
    }
    return (p1.y() > 0) != (p2.y() > 0);
}

Slic3r::Line VoronoiGraphUtils::to_line(const VD::edge_type & edge,
                                        const Points &points,
                                        double maximal_distance)
{
    assert(edge.is_linear());
    assert(edge.is_primary());

    if (edge.is_finite()) return to_line(edge);

    const VD::cell_type &cell1 = *edge.cell();
    const VD::cell_type &cell2 = *edge.twin()->cell();
    assert(cell1.contains_point());
    assert(cell2.contains_point());
    Point p1 = retrieve_point(points, cell1);
    Point p2 = retrieve_point(points, cell2);
    if (edge.vertex0() == nullptr && edge.vertex1() == nullptr)
        return to_line(p1, p2, maximal_distance);

    bool is_v0_null = edge.vertex0() == nullptr;
    if (is_v0_null) std::swap(p1, p2);
    Point direction(p1.y() - p2.y(), p2.x() - p1.x());

    auto get_koef = [&](const Point &v)->double {
        /*/ 
        // faster but less preciss version
        double abs_max_dir = (std::max)(fabs(direction.x()),
                                        fabs(direction.y()));
        return 2 * maximal_distance / abs_max_dir; 
        
        //*/

        // slower but more precisse version
        double dir_size = direction.cast<double>().operatorNorm();
        Point  middle   = (p1 + p2) / 2;
        Point  to_middle = middle - v;
        double to_middle_size = to_middle.cast<double>().operatorNorm();
        double from_middle_size = sqrt(maximal_distance * maximal_distance -
                                  to_middle_size * to_middle_size);
        if (is_oposit_direction(direction, to_middle)) to_middle_size *= -1;
        return (from_middle_size + to_middle_size)/dir_size;
    };

    if (is_v0_null) {
        Point v1(edge.vertex1()->x(), edge.vertex1()->y());
        Point a = v1 + direction * get_koef(v1);
        return Line(a, v1);
    } else {
        Point v0(edge.vertex0()->x(), edge.vertex0()->y());
        Point b = v0 + direction * get_koef(v0);
        return Line(v0, b);
    }
}

Slic3r::Polygon VoronoiGraphUtils::to_polygon(const Lines &lines,
                                              const Point &center,
                                              double       maximal_distance,
                                              double       minimal_distance,
                                              size_t       count_points)
{
    assert(lines.size() >= 1);
    assert(minimal_distance > 0.);
    assert(maximal_distance > minimal_distance);
    assert(count_points >= 3);
    Points points;
    points.reserve(lines.size());
    const Line *prev_line = &lines.back();
    double max_angle = 2 * M_PI / count_points;
    for (const Line &line : lines) {
        const Point &p1   = prev_line->b;
        const Point &p2   = line.a;
        prev_line         = &line;
        Point diff = p1-p2;
        if (abs(diff.x()) < minimal_distance &&
            abs(diff.y()) < minimal_distance) {
            Point avg   = (p1 + p2) / 2;
            points.push_back(avg);
            continue;
        } 
        Point  v1 = p1 - center;
        Point  v2 = p2 - center;
        double a1 = std::atan2(v1.y(), v1.x());
        double a2 = std::atan2(v2.y(), v2.x());

        double diff_angle = a2 - a1;
        if(diff_angle < 0.) diff_angle += 2 * M_PI;
        if(diff_angle > 2 * M_PI) diff_angle -= 2 * M_PI;

        size_t count_segment = std::floor(fabs(diff_angle) / max_angle) + 1;
        double increase_angle = diff_angle / count_segment;
        points.push_back(p1);
        for (size_t i = 1; i < count_segment; i++) {
            double angle = a1 + i*increase_angle;
            Point  direction(
                static_cast<coord_t>(cos(angle) * maximal_distance),
                static_cast<coord_t>(sin(angle) * maximal_distance));
            points.push_back(center + direction);
        }
        points.push_back(p2);
        
    }
    return Polygon(points);
}

Slic3r::Polygon VoronoiGraphUtils::to_polygon(const VD::cell_type & cell,
                                              const Slic3r::Points &points,
                                              double maximal_distance)
{
    const VD::edge_type *edge = cell.incident_edge();

    Lines lines;
    Point center = points[cell.source_index()];
    // Convenient way to iterate edges around Voronoi cell.
    do {
        assert(edge->is_linear());
        if (edge->is_primary()) {
            Line line = to_line(*edge, points, maximal_distance);
            if (!PointUtils::is_ccw(line.a, line.b, center)) std::swap(line.a, line.b);
            lines.push_back(line);
        }
        edge = edge->next();
    } while (edge != cell.incident_edge());
    LineUtils::sort_CCW(lines, center);
    // preccission to decide when not connect neighbor points
    double          min_distance = maximal_distance / 1000.; 
    size_t          count_point  = 6; // count added points
    Slic3r::Polygon polygon = to_polygon(lines, center, maximal_distance, min_distance, count_point);
#ifdef SLA_SUPPORTPOINTGEN_DEBUG
    {
        std::cout << "cell " << cell.source_index() << " has " << lines.size()  << "edges" << std::endl;
        BoundingBox bbox(center - Point(maximal_distance, maximal_distance),
                         center + Point(maximal_distance, maximal_distance));
        static int  counter  = 0;
        std::string filename = "polygon" + std::to_string(counter++) + ".svg";
        SVG svg(filename.c_str(), bbox);
        svg.draw(center, "lightgreen", maximal_distance);
        svg.draw(polygon, "lightblue");
        int index = 0;
        for (auto &line : lines) {
            svg.draw(line);
            svg.draw_text(line.a, ("A"+std::to_string(++index)).c_str(), "green");
            svg.draw_text(line.b, ("B" + std::to_string(index)).c_str(), "blue");
        }
        svg.draw(center, "red", maximal_distance / 100);
    }
#endif /* SLA_SUPPORTPOINTGEN_DEBUG */
    return polygon;
}

VoronoiGraph::Node *VoronoiGraphUtils::getNode(VoronoiGraph &         graph,
                                               const VD::vertex_type *vertex,
                                               const VD::edge_type *  edge,
                                               const Lines &          lines)
{
    std::map<const VD::vertex_type *, VoronoiGraph::Node> &data = graph.data;
    auto &mapItem = data.find(vertex);
    // return when exists
    if (mapItem != data.end()) return &mapItem->second;

    // is new vertex (first edge to this vertex)
    // calculate distance to islad border + fill item0
    const VD::cell_type *cell = edge->cell();
    // const VD::cell_type *  cell2     = edge.twin()->cell();
    const Line &line = lines[cell->source_index()];
    // const Line &           line1     = lines[cell2->source_index()];
    Point  point(vertex->x(), vertex->y());
    double distance = line.distance_to(point);

    auto &[iterator,
           success] = data.emplace(vertex,
                                   VoronoiGraph::Node(vertex, distance));
    assert(success);
    return &iterator->second;
}

Slic3r::Point VoronoiGraphUtils::retrieve_point(const Lines &        lines,
                                                const VD::cell_type &cell)
{
    using namespace boost::polygon;
    assert(cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT ||
           cell.source_category() == SOURCE_CATEGORY_SEGMENT_END_POINT);
    return (cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT) ?
               lines[cell.source_index()].a :
               lines[cell.source_index()].b;
}

Slic3r::Point VoronoiGraphUtils::retrieve_point(const Points &       points,
                                                const VD::cell_type &cell)
{
    assert(cell.source_category() == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT);
    return points[cell.source_index()];
}

Slic3r::Point VoronoiGraphUtils::get_parabola_point(
    const VD::edge_type &parabola, const Slic3r::Lines &lines)
{
    using namespace boost::polygon;
    assert(parabola.is_curved());
    const VD::cell_type& cell = (parabola.cell()->contains_point())?
                                *parabola.cell() : *parabola.twin()->cell();
    assert(cell.contains_point());
    assert(cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT ||
           cell.source_category() == SOURCE_CATEGORY_SEGMENT_END_POINT);
    return (cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT) ?
           lines[cell.source_index()].a :
           lines[cell.source_index()].b;
}

Slic3r::Line VoronoiGraphUtils::get_parabola_line(
    const VD::edge_type &parabola, const Slic3r::Lines &lines)
{
    assert(parabola.is_curved());
    const VD::cell_type& cell = (parabola.cell()->contains_segment())?
                                *parabola.cell() : *parabola.twin()->cell();
    assert(cell.contains_segment());
    return lines[cell.source_index()];
}

Parabola VoronoiGraphUtils::get_parabola(
    const VD::edge_type &edge, const Lines &lines)
{
    Point    point = get_parabola_point(edge, lines);
    Line     line  = get_parabola_line(edge, lines);
    return Parabola(line, point);
}

double VoronoiGraphUtils::calculate_length_of_parabola(
    const VD::edge_type &                               edge,
    const Lines &                                       lines)
{
    Point v0{edge.vertex0()->x(), edge.vertex0()->y()};
    Point v1{edge.vertex1()->x(), edge.vertex1()->y()};
    ParabolaSegment parabola(get_parabola(edge, lines), v0, v1);
    return ParabolaUtils::length(parabola);
}

double VoronoiGraphUtils::calculate_length(
    const VD::edge_type &edge, const Lines &lines)
{
    if (edge.is_linear()) {
        const VD::vertex_type* v0 = edge.vertex0();
        const VD::vertex_type* v1 = edge.vertex1();
        double diffX = v0->x() - v1->x();
        double diffY = v0->y() - v1->y();
        return sqrt(diffX * diffX + diffY * diffY);
    }    
    assert(edge.is_curved());
    return calculate_length_of_parabola(edge, lines);
}

double VoronoiGraphUtils::calculate_max_width(
    const VD::edge_type &edge, const Lines &lines)
{
    Point v0{edge.vertex0()->x(), edge.vertex0()->y()};
    Point v1{edge.vertex1()->x(), edge.vertex1()->y()};

    if (edge.is_linear()) {
        // edge line could be initialized by 2 points
        if (edge.cell()->contains_point()) {
            const Line &source_line = lines[edge.cell()->source_index()];
            Point source_point;
            if (edge.cell()->source_category() ==
                boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT)
                source_point = source_line.a;
            else {
                assert(edge.cell()->source_category() ==
                    boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT);
                source_point = source_line.b;
            }
            Point vec0 = source_point - v0;
            Point vec1 = source_point - v1;
            double distance0 = sqrt(vec0.x() * vec0.x() + vec0.y() * vec0.y());
            double distance1 = sqrt(vec0.x() * vec0.x() + vec0.y() * vec0.y());
            return 2 * std::max(distance0, distance1);
        }
        assert(edge.cell()->contains_segment());
        assert(!edge.twin()->cell()->contains_point());
        assert(edge.twin()->cell()->contains_segment());

        const Line &line = lines[edge.cell()->source_index()];

        double distance0 = line.perp_distance_to(v0);
        double distance1 = line.perp_distance_to(v1);
        return 2 * std::max(distance0, distance1);
    }
    assert(edge.is_curved());
    Parabola parabola = get_parabola(edge, lines);
    // distance to point and line is same
    // vector from edge vertex to parabola focus point
    Point  vec0 = parabola.focus - v0;
    Point  vec1 = parabola.focus - v1;
    double distance0 = sqrt(vec0.x() * vec0.x() + vec0.y() * vec0.y());
    double distance1 = sqrt(vec0.x() * vec0.x() + vec0.y() * vec0.y());
    return 2 * std::max(distance0, distance1);
}

VoronoiGraph VoronoiGraphUtils::getSkeleton(const VD &vd, const Lines &lines)
{
    // vd should be annotated.
    // assert(Voronoi::debug::verify_inside_outside_annotations(vd));

    VoronoiGraph         skeleton;
    const VD::edge_type *first_edge = &vd.edges().front();
    for (const VD::edge_type &edge : vd.edges()) {
        size_t edge_idx = &edge - first_edge;
        if (
            // Ignore secondary and unbounded edges, they shall never be part
            // of the skeleton.
            edge.is_secondary() || edge.is_infinite() ||
            // Skip the twin edge of an edge, that has already been processed.
            &edge > edge.twin() ||
            // Ignore outer edges.
            (Voronoi::edge_category(edge) !=
                 Voronoi::EdgeCategory::PointsInside &&
             Voronoi::edge_category(edge.twin()) !=
                 Voronoi::EdgeCategory::PointsInside))
            continue;

        const VD::vertex_type * v0        = edge.vertex0();
        const VD::vertex_type * v1        = edge.vertex1();
        Voronoi::VertexCategory category0 = Voronoi::vertex_category(*v0);
        Voronoi::VertexCategory category1 = Voronoi::vertex_category(*v1);
        if (category0 == Voronoi::VertexCategory::Outside ||
            category1 == Voronoi::VertexCategory::Outside)
            continue;
        // only debug check annotation
        if (category0 == Voronoi::VertexCategory::Unknown ||
            category1 == Voronoi::VertexCategory::Unknown)
            return {}; // vd must be annotated

        double length = calculate_length(edge, lines);
        double max_width = calculate_max_width(edge, lines);
        VoronoiGraph::Node *node0 = getNode(skeleton, v0, &edge, lines);
        VoronoiGraph::Node *node1 = getNode(skeleton, v1, &edge, lines);

        // TODO: Do not store twice length and max_width.
        // add extended Edge to graph, both side
        VoronoiGraph::Node::Neighbor neighbor0(&edge, node1, length, max_width);
        node0->neighbors.push_back(neighbor0);
        VoronoiGraph::Node::Neighbor neighbor1(edge.twin(), node0, length, max_width);
        node1->neighbors.push_back(neighbor1);
    }
    return skeleton;
}

const VoronoiGraph::Node::Neighbor *VoronoiGraphUtils::get_neighbor(
    const VoronoiGraph::Node *from, const VoronoiGraph::Node *to)
{
    for (const VoronoiGraph::Node::Neighbor &neighbor : from->neighbors)
        if (neighbor.node == to) return &neighbor;
    return nullptr;
}

double VoronoiGraphUtils::get_neighbor_distance(const VoronoiGraph::Node *from,
                                                const VoronoiGraph::Node *to)
{
    const VoronoiGraph::Node::Neighbor *neighbor = get_neighbor(from, to);
    assert(neighbor != nullptr);
    return neighbor->edge_length;
}

VoronoiGraph::Path VoronoiGraphUtils::find_longest_path_on_circle(
    const VoronoiGraph::Circle &                 circle,
    const VoronoiGraph::ExPath::SideBranchesMap &side_branches)
{
    double half_circle_length = circle.length / 2.;
    double distance_on_circle = 0;

    bool                      is_longest_revers_direction = false;
    const VoronoiGraph::Node *longest_circle_node         = nullptr;
    const VoronoiGraph::Path *longest_circle_branch       = nullptr;
    double                    longest_branch_length       = 0;

    bool is_short_revers_direction = false;
    // find longest side branch
    const VoronoiGraph::Node *prev_circle_node = nullptr;
    for (const VoronoiGraph::Node *circle_node : circle.nodes) {
        if (prev_circle_node != nullptr)
            distance_on_circle += get_neighbor_distance(circle_node,
                                                        prev_circle_node);
        prev_circle_node = circle_node;

        auto side_branches_item = side_branches.find(circle_node);
        if (side_branches_item != side_branches.end()) {
            // side_branches should be sorted by length
            if (distance_on_circle > half_circle_length)
                is_short_revers_direction = true;
            const auto &longest_node_branch = side_branches_item->second.top();
            double      circle_branch_length = longest_node_branch.length +
                                          ((is_short_revers_direction) ?
                                               (circle.length -
                                                distance_on_circle) :
                                               distance_on_circle);
            if (longest_branch_length < circle_branch_length) {
                longest_branch_length       = circle_branch_length;
                is_longest_revers_direction = is_short_revers_direction;
                longest_circle_node         = circle_node;
                longest_circle_branch       = &longest_node_branch;
            }
        }
    }
    assert(longest_circle_node !=
           nullptr); // only circle with no side branches
    assert(longest_circle_branch != nullptr);
    // almost same - double preccission
    // distance_on_circle += get_neighbor_distance(circle.path.back(),
    // circle.path.front()); assert(distance_on_circle == circle.length);

    // circlePath
    auto circle_iterator = std::find(circle.nodes.begin(), circle.nodes.end(),
                                     longest_circle_node);
    VoronoiGraph::Nodes circle_path;
    if (is_longest_revers_direction) {
        circle_path = VoronoiGraph::Nodes(circle_iterator, circle.nodes.end());
        std::reverse(circle_path.begin(), circle_path.end());
    } else {
        if (longest_circle_node != circle.nodes.front())
            circle_path = VoronoiGraph::Nodes(circle.nodes.begin() + 1,
                                              circle_iterator + 1);
    }
    // append longest side branch
    circle_path.insert(circle_path.end(),
                       longest_circle_branch->nodes.begin(),
                       longest_circle_branch->nodes.end());
    return {circle_path, longest_branch_length};
}

VoronoiGraph::Path VoronoiGraphUtils::find_longest_path_on_circles(
    const VoronoiGraph::Node &  input_node,
    size_t                      finished_circle_index,
    const VoronoiGraph::ExPath &ex_path)
{
    const std::vector<VoronoiGraph::Circle> &circles = ex_path.circles;
    const auto &circle                = circles[finished_circle_index];
    auto        connected_circle_item = ex_path.connected_circle.find(
        finished_circle_index);
    // is only one circle
    if (connected_circle_item == ex_path.connected_circle.end()) {
        // find longest path over circle and store it into next_path
        return find_longest_path_on_circle(circle, ex_path.side_branches);
    }

    // multi circle
    // find longest path over circles
    const std::set<size_t> &connected_circles = connected_circle_item->second;

    // collect all circle ndoes
    std::set<const VoronoiGraph::Node *> nodes;
    nodes.insert(circle.nodes.begin(), circle.nodes.end());
    for (size_t circle_index : connected_circles) {
        const auto &circle = circles[circle_index];
        nodes.insert(circle.nodes.begin(), circle.nodes.end());
    }

    // nodes are path throw circles
    // length is sum path throw circles PLUS length of longest side_branch
    VoronoiGraph::Path longest_path;

    // wide search by shortest distance for path over circle's node
    // !! Do NOT use recursion, may cause stack overflow
    std::set<const VoronoiGraph::Node *> done; // all ready checked
    // on top is shortest path
    std::priority_queue<VoronoiGraph::Path, std::vector<VoronoiGraph::Path>,
                        VoronoiGraph::Path::OrderLengthFromShortest>
                       search_queue;
    VoronoiGraph::Path start_path({&input_node}, 0.);
    search_queue.emplace(start_path);
    while (!search_queue.empty()) {
        // shortest path from input_node
        VoronoiGraph::Path path(std::move(search_queue.top()));
        search_queue.pop();
        const VoronoiGraph::Node &node = *path.nodes.back();
        if (done.find(&node) != done.end()) { // already checked
            continue;
        }
        done.insert(&node);
        for (const VoronoiGraph::Node::Neighbor &neighbor : node.neighbors) {
            if (nodes.find(neighbor.node) == nodes.end())
                continue; // out of circles
            if (done.find(neighbor.node) != done.end()) continue;
            VoronoiGraph::Path neighbor_path = path; // make copy
            neighbor_path.append(neighbor.node, neighbor.edge_length);
            search_queue.push(neighbor_path);

            auto branches_item = ex_path.side_branches.find(neighbor.node);
            // exist side from this neighbor node ?
            if (branches_item == ex_path.side_branches.end()) continue;
            const VoronoiGraph::Path &longest_branch = branches_item->second
                                                           .top();
            double length = longest_branch.length + neighbor_path.length;
            if (longest_path.length < length) {
                longest_path.length = length;
                longest_path.nodes  = neighbor_path.nodes; // copy path
            }
        }
    }

    // create result path
    assert(!longest_path.nodes.empty());
    longest_path.nodes.erase(longest_path.nodes.begin()); // remove input_node
    assert(!longest_path.nodes.empty());
    auto branches_item = ex_path.side_branches.find(longest_path.nodes.back());
    if (branches_item == ex_path.side_branches.end()) {
        // longest path ends on circle
        return longest_path;
    }
    const VoronoiGraph::Path &longest_branch = branches_item->second.top();
    longest_path.nodes.insert(longest_path.nodes.end(),
                              longest_branch.nodes.begin(),
                              longest_branch.nodes.end());
    return longest_path;
}

std::optional<VoronoiGraph::Circle> VoronoiGraphUtils::create_circle(
    const VoronoiGraph::Path &          path,
    const VoronoiGraph::Node::Neighbor &neighbor)
{
    VoronoiGraph::Nodes passed_nodes = path.nodes;
    // detection of circle
    // not neccesary to check last one in path
    auto        end_find  = passed_nodes.end() - 1;
    const auto &path_item = std::find(passed_nodes.begin(), end_find,
                                      neighbor.node);
    if (path_item == end_find) return {}; // circle not detected
    // separate Circle:
    VoronoiGraph::Nodes circle_path(path_item, passed_nodes.end());
    // !!! Real circle lenght is calculated on detection of end circle
    // now circle_length contain also lenght of path before circle
    double circle_length = path.length + neighbor.edge_length;
    // solve of branch length will be at begin of cirlce
    return VoronoiGraph::Circle(std::move(circle_path), circle_length);
};

void VoronoiGraphUtils::merge_connected_circle(
    VoronoiGraph::ExPath::ConnectedCircles &dst,
    VoronoiGraph::ExPath::ConnectedCircles &src,
    size_t                                  dst_circle_count)
{
    std::set<size_t> done;
    for (const auto &item : src) {
        size_t dst_index = dst_circle_count + item.first;
        if (done.find(dst_index) != done.end()) continue;
        done.insert(dst_index);

        std::set<size_t> connected_circle;
        for (const size_t &src_index : item.second)
            connected_circle.insert(dst_circle_count + src_index);

        auto &dst_set = dst[dst_index];
        dst_set.merge(connected_circle);

        // write same information into connected circles
        connected_circle = dst_set; // copy
        connected_circle.insert(dst_index);
        for (size_t prev_connection_idx : dst_set) {
            done.insert(prev_connection_idx);
            for (size_t connected_circle_idx : connected_circle) {
                if (connected_circle_idx == prev_connection_idx) continue;
                dst[prev_connection_idx].insert(connected_circle_idx);
            }
        }
    }
}

void VoronoiGraphUtils::append_neighbor_branch(VoronoiGraph::ExPath &dst,
                                               VoronoiGraph::ExPath &src)
{
    // move side branches
    if (!src.side_branches.empty())
        dst.side_branches
            .insert(std::make_move_iterator(src.side_branches.begin()),
                    std::make_move_iterator(src.side_branches.end()));

    // move circles
    if (!src.circles.empty()) {
        // copy connected circles indexes
        if (!src.connected_circle.empty()) {
            merge_connected_circle(dst.connected_circle, src.connected_circle,
                                   dst.circles.size());
        }
        dst.circles.insert(dst.circles.end(),
                           std::make_move_iterator(src.circles.begin()),
                           std::make_move_iterator(src.circles.end()));
    }
}

void VoronoiGraphUtils::reshape_longest_path(VoronoiGraph::ExPath &path)
{
    assert(path.nodes.size() >= 1);

    double                    actual_length = 0.;
    const VoronoiGraph::Node *prev_node     = nullptr;
    VoronoiGraph::Nodes       origin_path   = path.nodes; // make copy
    // index to path
    size_t path_index = 0;
    for (const VoronoiGraph::Node *node : origin_path) {
        if (prev_node != nullptr) {
            ++path_index;
            actual_length += get_neighbor_distance(prev_node, node);
        }
        prev_node = node;
        // increase actual length

        auto side_branches_item = path.side_branches.find(node);
        if (side_branches_item == path.side_branches.end())
            continue; // no side branches
        VoronoiGraph::ExPath::SideBranches &branches = side_branches_item
                                                           ->second;
        if (actual_length >= branches.top().length)
            continue; // no longer branch

        auto               end_path = path.nodes.begin() + path_index;
        VoronoiGraph::Path side_branch({path.nodes.begin(), end_path},
                                       actual_length);
        std::reverse(side_branch.nodes.begin(), side_branch.nodes.end());
        VoronoiGraph::Path new_main_branch(std::move(branches.top()));
        branches.pop();
        std::reverse(new_main_branch.nodes.begin(),
                     new_main_branch.nodes.end());
        // add old main path store into side branches - may be it is not neccessary
        branches.push(std::move(side_branch));

        // swap side branch with main branch
        path.nodes.erase(path.nodes.begin(), end_path);
        path.nodes.insert(path.nodes.begin(), new_main_branch.nodes.begin(),
                          new_main_branch.nodes.end());

        path.length += new_main_branch.length;
        path.length -= actual_length;
        path_index    = new_main_branch.nodes.size();
        actual_length = new_main_branch.length;
    }
}

VoronoiGraph::ExPath VoronoiGraphUtils::create_longest_path(
    const VoronoiGraph::Node *start_node)
{
    VoronoiGraph::ExPath longest_path;
    CallStack            call_stack;
    call_stack.emplace(
        std::make_unique<EvaluateNeighbor>(longest_path, start_node));

    // depth search for longest path in graph
    while (!call_stack.empty()) {
        std::unique_ptr<IStackFunction> stack_function = std::move(
            call_stack.top());
        call_stack.pop();
        stack_function->process(call_stack);
        // stack function deleted
    }

    reshape_longest_path(longest_path);
    // after reshape it shoud be longest path for whole Voronoi Graph
    return longest_path;
}

const VoronoiGraph::Node *VoronoiGraphUtils::get_twin_node(const VoronoiGraph::Node::Neighbor *neighbor)
{
    auto twin_edge = neighbor->edge->twin();
    for (const VoronoiGraph::Node::Neighbor n : neighbor->node->neighbors) {
        if (n.edge == twin_edge) return n.node;
    }
    assert(false);
    return nullptr;
}

Slic3r::Point VoronoiGraphUtils::get_edge_point(const VD::edge_type *edge,
                                                double               ratio)
{
    const VD::vertex_type *v0 = edge->vertex0();
    const VD::vertex_type *v1 = edge->vertex1();
    if (ratio <= std::numeric_limits<double>::epsilon())
        return Point(v0->x(), v0->y());
    if (ratio >= 1. - std::numeric_limits<double>::epsilon())
        return Point(v1->x(), v1->y());

    if (edge->is_linear()) {
        Point dir(v1->x() - v0->x(), v1->y() - v0->y());
        // normalize
        dir *= ratio;
        return Point(v0->x() + dir.x(), v0->y() + dir.y());
    }

    assert(edge->is_curved());
    // TODO: distance on curve

    // approx by line
    Point dir(v1->x() - v0->x(), v1->y() - v0->y());
    dir *= ratio;
    return Point(v0->x() + dir.x(), v0->y() + dir.y());
}

const VoronoiGraph::Node *VoronoiGraphUtils::getFirstContourNode(
    const VoronoiGraph &graph)
{
    for (const auto &[key, value] : graph.data) {
        const VD::vertex_type & vertex   = *key;
        Voronoi::VertexCategory category = Voronoi::vertex_category(vertex);
        if (category == Voronoi::VertexCategory::OnContour) {
            return &value;
        }
    }
    return nullptr;
}

double VoronoiGraphUtils::get_max_width(const VoronoiGraph::Nodes &path)
{
    double max = 0.;
    const VoronoiGraph::Node *prev_node = nullptr;
    for (const VoronoiGraph::Node *node : path) { 
        if (prev_node == nullptr) {
            prev_node = node;
            continue;
        }
        const VoronoiGraph::Node::Neighbor *neighbor = get_neighbor(prev_node, node);
        if (max < neighbor->max_width)  max = neighbor->max_width;        
        prev_node = node;
    }
    return max;
}

double VoronoiGraphUtils::get_max_width(
    const VoronoiGraph::ExPath &longest_path)
{
    double max = get_max_width(longest_path.nodes);
    for (const auto &side_branches_item : longest_path.side_branches) {
        const VoronoiGraph::Node *prev_node = side_branches_item.first;
        VoronoiGraph::ExPath::SideBranches side_branches = side_branches_item.second; // !!! copy
        while (!side_branches.empty()) {
            const VoronoiGraph::Path &side_path = side_branches.top();
            const VoronoiGraph::Node::Neighbor *first_neighbor =
                get_neighbor(prev_node, side_path.nodes.front());
            double max_side_branch = std::max(
                get_max_width(side_path.nodes), first_neighbor->max_width);
            if (max < max_side_branch) max = max_side_branch;
            side_branches.pop();
        }
    }

    for (const VoronoiGraph::Circle &circle : longest_path.circles) {
        const VoronoiGraph::Node::Neighbor *first_neighbor =
            get_neighbor(circle.nodes.front(), circle.nodes.back());
        double max_circle = std::max(
            first_neighbor->max_width, get_max_width(circle.nodes));
        if (max < max_circle) max = max_circle;
    }

    return max;
}

// !!! is slower than go along path
double VoronoiGraphUtils::get_max_width(const VoronoiGraph::Node *node)
{
    double max = 0.;
    std::set<const VoronoiGraph::Node *> done;
    std::queue<const VoronoiGraph::Node *> process;
    process.push(node);
    while (!process.empty()) {
        const VoronoiGraph::Node *actual_node = process.front();
        process.pop();
        if (done.find(actual_node) != done.end()) continue;
        for (const VoronoiGraph::Node::Neighbor& neighbor: actual_node->neighbors) {
            if (done.find(neighbor.node) != done.end()) continue;
            process.push(neighbor.node);
            if (max < neighbor.max_width) max = neighbor.max_width;
        }
        done.insert(actual_node);
    }
    return max;
}

void VoronoiGraphUtils::draw(SVG &               svg,
                             const VoronoiGraph &graph,
                             coord_t             width)
{
    for (const auto &[key, value] : graph.data) {
        svg.draw(Point(key->x(), key->y()), "lightgray", width);
        for (const auto &n : value.neighbors) {
            if (n.edge->vertex0() > n.edge->vertex1()) continue;
            auto  v0 = *n.edge->vertex0();
            Point from(v0.x(), v0.y());
            auto  v1 = *n.edge->vertex1();
            Point to(v1.x(), v1.y());
            svg.draw(Line(from, to), "gray", width);

            Point center = from + to;
            center *= .5;
            // svg.draw_text(center,
            // (std::to_string(std::round(n.edge_length/3e5)/100.)).c_str(), "gray");
        }
    }
}

void VoronoiGraphUtils::draw(SVG &                      svg,
                             const VoronoiGraph::Nodes &path,
                             coord_t                    width,
                             const char *               color,
                             bool                       finish)
{
    const VoronoiGraph::Node *prev_node = (finish) ? path.back() : nullptr;
    int                       index     = 0;
    for (auto &node : path) {
        ++index;
        if (prev_node == nullptr) {
            prev_node = node;
            continue;
        }
        Point from(prev_node->vertex->x(), prev_node->vertex->y());
        Point to(node->vertex->x(), node->vertex->y());
        svg.draw(Line(from, to), color, width);

        svg.draw_text(from, std::to_string(index - 1).c_str(), color);
        svg.draw_text(to, std::to_string(index).c_str(), color);
        prev_node = node;
    }
}

void VoronoiGraphUtils::draw(SVG &                       svg,
                             const VoronoiGraph::ExPath &path,
                             coord_t                     width)
{
    const char *circlePathColor   = "green";
    const char *sideBranchesColor = "blue";
    const char *mainPathColor     = "red";

    for (auto &circle : path.circles) {
        draw(svg, circle.nodes, width, circlePathColor, true);
        Point center(0, 0);
        for (auto p : circle.nodes) {
            center.x() += p->vertex->x();
            center.y() += p->vertex->y();
        }
        center.x() /= circle.nodes.size();
        center.y() /= circle.nodes.size();

        svg.draw_text(center,
                      ("C" + std::to_string(&circle - &path.circles.front()))
                          .c_str(),
                      circlePathColor);
    }

    for (const auto &branches : path.side_branches) {
        auto tmp = branches.second; // copy
        while (!tmp.empty()) {
            const auto &branch = tmp.top();
            auto        path   = branch.nodes;
            path.insert(path.begin(), branches.first);
            draw(svg, path, width, sideBranchesColor);
            tmp.pop();
        }
    }

    draw(svg, path.nodes, width, mainPathColor);
}

