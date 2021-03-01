#include "ParabolaUtils.hpp"

using namespace Slic3r::sla;

double ParabolaUtils::calculate_length_of_parabola(
    const Parabola &parabola, const Point &from, const Point &to)
{
    const Point &point = parabola.focus;
    const Line & line  = parabola.directrix;
    Line norm_line(point, point + line.normal());

    // sign of distance is resolved by dot product in function is_over_zero()
    double scaled_x1 = norm_line.perp_distance_to(from);
    double scaled_x2 = norm_line.perp_distance_to(to);

    double parabola_scale = 1. / (4. * focal_length(parabola));

    double x1 = scaled_x1 * parabola_scale;
    double x2 = scaled_x2 * parabola_scale;

    double length_x1 = parabola_arc_length(x1) / parabola_scale;
    double length_x2 = parabola_arc_length(x2) / parabola_scale;

    return (is_over_zero(parabola, from, to)) ?
               (length_x1 + length_x2) :    // interval is over zero
               fabs(length_x1 - length_x2); // interval is on same side of parabola
}


#include <Libslic3r/Geometry.hpp>
#include <Libslic3r/VoronoiOffset.hpp>
#include <Libslic3r/VoronoiVisualUtils.hpp>
double ParabolaUtils::calculate_length_of_parabola_by_sampling(
    const Parabola &parabola,
    const Point &   from,
    const Point &   to,
    double          discretization_step)
{
    using VD = Slic3r::Geometry::VoronoiDiagram;
    std::vector<Voronoi::Internal::point_type> parabola_samples({from, to});

    VD::point_type   source_point = parabola.focus;
    VD::segment_type source_segment(parabola.directrix.a, parabola.directrix.b);
    ::boost::polygon::voronoi_visual_utils<double>::discretize(
        source_point, source_segment, discretization_step, &parabola_samples);

    double sumLength = 0;
    for (size_t index = 1; index < parabola_samples.size(); ++index) {
        double diffX = parabola_samples[index - 1].x() -
                       parabola_samples[index].x();
        double diffY = parabola_samples[index - 1].y() -
                       parabola_samples[index].y();
        double length = sqrt(diffX * diffX + diffY * diffY);
        sumLength += length;
    }
    return sumLength;
}

double ParabolaUtils::focal_length(const Parabola &parabola)
{
    // https://en.wikipedia.org/wiki/Parabola
    // p = 2f; y = 1/(4f) * x^2; y = 1/(2p) * x^2
    double p = parabola.directrix.perp_distance_to(parabola.focus);
    double f = p / 2.;
    return f;
}

bool ParabolaUtils::is_over_zero(const Parabola &parabola,
                                 const Point &   from,
                                 const Point &   to)
{
    Point line_direction = parabola.directrix.b - parabola.directrix.a;
    bool  is_positive_x1 = line_direction.dot(parabola.focus - from) > 0.;
    bool  is_positive_x2 = line_direction.dot(parabola.focus - to) > 0.;
    return is_positive_x1 != is_positive_x2;
}
