#ifndef slic3r_SLA_SuppotstIslands_ParabolaUtils_hpp_
#define slic3r_SLA_SuppotstIslands_ParabolaUtils_hpp_

#include "Parabola.hpp"

namespace Slic3r::sla {

/// <summary>
/// Class which contain collection of static function
/// for work with Parabola.
/// </summary>
class ParabolaUtils
{
public:
    ParabolaUtils() = delete;

    /// <summary>
    /// Integrate length over interval defined by points from and to
    /// </summary>
    /// <param name="parabola">Input parabola</param>
    /// <param name="from">Input point lay on parabola</param>
    /// <param name="to">Input point lay on parabola</param>
    /// <returns>Length of parabola arc</returns>
    static double calculate_length_of_parabola(const Parabola &parabola,
                                               const Point &   from,
                                               const Point &   to);

    /// <summary>
    /// Sample parabola between points from and to by step.
    /// </summary>
    /// <param name="parabola">Input parabola</param>
    /// <param name="from">Input point lay on parabola</param>
    /// <param name="to">Input point lay on parabola</param>
    /// <param name="discretization_step">Define sampling</param>
    /// <returns>Length of parabola arc</returns>
    static double calculate_length_of_parabola_by_sampling(
        const Parabola &parabola,
        const Point &   from,
        const Point &   to,
        double          discretization_step = 0.0002 * 1e6);

    /// <summary>
    /// calculate focal length of parabola
    /// </summary>
    /// <param name="parabola">input parabola</param>
    /// <returns>Focal length</returns>
    static double focal_length(const Parabola &parabola);

    /// <summary>
    /// Check if parabola interval (from, to) contains top of parabola
    /// </summary>
    /// <param name="parabola">input parabola</param>
    /// <param name="from">Start of interval, point lay on parabola</param>
    /// <param name="to">End of interval, point lay on parabola</param>
    /// <returns>True when interval contain top of parabola otherwise False</returns>
    static bool is_over_zero(const Parabola &parabola,
                             const Point &   from,
                             const Point &   to);

private:
    /// <summary>
    /// Integral of parabola: y = x^2 from zero to point x
    /// https://ocw.mit.edu/courses/mathematics/18-01sc-single-variable-calculus-fall-2010/unit-4-techniques-of-integration/part-b-partial-fractions-integration-by-parts-arc-length-and-surface-area/session-78-computing-the-length-of-a-curve/MIT18_01SCF10_Ses78d.pdf
    /// </summary>
    /// <param name="x">x coordinate of parabola, Positive number</param>
    /// <returns>Length of parabola from zero to x</returns> 
    static double parabola_arc_length(double x) {
        double sqrtRes = sqrt(1 + 4 * x * x);
        return 1 / 4. * log(2 * x + sqrtRes) + 1 / 2. * x * sqrtRes;
    };
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_ParabolaUtils_hpp_
