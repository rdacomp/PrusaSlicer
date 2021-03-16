#ifndef slic3r_SLA_SuppotstIslands_LineUtils_hpp_
#define slic3r_SLA_SuppotstIslands_LineUtils_hpp_

#include <libslic3r/Line.hpp>

namespace Slic3r::sla {

/// <summary>
/// Class which contain collection of static function
/// for work with Line and Lines.
/// QUESTION: Is it only for SLA?
/// </summary>
class LineUtils
{
public:
    LineUtils() = delete;

    /// <summary>
    /// Sort lines to be in counter clock wise order
    /// </summary>
    /// <param name="lines">Lines to sort</param>
    /// <param name="center">Center for order</param>
    static void sort_CCW(Lines &lines, const Point &center);
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_LineUtils_hpp_
