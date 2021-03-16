#include "LineUtils.hpp"

using namespace Slic3r::sla;

// sort counter clock wise lines
void LineUtils::sort_CCW(Lines &lines, const Point& center)
{
    using namespace Slic3r;
    std::sort(lines.begin(), lines.end(), [&](const Line &l1, const Line &l2) {
        Point p1 = l1.a - center;
        Point p2 = l2.a - center;
        if (p1.y() < 0) {
            if (p2.y() > 0) return false;
            return p1.x() < p2.x();
        } else {
            if (p2.y() < 0) return true;
            return p1.x() > p2.x();
        }
    });
}
