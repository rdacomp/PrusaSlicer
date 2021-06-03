#ifndef MESHSIMPLIFY_HPP
#define MESHSIMPLIFY_HPP

#include <vector>

#include <libslic3r/TriangleMesh.hpp>

namespace Slic3r {

void simplify_mesh(indexed_triangle_set &);

/// <summary>
/// Main simplification function - original functionality
/// </summary>
/// <param name="its">Input/Output vertices and faces</param>
/// <param name="target_count">target nr. of triangles</param>
/// <param name="agressiveness">sharpness to increase the threshold.
/// 5..8 are good numbers,
/// more iterations yield higher quality</param>
void simplify_mesh(indexed_triangle_set &its,
                   int                   target_count,
                   double                agressiveness);

/// <summary>
/// Remove edges smaller than edge_length && error smaller than max_trh
/// </summary>
/// <param name="its">Input/Output vertices and faces</param>
/// <param name="edge_length">Maximal edge length to be reduced</param>
/// <param name="max_thr">Maximal error of edge to be discarded</param>
void remove_small_edges(indexed_triangle_set &its,
                        double                edge_length,
                        double                max_thr);

// TODO: (but this can be done with IGL as well)
// void simplify_mesh(indexed_triangle_set &, int face_count, float agressiveness = 0.5f);

template<class...Args> void simplify_mesh(TriangleMesh &m, Args &&...a)
{
    m.require_shared_vertices();
    simplify_mesh(m.its, std::forward<Args>(a)...);
    m = TriangleMesh{m.its};
    m.require_shared_vertices();
}

} // namespace Slic3r

#endif // MESHSIMPLIFY_H
