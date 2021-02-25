#ifndef slic3r_SLA_SuppotstIslands_PostProcessNeighbors_hpp_
#define slic3r_SLA_SuppotstIslands_PostProcessNeighbors_hpp_

#include "IStackFunction.hpp"
#include "VoronoiGraph.hpp"
#include "NodeDataWithResult.hpp"

namespace Slic3r::sla {

/// <summary>
/// call after all neighbors are processed
/// </summary>
class PostProcessNeighbors : public NodeDataWithResult, public IStackFunction
{
public:
    PostProcessNeighbors(VoronoiGraph::ExPath &    result,
                         const VoronoiGraph::Node *node,
                         double                    distance_to_node = 0.,
                         const VoronoiGraph::Path &prev_path =
                             VoronoiGraph::Path({}, 0.) // make copy
                         )
        : NodeDataWithResult(result, node, distance_to_node, prev_path)
    {}

    virtual void process([[maybe_unused]] CallStack &call_stack)
    {
        process();
    }

private:
    void process();
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_PostProcessNeighbors_hpp_
