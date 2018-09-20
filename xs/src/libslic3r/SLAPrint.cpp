#include "SLAPrint.hpp"
#include "TriangleMesh.hpp"

namespace Slic3r {

void SLAPrint::process()
{
    using Layer = ExPolygons ;
    using Layers = std::vector<Layer>;

    float ilh = m_config.initial_layer_height.getFloat();
    float lh = m_config.layer_height.getFloat();

    std::vector<float> heights = {ilh, ilh + lh};

    for(ModelObject *o : m_model->objects) {
        if(m_canceled.load()) {
            // clear everything
            return;
        }

        auto&& mesh = o->raw_mesh();
        TriangleMeshSlicer slicer(&mesh);
        Layers layers;


        slicer.slice(heights, &layers, [](){});
    }
}

bool SLAPrint::canceled() const
{
    return m_canceled.load();
}

void SLAPrint::cancel()
{
    m_canceled.store(true);
}

void SLAPrint::restart()
{
    m_canceled.store(false);
}

bool SLAPrint::apply_config(DynamicPrintConfig cfg)
{
    m_proc->stop(); // for now we don't bother
    m_config.apply(cfg);
    return true;
}

void SLAPrint::export_print_data(const std::string &path)
{

}

Polyline SLAPrint::bed_shape() const
{
    return Polyline();
}

}
