#include "SLAPrint.hpp"
#include "TriangleMesh.hpp"

#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>//#include "tbb/mutex.h"

namespace Slic3r {

void SLAPrint::process()
{
    auto ilh = float(m_config.initial_layer_height.getFloat());
    auto lh = float(m_config.layer_height.getFloat());

    auto initial = m_pri->state();
    if(m_pri) m_pri->update(initial + 10, "Slicing only the first object");

//    for(ModelObject *o : m_model->objects) {
    ModelObject *o = m_model->objects.front();
        if(m_canceled.load()) {
            // clear everything
            return;
        }

        TriangleMesh&& mesh = o->raw_mesh();
        TriangleMeshSlicer slicer(&mesh);
        auto bb3d = mesh.bounding_box();

        auto H = bb3d.max(2) - bb3d.min(2);
        std::vector<float> heights = {ilh};
        for(float h = ilh; h < H; h += lh) heights.emplace_back(h);

        slicer.slice(heights, &m_layers, [](){});
//    }

    if(m_pri) m_pri->update(initial + 100, "Demo slicing done");
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
    if(m_proc) m_proc->stop(); // for now we don't bother

    // this throws
//    m_config.apply(cfg);

    return true;
}

void SLAPrint::export_print_data(const std::string &path)
{
    double width_mm  =  m_config.display_width.getFloat();
    double height_mm =  m_config.display_height.getFloat();
    auto width_px    =  unsigned(m_config.display_pixels_x.getInt());
    auto height_px   =  unsigned(m_config.display_pixels_y.getInt());

    FilePrinter<FilePrinterFormat::PNG> printer(width_mm, height_mm,
                                                width_px, height_px, m_writer);

    printer.layers(unsigned(m_layers.size()));

    auto& layers = m_layers;

    auto& pri = m_pri;
    int st_prev = 0;
    tbb::spin_mutex m;
    const std::string desc = "Rasterizing and compressing sliced layers";

    auto initial = pri->state();
    auto printfn = [&layers, &printer, &pri, initial, &st_prev, &m, desc]
            (unsigned layer_id)
    {
        auto& lyr = layers[layer_id];
        printer.begin_layer(layer_id);
        for(auto& p : lyr) printer.draw_polygon(p, layer_id);
        printer.finish_layer(layer_id);

        auto st = static_cast<int>(layer_id*80.0/layers.size());
        m.lock();
        if( st - st_prev > 10) {
            pri->update(initial + st, desc);
            st_prev = st;
        }
        m.unlock();
    };

    tbb::parallel_for<size_t, decltype(printfn)>(0, layers.size(), printfn);

    pri->update(initial + 90, "Writing layers to disk");
    printer.save(path);
    pri->update(initial + 100, "Writing layers completed");
}

Polyline SLAPrint::bed_shape() const
{
    auto& bedpoints = m_config.bed_shape.values;
    Polyline bed; bed.points.reserve(bedpoints.size());
    for(auto& v : bedpoints)
        bed.append(Point::new_scale(v(0), v(1)));
    return bed;
}

}
