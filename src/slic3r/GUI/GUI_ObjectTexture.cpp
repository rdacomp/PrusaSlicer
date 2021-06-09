#include "libslic3r/libslic3r.h"
#include "GUI_ObjectTexture.hpp"
#include "OptionsGroup.hpp"
#include "GUI_App.hpp"
#include "Plater.hpp"
#include "GUI_ObjectList.hpp"

#include "libslic3r/Model.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#if ENABLE_TEXTURED_VOLUMES

namespace Slic3r {
namespace GUI {

ObjectTexture::ObjectTexture(wxWindow* parent) :
    OG_Settings(parent, true)
{
    m_main_sizer = new wxBoxSizer(wxVERTICAL);
    
    wxBoxSizer* tex_sizer = new wxBoxSizer(wxHORIZONTAL);

    wxStaticText* tex_label = new wxStaticText(m_parent, wxID_ANY, _L("Texture"), wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_MIDDLE);
    m_tex_string = new wxTextCtrl(m_parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    m_tex_string->Enable(false);
    wxButton* tex_browse_btn = new wxButton(m_parent, wxID_ANY, _L("Browse..."));
    m_tex_remove_btn = new wxButton(m_parent, wxID_ANY, _L("Remove"));

    tex_browse_btn->Bind(wxEVT_BUTTON, [this](wxCommandEvent& evt) {
        wxFileDialog dialog(m_parent, _L("Choose a texture file:"), "", "", "Texture files (*.texture)|*.texture", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
        if (dialog.ShowModal() != wxID_OK)
            return;

        std::string filename = dialog.GetPath().ToUTF8().data();
        if (!boost::algorithm::iends_with(filename, ".texture"))
            return;

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture = filename;

        update();
        wxGetApp().plater()->add_textures_to_volumes(obj_idx);
//        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    m_tex_remove_btn->Bind(wxEVT_BUTTON, [this](wxCommandEvent& evt) {
        wxMessageDialog dlg(m_parent, _L("Do you want to remove the texture ?"), wxString(SLIC3R_APP_NAME), wxYES_NO);
        if (dlg.ShowModal() == wxID_YES) {
            const auto& [obj_idx, model_object] = get_model_object();
            if (model_object != nullptr)
                model_object->texture.clear();

            update();
            wxGetApp().plater()->add_textures_to_volumes(obj_idx);
            wxGetApp().obj_list()->del_texture_item();
        }
        });

    tex_sizer->Add(tex_label, 0, wxALIGN_CENTER_VERTICAL);
    tex_sizer->Add(m_tex_string, 1, wxEXPAND | wxLEFT | wxTOP | wxBOTTOM, 5);
    tex_sizer->Add(tex_browse_btn, 0, wxEXPAND | wxALL, 5);
    tex_sizer->Add(m_tex_remove_btn, 0, wxEXPAND | wxALL, 5);

    m_main_sizer->Add(tex_sizer, 1);


    // https://docs.wxwidgets.org/trunk/classwx_property_grid.html


    m_og->activate();
    m_og->sizer->Clear(true);
    m_og->sizer->Add(m_main_sizer, 1, wxEXPAND | wxALL, wxOSX ? 0 : 5);

//    m_grid_sizer = new wxFlexGridSizer(3, 5, wxGetApp().em_unit()); // "Min Z", "Max Z", "Layer height" & buttons sizer
//    m_grid_sizer->SetFlexibleDirection(wxHORIZONTAL);
//
//    // Legend for object layers
//    for (const std::string col : { L("Start at height"), L("Stop at height"), L("Layer height") }) {
//        auto temp = new wxStaticText(m_parent, wxID_ANY, _(col), wxDefaultPosition, /*size*/wxDefaultSize, wxST_ELLIPSIZE_MIDDLE);
//        temp->SetBackgroundStyle(wxBG_STYLE_PAINT);
//        temp->SetFont(wxGetApp().bold_font());
//
//        m_grid_sizer->Add(temp);
//    }
//
//    m_og->activate();
//    m_og->sizer->Clear(true);
//    m_og->sizer->Add(m_grid_sizer, 0, wxEXPAND | wxALL, wxOSX ? 0 : 5);
//
//    m_bmp_delete = ScalableBitmap(parent, "remove_copies"/*"cross"*/);
//    m_bmp_add = ScalableBitmap(parent, "add_copies");

}

void ObjectTexture::msw_rescale()
{
    {
    }

    m_main_sizer->Layout();


//    m_bmp_delete.msw_rescale();
//    m_bmp_add.msw_rescale();
//
//    m_grid_sizer->SetHGap(wxGetApp().em_unit());
//
//    // rescale edit-boxes
//    const int cells_cnt = m_grid_sizer->GetCols() * m_grid_sizer->GetEffectiveRowsCount();
//    for (int i = 0; i < cells_cnt; ++i) {
//        const wxSizerItem* item = m_grid_sizer->GetItem(i);
//        if (item->IsWindow()) {
//            LayerRangeEditor* editor = dynamic_cast<LayerRangeEditor*>(item->GetWindow());
//            if (editor != nullptr)
//                editor->msw_rescale();
//        }
//        else if (item->IsSizer()) // case when we have editor with buttons
//        {
//            wxSizerItem* e_item = item->GetSizer()->GetItem(size_t(0)); // editor
//            if (e_item->IsWindow()) {
//                LayerRangeEditor* editor = dynamic_cast<LayerRangeEditor*>(e_item->GetWindow());
//                if (editor != nullptr)
//                    editor->msw_rescale();
//            }
//
//            if (item->GetSizer()->GetItemCount() > 2) // if there are Add/Del buttons
//                for (size_t btn : {2, 3}) { // del_btn, add_btn
//                    wxSizerItem* b_item = item->GetSizer()->GetItem(btn);
//                    if (b_item->IsWindow()) {
//                        auto button = dynamic_cast<PlusMinusButton*>(b_item->GetWindow());
//                        if (button != nullptr)
//                            button->msw_rescale();
//                    }
//                }
//        }
//    }
//    m_grid_sizer->Layout();
}

void ObjectTexture::sys_color_changed()
{



//    m_bmp_delete.msw_rescale();
//    m_bmp_add.msw_rescale();
//
//    m_grid_sizer->SetHGap(wxGetApp().em_unit());
//
//    // rescale edit-boxes
//    const int cells_cnt = m_grid_sizer->GetCols() * m_grid_sizer->GetEffectiveRowsCount();
//    for (int i = 0; i < cells_cnt; ++i) {
//        const wxSizerItem* item = m_grid_sizer->GetItem(i);
//        if (item->IsSizer()) {// case when we have editor with buttons
//            const std::vector<size_t> btns = { 2, 3 };  // del_btn, add_btn
//            for (auto btn : btns) {
//                wxSizerItem* b_item = item->GetSizer()->GetItem(btn);
//                if (b_item->IsWindow()) {
//                    auto button = dynamic_cast<PlusMinusButton*>(b_item->GetWindow());
//                    if (button != nullptr)
//                        button->msw_rescale();
//                }
//            }
//        }
//    }
//    m_grid_sizer->Layout();
}

void ObjectTexture::UpdateAndShow(const bool show)
{
    if (show)
        update();

    OG_Settings::UpdateAndShow(show);
}

void ObjectTexture::update()
{
    const std::pair<int, ModelObject*>& model_object = get_model_object();
    bool has_texture = model_object.second != nullptr && !model_object.second->texture.empty();

    // update widgets
    m_tex_string->SetValue(has_texture ? wxString(boost::filesystem::path(model_object.second->texture).stem().string()) : "");
    m_tex_remove_btn->Enable(has_texture);

    // show/hide icon into object list
    ObjectDataViewModel* objects_model = wxGetApp().obj_list()->GetModel();
    wxDataViewItem selectable_item = wxGetApp().obj_list()->GetSelection();
    selectable_item = objects_model->GetParent(selectable_item);
    wxDataViewItem texture_item = objects_model->GetItemByType(selectable_item, itTexture);
    ObjectDataViewModelNode* node = static_cast<ObjectDataViewModelNode*>(texture_item.GetID());
    node->SetBitmap(has_texture ? create_scaled_bitmap("edit_texture") : wxBitmap());
    wxGetApp().obj_list()->Refresh();
}

std::pair<int, ModelObject*> ObjectTexture::get_model_object()
{
    ObjectList* objects_ctrl = wxGetApp().obj_list();
    if (objects_ctrl->multiple_selection())
        return { -1, nullptr };

    const auto item = objects_ctrl->GetSelection();
    if (!item)
        return { -1, nullptr };

    const ItemType type = objects_ctrl->GetModel()->GetItemType(item);
    if (!(type & itTexture))
        return { -1, nullptr };

    const int obj_idx = objects_ctrl->get_selected_obj_idx();
    if (obj_idx < 0)
        return { -1, nullptr };

    return { obj_idx, objects_ctrl->object(obj_idx) };
}

} // namespace GUI
} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES
