#include "libslic3r/libslic3r.h"
#include "GUI_ObjectTexture.hpp"
#if ENABLE_TEXTURED_VOLUMES
#include "3DScene.hpp"
#include "OptionsGroup.hpp"
#include "GUI_App.hpp"
#include "Plater.hpp"
#include "GUI_ObjectList.hpp"

#include "libslic3r/Model.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

namespace Slic3r {
namespace GUI {

ObjectTexture::ObjectTexture(wxWindow* parent) :
    OG_Settings(parent, true)
{
    m_bmp_delete = ScalableBitmap(m_parent, "cross");
    m_bmp_delete_focus = ScalableBitmap(m_parent, "cross_focus");
    m_bmp_delete_disabled = ScalableBitmap(m_parent, "transparent.png");

    m_main_sizer = new wxBoxSizer(wxVERTICAL);
    
    m_tex_sizer = init_tex_sizer();
    m_metadata_sizer = init_metadata_sizer();

    m_main_sizer->Add(m_tex_sizer, 0, wxEXPAND | wxALL);
    m_main_sizer->Add(m_metadata_sizer, 0, wxEXPAND | wxALL);
    m_main_sizer->Hide(m_metadata_sizer);

    m_og->activate();
    m_og->sizer->Clear(true);
    m_og->sizer->Add(m_main_sizer, 0, wxEXPAND | wxALL, wxOSX ? 0 : 5);
}

void ObjectTexture::msw_rescale()
{
    m_bmp_delete.msw_rescale();
    m_bmp_delete_focus.msw_rescale();
    m_bmp_delete_disabled.msw_rescale();

    m_main_sizer->Layout();
}

void ObjectTexture::sys_color_changed()
{
    m_bmp_delete.msw_rescale();
    m_bmp_delete_focus.msw_rescale();
    m_bmp_delete_disabled.msw_rescale();

    m_main_sizer->Layout();
}

void ObjectTexture::UpdateAndShow(const bool show)
{
    if (show)
        update();

    OG_Settings::UpdateAndShow(show);
}

wxBoxSizer* ObjectTexture::init_tex_sizer()
{
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

    wxStaticText* label = new wxStaticText(m_parent, wxID_ANY, _L("Texture"), wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_MIDDLE);
    m_tex_string = new wxTextCtrl(m_parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    m_tex_string->Enable(false);
    wxButton* browse_btn = new wxButton(m_parent, wxID_ANY, _L("Browse..."));

    m_tex_delete_btn = new ScalableButton(m_parent, wxID_ANY, m_bmp_delete);
    m_tex_delete_btn->SetToolTip(_L("Remove texture"));
    m_tex_delete_btn->SetBitmapFocus(m_bmp_delete_focus.bmp());
    m_tex_delete_btn->SetBitmapHover(m_bmp_delete_focus.bmp());
    m_tex_delete_btn->SetBitmapDisabled_(m_bmp_delete_disabled);

    browse_btn->Bind(wxEVT_BUTTON, [this](wxCommandEvent& evt) {
        wxFileDialog dialog(m_parent, _L("Choose a texture file:"), "", "", file_wildcards(FT_OBJ_TEX), wxFD_OPEN | wxFD_FILE_MUST_EXIST);
        if (dialog.ShowModal() != wxID_OK)
            return;

        std::string filename = dialog.GetPath().ToUTF8().data();
        if (!boost::algorithm::iends_with(filename, ".png") && !boost::algorithm::iends_with(filename, ".texture"))
            return;

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr) {
            const std::string& curr_name = model_object->texture.name;
            if (curr_name.empty())
                wxGetApp().plater()->take_snapshot(_L("Add texture"));
            else {
                wxGetApp().plater()->take_snapshot(_L("Change texture"));
                wxGetApp().plater()->remove_object_texture(curr_name);
            }

            // add texture to the cache
            std::string tex_name = wxGetApp().plater()->add_object_texture(filename);
            // update object texture
            model_object->texture.name = tex_name;

            update();
            wxGetApp().plater()->update_volumes_texture_from_objects();
            wxGetApp().obj_list()->changed_object(obj_idx);
        }
        });

    m_tex_delete_btn->Bind(wxEVT_BUTTON, [this](wxEvent& event) {
        wxMessageDialog dlg(m_parent, _L("Do you really want to remove the texture ?"), wxString(SLIC3R_APP_NAME), wxYES_NO);
        if (dlg.ShowModal() == wxID_YES) {
            const auto& model_object = get_model_object();
            if (model_object.second != nullptr) {
                wxGetApp().plater()->take_snapshot(_L("Remove texture"));

                // remove texture from the cache
                wxGetApp().plater()->remove_object_texture(model_object.second->texture.name);
                // reset object texture
                model_object.second->texture.reset();

                update();
                wxGetApp().plater()->update_volumes_texture_from_objects();
                wxGetApp().obj_list()->del_texture_item();
            }
        }
        });

    sizer->Add(label, 0, wxALIGN_CENTER_VERTICAL);
    sizer->Add(m_tex_string, 1, wxEXPAND | wxLEFT | wxTOP | wxBOTTOM, 5);
    sizer->Add(browse_btn, 0, wxEXPAND | wxALL, 5);
    sizer->Add(m_tex_delete_btn, 0, wxEXPAND | wxALL, 5);

    return sizer;
}

wxSizer* ObjectTexture::init_metadata_sizer()
{
    wxStaticBoxSizer* sizer = new wxStaticBoxSizer(wxVERTICAL, m_parent, _L("Parameters"));
    wxFlexGridSizer* grid_sizer = new wxFlexGridSizer(2, 5, 5);

    grid_sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("Mapping type")), 0, wxALIGN_CENTER_VERTICAL);
    grid_sizer->Add(init_map_sizer());

    grid_sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("Move")), 0, wxALIGN_CENTER_VERTICAL);
    grid_sizer->Add(init_move_sizer());

    grid_sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("Repeat")), 0, wxALIGN_CENTER_VERTICAL);
    grid_sizer->Add(init_repeat_sizer());
    
    grid_sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("Rotation")), 0, wxALIGN_CENTER_VERTICAL);
    grid_sizer->Add(init_rot_sizer());

    grid_sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("Wrapping")), 0, wxALIGN_CENTER_VERTICAL);
    grid_sizer->Add(init_wrap_sizer());

    sizer->Add(grid_sizer, 0, wxEXPAND | wxALL, 5);
    return sizer;
}

wxBoxSizer* ObjectTexture::init_map_sizer()
{
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

    const wxString mappings[] = { _L("Cubic"), _L("Cylindrical"), _L("Spherical") };
    m_map_choices = new wxChoice(m_parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxArrayString(3, mappings));

    m_map_choices->Bind(wxEVT_CHOICE, [this](wxEvent& event) {
        int selection = m_map_choices->GetSelection();
        if (selection == wxNOT_FOUND)
            return;

        wxGetApp().plater()->take_snapshot(_L("Changed texture mapping"));

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture.mapping = static_cast<TextureMetadata::EMapping>(selection);

        update();
        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    sizer->Add(m_map_choices, 0, wxEXPAND | wxALL);

    return sizer;
}

wxBoxSizer* ObjectTexture::init_move_sizer()
{
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

    m_move_u_spin = new wxSpinCtrlDouble(m_parent, wxID_ANY, "", wxDefaultPosition, wxSize(100, -1), wxSP_ARROW_KEYS, 0.0, 100.0, 0.0, 1.0);
    m_move_u_spin->SetDigits(2);
    m_move_v_spin = new wxSpinCtrlDouble(m_parent, wxID_ANY, "", wxDefaultPosition, wxSize(100, -1), wxSP_ARROW_KEYS, 0.0, 100.0, 0.0, 1.0);
    m_move_v_spin->SetDigits(2);

    m_move_u_spin->Bind(wxEVT_SPINCTRLDOUBLE, [this](wxEvent& event) {
        wxGetApp().plater()->take_snapshot(_L("Changed texture offset u"));

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture.offset_u = 0.01f * static_cast<float>(m_move_u_spin->GetValue());

        update();
        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    m_move_v_spin->Bind(wxEVT_SPINCTRLDOUBLE, [this](wxEvent& event) {
        wxGetApp().plater()->take_snapshot(_L("Changed texture offset v"));

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture.offset_v = 0.01f * static_cast<float>(m_move_v_spin->GetValue());

        update();
        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("U")), 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(5);
    sizer->Add(m_move_u_spin, 0, wxEXPAND | wxALL);
    sizer->AddSpacer(10);
    sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("V")), 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(5);
    sizer->Add(m_move_v_spin, 0, wxEXPAND | wxALL);

    return sizer;
}

wxBoxSizer* ObjectTexture::init_repeat_sizer()
{
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

    m_repeat_u_spin = new wxSpinCtrlDouble(m_parent, wxID_ANY, "", wxDefaultPosition, wxSize(100, -1), wxSP_ARROW_KEYS, 0.1, 100.0, 1.0, 1.0);
    m_repeat_u_spin->SetDigits(2);
    m_repeat_v_spin = new wxSpinCtrlDouble(m_parent, wxID_ANY, "", wxDefaultPosition, wxSize(100, -1), wxSP_ARROW_KEYS, 0.1, 100.0, 1.0, 1.0);
    m_repeat_v_spin->SetDigits(2);

    m_repeat_u_spin->Bind(wxEVT_SPINCTRLDOUBLE, [this](wxEvent& event) {
        wxGetApp().plater()->take_snapshot(_L("Changed texture repeat u"));

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture.repeat_u = static_cast<float>(m_repeat_u_spin->GetValue());

        update();
        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    m_repeat_v_spin->Bind(wxEVT_SPINCTRLDOUBLE, [this](wxEvent& event) {
        wxGetApp().plater()->take_snapshot(_L("Changed texture repeat v"));

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture.repeat_v = static_cast<float>(m_repeat_v_spin->GetValue());

        update();
        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("U")), 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(5);
    sizer->Add(m_repeat_u_spin, 0, wxEXPAND | wxALL);
    sizer->AddSpacer(10);
    sizer->Add(new wxStaticText(m_parent, wxID_ANY, _L("V")), 0, wxALIGN_CENTER_VERTICAL);
    sizer->AddSpacer(5);
    sizer->Add(m_repeat_v_spin, 0, wxEXPAND | wxALL);

    return sizer;
}

wxBoxSizer* ObjectTexture::init_rot_sizer()
{
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

    m_rotation_spin = new wxSpinCtrlDouble(m_parent, wxID_ANY, "", wxDefaultPosition, wxSize(100, -1), wxSP_ARROW_KEYS, 0.0, 360.0, 0.0, 1.0);
    m_rotation_spin->SetDigits(2);

    m_rotation_spin->Bind(wxEVT_SPINCTRLDOUBLE, [this](wxEvent& event) {
        wxGetApp().plater()->take_snapshot(_L("Changed texture rotation"));

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture.rotation = static_cast<float>(m_rotation_spin->GetValue());

        update();
        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    sizer->Add(m_rotation_spin, 0, wxEXPAND | wxALL);

    return sizer;
}

wxBoxSizer* ObjectTexture::init_wrap_sizer()
{
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);

    const wxString wrappings[] = { _L("Repeat"), _L("Mirror"), _L("Clamp to edge"), _L("Clamp to border") };
    m_wrap_choices = new wxChoice(m_parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxArrayString(4, wrappings));

    m_wrap_choices->Bind(wxEVT_CHOICE, [this](wxEvent& event) {
        int selection = m_wrap_choices->GetSelection();
        if (selection == wxNOT_FOUND)
            return;

        wxGetApp().plater()->take_snapshot(_L("Changed texture wrapping"));

        const auto& [obj_idx, model_object] = get_model_object();
        if (model_object != nullptr)
            model_object->texture.wrapping = static_cast<TextureMetadata::EWrapping>(selection);

        update();
        wxGetApp().obj_list()->changed_object(obj_idx);
        });

    sizer->Add(m_wrap_choices, 0, wxEXPAND | wxALL);

    return sizer;
}

void ObjectTexture::update()
{
    const std::pair<int, ModelObject*>& model_object = get_model_object();
    bool has_texture = model_object.second != nullptr && !model_object.second->texture.name.empty();

    // update texture widgets
    std::string name = has_texture ? model_object.second->texture.name : "";
    m_tex_string->SetValue(name.empty() ? _L("None") : _(name));
    m_tex_delete_btn->Enable(has_texture);

    if (has_texture) {
        if (m_main_sizer->GetItem(m_metadata_sizer) == nullptr) {
            // show metadata
            m_main_sizer->Add(m_metadata_sizer, 0, wxEXPAND | wxALL);
            m_main_sizer->Show(m_metadata_sizer);
            m_parent->Layout();
        }

        // update widgets data
        m_map_choices->SetSelection(static_cast<int>(model_object.second->texture.mapping));
        m_move_u_spin->SetValue(100.0 * static_cast<double>(model_object.second->texture.offset_u));
        m_move_v_spin->SetValue(100.0 * static_cast<double>(model_object.second->texture.offset_v));
        m_repeat_u_spin->SetValue(static_cast<double>(model_object.second->texture.repeat_u));
        m_repeat_v_spin->SetValue(static_cast<double>(model_object.second->texture.repeat_v));
        m_rotation_spin->SetValue(static_cast<double>(model_object.second->texture.rotation));
        m_wrap_choices->SetSelection(static_cast<int>(model_object.second->texture.wrapping));
    }
    else {
        if (m_main_sizer->GetItem(m_metadata_sizer) != nullptr) {
            // hide metadata
            m_main_sizer->Hide(m_metadata_sizer);
            m_main_sizer->Detach(m_metadata_sizer);
            m_parent->Layout();
        }
    }

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
