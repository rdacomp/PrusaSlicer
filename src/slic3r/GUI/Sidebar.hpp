#ifndef SIDEBAR_HPP
#define SIDEBAR_HPP

#include <memory>

#include <wx/panel.h>
#include <wx/scrolwin.h>

#include "libslic3r/PrintConfig.hpp"

#include "Preset.hpp"
#include "GUI_ObjectList.hpp"
#include "GUI_ObjectManipulation.hpp"
#include "GUI_ObjectSettings.hpp"
#include "Event.hpp"
#include "wxExtensions.hpp"

namespace Slic3r { namespace GUI {

wxDECLARE_EVENT(EVT_SCHEDULE_BACKGROUND_PROCESS, SimpleEvent);

class Plater;

class PresetComboBox : public PresetBitmapComboBox
{
public:
    PresetComboBox(wxWindow *parent, Preset::Type preset_type);
    ~PresetComboBox();
    
        ScalableButton* edit_btn { nullptr };
    
        enum LabelItemType {
            LABEL_ITEM_MARKER = 0xffffff01,
            LABEL_ITEM_WIZARD_PRINTERS,
            LABEL_ITEM_WIZARD_FILAMENTS,
            LABEL_ITEM_WIZARD_MATERIALS,
            
            LABEL_ITEM_MAX
            };
    
    void set_label_marker(int item, LabelItemType label_item_type = LABEL_ITEM_MARKER);
    void set_extruder_idx(const int extr_idx)   { extruder_idx = extr_idx; }
    int  get_extruder_idx() const               { return extruder_idx; }
    int  em_unit() const                        { return m_em_unit; }
    void check_selection(int selection);
    
    void msw_rescale();
    
private:
    typedef std::size_t Marker;
    
    Preset::Type preset_type;
    int last_selected;
    int extruder_idx = -1;
    int m_em_unit;
};

enum class ActionButtonType : int {
    abReslice,
    abExport,
    abSendGCode
};

class Sidebar : public wxPanel
{
    ConfigOptionMode    m_mode;
public:
    Sidebar(Plater *parent);
    Sidebar(Sidebar &&) = delete;
    Sidebar(const Sidebar &) = delete;
    Sidebar &operator=(Sidebar &&) = delete;
    Sidebar &operator=(const Sidebar &) = delete;
    ~Sidebar();
    
    void init_filament_combo(PresetComboBox **combo, const int extr_idx);
    void remove_unused_filament_combos(const size_t current_extruder_count);
    void update_all_preset_comboboxes();
    void update_presets(Slic3r::Preset::Type preset_type);
    void update_mode_sizer() const;
    void update_reslice_btn_tooltip() const;
    void msw_rescale();
    
    ObjectManipulation*     obj_manipul();
    ObjectList*             obj_list();
    ObjectSettings*         obj_settings();
    ObjectLayers*           obj_layers();
    wxScrolledWindow*       scrolled_panel();
    wxPanel*                presets_panel();
    
    ConfigOptionsGroup*     og_freq_chng_params(const bool is_fff);
    wxButton*               get_wiping_dialog_button();
    void                    update_objects_list_extruder_column(size_t extruders_count);
    void                    show_info_sizer();
    void                    show_sliced_info_sizer(const bool show);
    void                    update_sliced_info_sizer();
    void                    enable_buttons(bool enable);
    void                    set_btn_label(const ActionButtonType btn_type, const wxString& label) const;
    bool                    show_reslice(bool show) const;
    bool                    show_export(bool show) const;
    bool                    show_send(bool show) const;
    bool                    show_disconnect(bool show)const;
    bool                    show_export_removable(bool show) const;
    bool                    is_multifilament();
    void                    update_mode();
    
    std::vector<PresetComboBox*>& combos_filament();
    
private:
    struct priv;
    std::unique_ptr<priv> p;
};

}} // namespace Slic3r::GUI

#endif // SIDEBAR_HPP
