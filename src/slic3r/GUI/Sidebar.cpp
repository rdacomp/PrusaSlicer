#include "Sidebar.hpp"

#include <boost/algorithm/string.hpp>

#include <wx/colordlg.h>
#include <wx/wupdlock.h>

#include "libslic3r/SLAPrint.hpp"

#include "OptionsGroup.hpp"
#include "PresetBundle.hpp"
#include "Tab.hpp"
#include "WipeTowerDialog.hpp"
#include "BackgroundSlicingProcess.hpp"

namespace Slic3r { namespace GUI {

// Frequently changed parameters

class FreqChangedParams : public OG_Settings
{
    double		    m_brim_width = 0.0;
    wxButton*       m_wiping_dialog_button{ nullptr };
    wxSizer*        m_sizer {nullptr};
    
    std::shared_ptr<ConfigOptionsGroup> m_og_sla;
    std::vector<ScalableButton*>        m_empty_buttons;
public:
    FreqChangedParams(wxWindow* parent);
    
    wxButton*       get_wiping_dialog_button() { return m_wiping_dialog_button; }
    wxSizer*        get_sizer() override;
    ConfigOptionsGroup* get_og(const bool is_fff);
    void            Show(const bool is_fff) override;
    
    void            msw_rescale();
};

void FreqChangedParams::msw_rescale()
{
    m_og->msw_rescale();
    m_og_sla->msw_rescale();
    
    for (auto btn: m_empty_buttons)
        btn->msw_rescale();
}

FreqChangedParams::FreqChangedParams(wxWindow* parent) :
    OG_Settings(parent, false)
{
    DynamicPrintConfig*	config = &wxGetApp().preset_bundle->prints.get_edited_preset().config;
    
    // Frequently changed parameters for FFF_technology
    m_og->set_config(config);
    m_og->hide_labels();
    
    m_og->m_on_change = [config, this](t_config_option_key opt_key, boost::any value) {
        Tab* tab_print = wxGetApp().get_tab(Preset::TYPE_PRINT);
        if (!tab_print) return;
        
        if (opt_key == "fill_density") {
            value = m_og->get_config_value(*config, opt_key);
            tab_print->set_value(opt_key, value);
            tab_print->update();
        }
        else{
            DynamicPrintConfig new_conf = *config;
            if (opt_key == "brim") {
                double new_val;
                double brim_width = config->opt_float("brim_width");
                if (boost::any_cast<bool>(value) == true)
                {
                    new_val = m_brim_width == 0.0 ? 5 :
                                                    m_brim_width < 0.0 ? m_brim_width * (-1) :
                                                                         m_brim_width;
                }
                else {
                    m_brim_width = brim_width * (-1);
                    new_val = 0;
                }
                new_conf.set_key_value("brim_width", new ConfigOptionFloat(new_val));
            }
            else {
                assert(opt_key == "support");
                const wxString& selection = boost::any_cast<wxString>(value);
                PrinterTechnology printer_technology = wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology();
                
                auto support_material = selection == _("None") ? false : true;
                new_conf.set_key_value("support_material", new ConfigOptionBool(support_material));
                
                if (selection == _("Everywhere")) {
                    new_conf.set_key_value("support_material_buildplate_only", new ConfigOptionBool(false));
                    if (printer_technology == ptFFF)
                        new_conf.set_key_value("support_material_auto", new ConfigOptionBool(true));
                } else if (selection == _("Support on build plate only")) {
                    new_conf.set_key_value("support_material_buildplate_only", new ConfigOptionBool(true));
                    if (printer_technology == ptFFF)
                        new_conf.set_key_value("support_material_auto", new ConfigOptionBool(true));
                } else if (selection == _("For support enforcers only")) {
                    assert(printer_technology == ptFFF);
                    new_conf.set_key_value("support_material_buildplate_only", new ConfigOptionBool(false));
                    new_conf.set_key_value("support_material_auto", new ConfigOptionBool(false));
                }
            }
            tab_print->load_config(new_conf);
        }
        
        tab_print->update_dirty();
    };
    
    
    Line line = Line { "", "" };
    
    ConfigOptionDef support_def;
    support_def.label = L("Supports");
    support_def.type = coStrings;
    support_def.gui_type = "select_open";
    support_def.tooltip = L("Select what kind of support do you need");
    support_def.enum_labels.push_back(L("None"));
    support_def.enum_labels.push_back(L("Support on build plate only"));
    support_def.enum_labels.push_back(L("For support enforcers only"));
    support_def.enum_labels.push_back(L("Everywhere"));
    support_def.set_default_value(new ConfigOptionStrings{ "None" });
    Option option = Option(support_def, "support");
    option.opt.full_width = true;
    line.append_option(option);
    
    /* Not a best solution, but
     * Temporary workaround for right border alignment
     */
    auto empty_widget = [this] (wxWindow* wparent) {
        auto sizer = new wxBoxSizer(wxHORIZONTAL);
        auto btn = new ScalableButton(wparent, wxID_ANY, "mirroring_transparent.png", wxEmptyString,
                                      wxDefaultSize, wxDefaultPosition, wxBU_EXACTFIT | wxNO_BORDER | wxTRANSPARENT_WINDOW);
        sizer->Add(btn, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, int(0.3 * wxGetApp().em_unit()));
        m_empty_buttons.push_back(btn);
        return sizer;
    };
    line.append_widget(empty_widget);
    
    m_og->append_line(line);
    
    
    line = Line { "", "" };
    
    option = m_og->get_option("fill_density");
    option.opt.label = L("Infill");
    option.opt.width = 7/*6*/;
    option.opt.sidetext = "   ";
    line.append_option(option);
    
    m_brim_width = config->opt_float("brim_width");
    ConfigOptionDef def;
    def.label = L("Brim");
    def.type = coBool;
    def.tooltip = L("This flag enables the brim that will be printed around each object on the first layer.");
    def.gui_type = "";
    def.set_default_value(new ConfigOptionBool{ m_brim_width > 0.0 ? true : false });
    option = Option(def, "brim");
    option.opt.sidetext = "";
    line.append_option(option);

    auto wiping_dialog_btn = [this](wxWindow *wparent) {
        m_wiping_dialog_button = new wxButton(wparent, wxID_ANY,
                                              _(L("Purging volumes")) + dots,
                                              wxDefaultPosition,
                                              wxDefaultSize, wxBU_EXACTFIT);
        
        m_wiping_dialog_button->SetFont(wxGetApp().normal_font());
        auto sizer = new wxBoxSizer(wxHORIZONTAL);
        sizer->Add(m_wiping_dialog_button, 0, wxALIGN_CENTER_VERTICAL);
        
        m_wiping_dialog_button->Bind(
            wxEVT_BUTTON, ([wparent](wxCommandEvent &) {
                auto &project_config = wxGetApp().preset_bundle->project_config;
                const std::vector<double> &init_matrix =
                    (project_config.option<ConfigOptionFloats>(
                         "wiping_volumes_matrix"))
                        ->values;
                const std::vector<double> &init_extruders =
                    (project_config.option<ConfigOptionFloats>(
                         "wiping_volumes_extruders"))
                        ->values;

                const std::vector<std::string> extruder_colours =
                    wxGetApp()
                        .plater()
                        ->get_extruder_colors_from_plater_config();

                WipingDialog dlg(wparent, cast<float>(init_matrix),
                                 cast<float>(init_extruders),
                                 extruder_colours);

                if (dlg.ShowModal() == wxID_OK) {
                    std::vector<float> matrix    = dlg.get_matrix();
                    std::vector<float> extruders = dlg.get_extruders();
                    (project_config.option<ConfigOptionFloats>(
                         "wiping_volumes_matrix"))
                        ->values = std::vector<double>(matrix.begin(),
                                                       matrix.end());
                    (project_config.option<ConfigOptionFloats>(
                         "wiping_volumes_extruders"))
                        ->values = std::vector<double>(extruders.begin(),
                                                       extruders.end());
                    wxPostEvent(wparent,
                                SimpleEvent(EVT_SCHEDULE_BACKGROUND_PROCESS,
                                            wparent));
                }
            }));

        auto btn = new ScalableButton(wparent, wxID_ANY,
                                      "mirroring_transparent.png",
                                      wxEmptyString, wxDefaultSize,
                                      wxDefaultPosition,
                                      wxBU_EXACTFIT | wxNO_BORDER |
                                          wxTRANSPARENT_WINDOW);

        sizer->Add(btn, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT,
                   int(0.3 * wxGetApp().em_unit()));
        
        m_empty_buttons.emplace_back(btn);
        
        return sizer;
    };
    
    line.append_widget(wiping_dialog_btn);
    
    m_og->append_line(line);
    
    
    // Frequently changed parameters for SLA_technology
    m_og_sla = std::make_shared<ConfigOptionsGroup>(parent, "");
    m_og_sla->hide_labels();
    DynamicPrintConfig*	config_sla = &wxGetApp().preset_bundle->sla_prints.get_edited_preset().config;
    m_og_sla->set_config(config_sla);
    
    m_og_sla->m_on_change = [config_sla](t_config_option_key opt_key, boost::any value) {
        Tab* tab = wxGetApp().get_tab(Preset::TYPE_SLA_PRINT);
        if (!tab) return;
        
        DynamicPrintConfig new_conf = *config_sla;
        if (opt_key == "pad") {
            const wxString& selection = boost::any_cast<wxString>(value);
            
            const bool pad_enable = selection == _("None") ? false : true;
            new_conf.set_key_value("pad_enable", new ConfigOptionBool(pad_enable));
            
            if (selection == _("Below object"))
                new_conf.set_key_value("pad_around_object", new ConfigOptionBool(false));
            else if (selection == _("Around object"))
                new_conf.set_key_value("pad_around_object", new ConfigOptionBool(true));
        }
        else
        {
            assert(opt_key == "support");
            const wxString& selection = boost::any_cast<wxString>(value);
            
            const bool supports_enable = selection == _("None") ? false : true;
            new_conf.set_key_value("supports_enable", new ConfigOptionBool(supports_enable));
            
            if (selection == _("Everywhere"))
                new_conf.set_key_value("support_buildplate_only", new ConfigOptionBool(false));
            else if (selection == _("Support on build plate only"))
                new_conf.set_key_value("support_buildplate_only", new ConfigOptionBool(true));
        }
        
        tab->load_config(new_conf);
        tab->update_dirty();
    };
    
    line = Line{ "", "" };
    
    ConfigOptionDef support_def_sla = support_def;
    support_def_sla.set_default_value(new ConfigOptionStrings{ "None" });
    assert(support_def_sla.enum_labels[2] == L("For support enforcers only"));
    support_def_sla.enum_labels.erase(support_def_sla.enum_labels.begin() + 2);
    option = Option(support_def_sla, "support");
    option.opt.full_width = true;
    line.append_option(option);
    line.append_widget(empty_widget);
    m_og_sla->append_line(line);
    
    line = Line{ "", "" };
    
    ConfigOptionDef pad_def;
    pad_def.label = L("Pad");
    pad_def.type = coStrings;
    pad_def.gui_type = "select_open";
    pad_def.tooltip = L("Select what kind of pad do you need");
    pad_def.enum_labels.push_back(L("None"));
    pad_def.enum_labels.push_back(L("Below object"));
    pad_def.enum_labels.push_back(L("Around object"));
    pad_def.set_default_value(new ConfigOptionStrings{ "Below object" });
    option = Option(pad_def, "pad");
    option.opt.full_width = true;
    line.append_option(option);
    line.append_widget(empty_widget);
    
    m_og_sla->append_line(line);
    
    m_sizer = new wxBoxSizer(wxVERTICAL);
    m_sizer->Add(m_og->sizer, 0, wxEXPAND);
    m_sizer->Add(m_og_sla->sizer, 0, wxEXPAND);
}


wxSizer* FreqChangedParams::get_sizer()
{
    return m_sizer;
}

void FreqChangedParams::Show(const bool is_fff)
{
    const bool is_wdb_shown = m_wiping_dialog_button->IsShown();
    m_og->Show(is_fff);
    m_og_sla->Show(!is_fff);
    
    // correct showing of the FreqChangedParams sizer when m_wiping_dialog_button is hidden
    if (is_fff && !is_wdb_shown)
        m_wiping_dialog_button->Hide();
}

ConfigOptionsGroup* FreqChangedParams::get_og(const bool is_fff)
{
    return is_fff ? m_og.get() : m_og_sla.get();
}


class ObjectInfo : public wxStaticBoxSizer
{
public:
    ObjectInfo(wxWindow *parent);
    
    wxStaticBitmap *manifold_warning_icon;
    wxStaticText *info_size;
    wxStaticText *info_volume;
    wxStaticText *info_facets;
    wxStaticText *info_materials;
    wxStaticText *info_manifold;
    
    wxStaticText *label_volume;
    wxStaticText *label_materials;
    std::vector<wxStaticText *> sla_hidden_items;
    
    bool        showing_manifold_warning_icon;
    void        show_sizer(bool show);
    void        msw_rescale();
};

ObjectInfo::ObjectInfo(wxWindow *parent) :
    wxStaticBoxSizer(new wxStaticBox(parent, wxID_ANY, _(L("Info"))), wxVERTICAL)
{
    GetStaticBox()->SetFont(wxGetApp().bold_font());
    
    auto *grid_sizer = new wxFlexGridSizer(4, 5, 15);
    grid_sizer->SetFlexibleDirection(wxHORIZONTAL);
    //     grid_sizer->AddGrowableCol(1, 1);
    //     grid_sizer->AddGrowableCol(3, 1);
    
    auto init_info_label = [parent, grid_sizer](wxStaticText **info_label, wxString text_label) {
        auto *text = new wxStaticText(parent, wxID_ANY, text_label+":");
        text->SetFont(wxGetApp().small_font());
        *info_label = new wxStaticText(parent, wxID_ANY, "");
        (*info_label)->SetFont(wxGetApp().small_font());
        grid_sizer->Add(text, 0);
        grid_sizer->Add(*info_label, 0);
        return text;
    };
    
    init_info_label(&info_size, _(L("Size")));
    label_volume = init_info_label(&info_volume, _(L("Volume")));
    init_info_label(&info_facets, _(L("Facets")));
    label_materials = init_info_label(&info_materials, _(L("Materials")));
    Add(grid_sizer, 0, wxEXPAND);
    
    auto *info_manifold_text = new wxStaticText(parent, wxID_ANY, _(L("Manifold")) + ":");
    info_manifold_text->SetFont(wxGetApp().small_font());
    info_manifold = new wxStaticText(parent, wxID_ANY, "");
    info_manifold->SetFont(wxGetApp().small_font());
    manifold_warning_icon = new wxStaticBitmap(parent, wxID_ANY, create_scaled_bitmap("exclamation"));
    auto *sizer_manifold = new wxBoxSizer(wxHORIZONTAL);
    sizer_manifold->Add(info_manifold_text, 0);
    sizer_manifold->Add(manifold_warning_icon, 0, wxLEFT, 2);
    sizer_manifold->Add(info_manifold, 0, wxLEFT, 2);
    Add(sizer_manifold, 0, wxEXPAND | wxTOP, 4);
    
    sla_hidden_items = { label_volume, info_volume, label_materials, info_materials };
}

void ObjectInfo::show_sizer(bool show)
{
    Show(show);
    if (show)
        manifold_warning_icon->Show(showing_manifold_warning_icon && show);
}

void ObjectInfo::msw_rescale()
{
    manifold_warning_icon->SetBitmap(create_scaled_bitmap("exclamation"));
}

enum SlicedInfoIdx
{
    siFilament_m,
    siFilament_mm3,
    siFilament_g,
    siMateril_unit,
    siCost,
    siEstimatedTime,
    siWTNumbetOfToolchanges,
    
    siCount
};

class SlicedInfo : public wxStaticBoxSizer
{
public:
    SlicedInfo(wxWindow *parent);
    void SetTextAndShow(SlicedInfoIdx idx, const wxString& text, const wxString& new_label="");
    
private:
    std::vector<std::pair<wxStaticText*, wxStaticText*>> info_vec;
};

SlicedInfo::SlicedInfo(wxWindow *parent) :
    wxStaticBoxSizer(new wxStaticBox(parent, wxID_ANY, _(L("Sliced Info"))), wxVERTICAL)
{
    GetStaticBox()->SetFont(wxGetApp().bold_font());
    
    auto *grid_sizer = new wxFlexGridSizer(2, 5, 15);
    grid_sizer->SetFlexibleDirection(wxVERTICAL);
    
    info_vec.reserve(siCount);
    
    auto init_info_label = [this, parent, grid_sizer](wxString text_label) {
        auto *text = new wxStaticText(parent, wxID_ANY, text_label);
        text->SetFont(wxGetApp().small_font());
        auto info_label = new wxStaticText(parent, wxID_ANY, "N/A");
        info_label->SetFont(wxGetApp().small_font());
        grid_sizer->Add(text, 0);
        grid_sizer->Add(info_label, 0);
        info_vec.push_back(std::pair<wxStaticText*, wxStaticText*>(text, info_label));
    };
    
    init_info_label(_(L("Used Filament (m)")));
    init_info_label(_(L("Used Filament (mm³)")));
    init_info_label(_(L("Used Filament (g)")));
    init_info_label(_(L("Used Material (unit)")));
    init_info_label(_(L("Cost (money)")));
    init_info_label(_(L("Estimated printing time")));
    init_info_label(_(L("Number of tool changes")));
    
    Add(grid_sizer, 0, wxEXPAND);
    this->Show(false);
}

void SlicedInfo::SetTextAndShow(SlicedInfoIdx idx, const wxString& text, const wxString& new_label/*=""*/)
{
    const bool show = text != "N/A";
    if (show)
        info_vec[idx].second->SetLabelText(text);
    if (!new_label.IsEmpty())
        info_vec[idx].first->SetLabelText(new_label);
    info_vec[idx].first->Show(show);
    info_vec[idx].second->Show(show);
}

PresetComboBox::PresetComboBox(wxWindow *parent, Preset::Type _preset_type) :
    PresetBitmapComboBox(parent, wxSize(15 * wxGetApp().em_unit(), -1)),
    preset_type(_preset_type),
    last_selected(wxNOT_FOUND),
    m_em_unit(wxGetApp().em_unit())
{
    SetFont(wxGetApp().normal_font());
#ifdef _WIN32
    // Workaround for ignoring CBN_EDITCHANGE events, which are processed after the content of the combo box changes, so that
    // the index of the item inside CBN_EDITCHANGE may no more be valid.
    EnableTextChangedEvents(false);
#endif /* _WIN32 */
    Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &evt) {
        auto selected_item = evt.GetSelection();
        
        assert(selected_item >= 0);
        auto marker = reinterpret_cast<Marker>(this->GetClientData(unsigned(selected_item)));
        if (marker >= LABEL_ITEM_MARKER && marker < LABEL_ITEM_MAX) {
            this->SetSelection(this->last_selected);
            evt.StopPropagation();
            if (marker >= LABEL_ITEM_WIZARD_PRINTERS) {
                ConfigWizard::StartPage sp = ConfigWizard::SP_WELCOME;
                switch (marker) {
                case LABEL_ITEM_WIZARD_PRINTERS: sp = ConfigWizard::SP_PRINTERS; break;
                case LABEL_ITEM_WIZARD_FILAMENTS: sp = ConfigWizard::SP_FILAMENTS; break;
                case LABEL_ITEM_WIZARD_MATERIALS: sp = ConfigWizard::SP_MATERIALS; break;
                }
                wxTheApp->CallAfter([sp]() { wxGetApp().run_wizard(ConfigWizard::RR_USER, sp); });
            }
        } else if ( this->last_selected != selected_item ||
                   wxGetApp().get_tab(this->preset_type)->get_presets()->current_is_dirty() ) {
            this->last_selected = selected_item;
            evt.SetInt(this->preset_type);
            evt.Skip();
        } else {
            evt.StopPropagation();
        }
    });
    
    if (preset_type == Slic3r::Preset::TYPE_FILAMENT)
    {
        Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent &event) {
            PresetBundle* preset_bundle = wxGetApp().preset_bundle;
            const Preset* selected_preset = preset_bundle->filaments.find_preset(preset_bundle->filament_presets[extruder_idx]);
            // Wide icons are shown if the currently selected preset is not compatible with the current printer,
            // and red flag is drown in front of the selected preset.
            bool          wide_icons = selected_preset != nullptr && !selected_preset->is_compatible;
            float scale = m_em_unit*0.1f;
            
            int shifl_Left = wide_icons ? int(scale * 16 + 0.5) : 0;
#if defined(wxBITMAPCOMBOBOX_OWNERDRAWN_BASED)
            shifl_Left  += int(scale * 4 + 0.5f); // IMAGE_SPACING_RIGHT = 4 for wxBitmapComboBox -> Space left of image
#endif
            int icon_right_pos = shifl_Left + int(scale * (24+4) + 0.5);
            int mouse_pos = event.GetLogicalPosition(wxClientDC(this)).x;
            if (mouse_pos < shifl_Left || mouse_pos > icon_right_pos ) {
                // Let the combo box process the mouse click.
                event.Skip();
                return;
            }
            
            // Swallow the mouse click and open the color picker.
            
            // get current color
            DynamicPrintConfig* cfg = wxGetApp().get_tab(Preset::TYPE_PRINTER)->get_config();
            auto colors = static_cast<ConfigOptionStrings*>(cfg->option("extruder_colour")->clone());
            wxColour clr(colors->values[extruder_idx]);
            if (!clr.IsOk())
                clr = wxColour(0,0,0); // Don't set alfa to transparence
            
            auto data = new wxColourData();
            data->SetChooseFull(1);
            data->SetColour(clr);
            
            wxColourDialog dialog(this, data);
            dialog.CenterOnParent();
            if (dialog.ShowModal() == wxID_OK)
            {
                colors->values[extruder_idx] = dialog.GetColourData().GetColour().GetAsString(wxC2S_HTML_SYNTAX).ToStdString();
                
                DynamicPrintConfig cfg_new = *cfg;
                cfg_new.set_key_value("extruder_colour", colors);
                
                wxGetApp().get_tab(Preset::TYPE_PRINTER)->load_config(cfg_new);
                preset_bundle->update_plater_filament_ui(extruder_idx, this);
                wxGetApp().plater()->on_config_change(cfg_new);
            }
        });
    }
    
    edit_btn = new ScalableButton(parent, wxID_ANY, "cog");
    edit_btn->SetToolTip(_(L("Click to edit preset")));

    edit_btn->Bind(
        wxEVT_BUTTON, ([this](wxCommandEvent) {
            Tab *tab = wxGetApp().get_tab(preset_type);
            if (!tab) return;

            int page_id = wxGetApp().tab_panel()->FindPage(tab);
            if (page_id == wxNOT_FOUND) return;
            
            assert(page_id >= 0);
            wxGetApp().tab_panel()->ChangeSelection(size_t(page_id));

            /* In a case of a multi-material printing, for editing another
             * Filament Preset it's needed to select this preset for the
             * "Filament settings" Tab
             */
            if (preset_type == Preset::TYPE_FILAMENT &&
                wxGetApp().extruders_edited_cnt() > 1) {
                const std::string &selected_preset =
                    GetString(unsigned(GetSelection())).ToUTF8().data();

                // Call select_preset() only if there is new preset and not
                // just modified
                if (!boost::algorithm::ends_with(selected_preset,
                                                 Preset::suffix_modified())) {
                    const std::string &preset_name =
                        wxGetApp()
                            .preset_bundle->filaments.get_preset_name_by_alias(
                                selected_preset);
                    tab->select_preset(/*selected_preset*/ preset_name);
                }
            }
        }));
}

PresetComboBox::~PresetComboBox()
{
    if (edit_btn)
        edit_btn->Destroy();
}


void PresetComboBox::set_label_marker(int item, LabelItemType label_item_type)
{
    assert(item >= 0);
    this->SetClientData(unsigned(item), reinterpret_cast<void*>(label_item_type));
}

void PresetComboBox::check_selection(int selection)
{
    this->last_selected = selection;
}

void PresetComboBox::msw_rescale()
{
    m_em_unit = wxGetApp().em_unit();
    edit_btn->msw_rescale();
}

// Sidebar / private

struct Sidebar::priv
{
    Plater *plater;
    
    wxScrolledWindow *scrolled;
    wxPanel* presets_panel; // Used for MSW better layouts
    
    ModeSizer  *mode_sizer;
    wxFlexGridSizer *sizer_presets;
    PresetComboBox *combo_print;
    std::vector<PresetComboBox*> combos_filament;
    wxBoxSizer *sizer_filaments;
    PresetComboBox *combo_sla_print;
    PresetComboBox *combo_sla_material;
    PresetComboBox *combo_printer;
    
    wxBoxSizer *sizer_params;
    FreqChangedParams   *frequently_changed_parameters{ nullptr };
    ObjectList          *object_list{ nullptr };
    ObjectManipulation  *object_manipulation{ nullptr };
    ObjectSettings      *object_settings{ nullptr };
    ObjectLayers        *object_layers{ nullptr };
    void                 set_btn_label(const ActionButtonType btn_type,
                                       const wxString &       label) const;
    
    ObjectInfo *object_info;
    SlicedInfo *sliced_info;
    
    wxButton *btn_export_gcode;
    wxButton *btn_reslice;
    ScalableButton *btn_send_gcode;
    ScalableButton *btn_remove_device;
    ScalableButton* btn_export_gcode_removable; //exports to removable drives (appears only if removable drive is connected)
    
    priv(Plater *plater) : plater(plater) {}
    ~priv();
    
    void show_preset_comboboxes();
};

Sidebar::priv::~priv()
{
    if (object_manipulation != nullptr)
        delete object_manipulation;
    
    if (object_settings != nullptr)
        delete object_settings;
    
    if (frequently_changed_parameters != nullptr)
        delete frequently_changed_parameters;
    
    if (object_layers != nullptr)
        delete object_layers;
}

void Sidebar::priv::show_preset_comboboxes()
{
    const bool showSLA = wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology() == ptSLA;
    
    for (size_t i = 0; i < 4; ++i)
        sizer_presets->Show(i, !showSLA);
    
    for (size_t i = 4; i < 8; ++i) {
        if (sizer_presets->IsShown(i) != showSLA)
            sizer_presets->Show(i, showSLA);
    }
    
    frequently_changed_parameters->Show(!showSLA);
    
    scrolled->GetParent()->Layout();
    scrolled->Refresh();
}


// Sidebar / public

wxDEFINE_EVENT(EVT_SCHEDULE_BACKGROUND_PROCESS, SimpleEvent);

Sidebar::Sidebar(Plater *parent)
    : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(40 * wxGetApp().em_unit(), -1)), p(new priv(parent))
{
    p->scrolled = new wxScrolledWindow(this);
    p->scrolled->SetScrollbars(0, 100, 1, 2);
    
    
    // Sizer in the scrolled area
    auto *scrolled_sizer = new wxBoxSizer(wxVERTICAL);
    p->scrolled->SetSizer(scrolled_sizer);
    
    // Sizer with buttons for mode changing
    p->mode_sizer = new ModeSizer(p->scrolled);
    
    // The preset chooser
    p->sizer_presets = new wxFlexGridSizer(10, 1, 1, 2);
    p->sizer_presets->AddGrowableCol(0, 1);
    p->sizer_presets->SetFlexibleDirection(wxBOTH);
    
    bool is_msw = false;
#ifdef __WINDOWS__
    p->scrolled->SetDoubleBuffered(true);
    
    p->presets_panel = new wxPanel(p->scrolled, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    p->presets_panel->SetSizer(p->sizer_presets);
    
    is_msw = true;
#else
    p->presets_panel = p->scrolled;
#endif //__WINDOWS__
    
    p->sizer_filaments = new wxBoxSizer(wxVERTICAL);
    
    auto init_combo = [this](PresetComboBox **combo, wxString label, Preset::Type preset_type, bool filament) {
        auto *text = new wxStaticText(p->presets_panel, wxID_ANY, label + " :");
        text->SetFont(wxGetApp().small_font());
        *combo = new PresetComboBox(p->presets_panel, preset_type);
        
        auto combo_and_btn_sizer = new wxBoxSizer(wxHORIZONTAL);
        combo_and_btn_sizer->Add(*combo, 1, wxEXPAND);
        if ((*combo)->edit_btn)
            combo_and_btn_sizer->Add((*combo)->edit_btn, 0, wxALIGN_CENTER_VERTICAL|wxLEFT|wxRIGHT,
                                     int(0.3*wxGetApp().em_unit()));
        
        auto *sizer_presets = this->p->sizer_presets;
        auto *sizer_filaments = this->p->sizer_filaments;
        sizer_presets->Add(text, 0, wxALIGN_LEFT | wxEXPAND | wxRIGHT, 4);
        if (! filament) {
            sizer_presets->Add(combo_and_btn_sizer, 0, wxEXPAND | wxBOTTOM, 1);
        } else {
            sizer_filaments->Add(combo_and_btn_sizer, 0, wxEXPAND | wxBOTTOM, 1);
            (*combo)->set_extruder_idx(0);
            sizer_presets->Add(sizer_filaments, 1, wxEXPAND);
        }
    };
    
    p->combos_filament.push_back(nullptr);
    init_combo(&p->combo_print,         _(L("Print settings")),     Preset::TYPE_PRINT,         false);
    init_combo(&p->combos_filament[0],  _(L("Filament")),           Preset::TYPE_FILAMENT,      true);
    init_combo(&p->combo_sla_print,     _(L("SLA print settings")), Preset::TYPE_SLA_PRINT,     false);
    init_combo(&p->combo_sla_material,  _(L("SLA material")),       Preset::TYPE_SLA_MATERIAL,  false);
    init_combo(&p->combo_printer,       _(L("Printer")),            Preset::TYPE_PRINTER,       false);
    
    const int margin_5  = int(0.5*wxGetApp().em_unit());// 5;
    
    p->sizer_params = new wxBoxSizer(wxVERTICAL);
    
    // Frequently changed parameters
    p->frequently_changed_parameters = new FreqChangedParams(p->scrolled);
    p->sizer_params->Add(p->frequently_changed_parameters->get_sizer(), 0, wxEXPAND | wxTOP | wxBOTTOM, wxOSX ? 1 : margin_5);
    
    // Object List
    p->object_list = new ObjectList(p->scrolled);
    p->sizer_params->Add(p->object_list->get_sizer(), 1, wxEXPAND);
    
    // Object Manipulations
    p->object_manipulation = new ObjectManipulation(p->scrolled);
    p->object_manipulation->Hide();
    p->sizer_params->Add(p->object_manipulation->get_sizer(), 0, wxEXPAND | wxTOP, margin_5);
    
    // Frequently Object Settings
    p->object_settings = new ObjectSettings(p->scrolled);
    p->object_settings->Hide();
    p->sizer_params->Add(p->object_settings->get_sizer(), 0, wxEXPAND | wxTOP, margin_5);
    
    // Object Layers
    p->object_layers = new ObjectLayers(p->scrolled);
    p->object_layers->Hide();
    p->sizer_params->Add(p->object_layers->get_sizer(), 0, wxEXPAND | wxTOP, margin_5);
    
    // Info boxes
    p->object_info = new ObjectInfo(p->scrolled);
    p->sliced_info = new SlicedInfo(p->scrolled);
    
    // Sizer in the scrolled area
    scrolled_sizer->Add(p->mode_sizer, 0, wxALIGN_CENTER_HORIZONTAL/*RIGHT | wxBOTTOM | wxRIGHT, 5*/);
    is_msw ?
        scrolled_sizer->Add(p->presets_panel, 0, wxEXPAND | wxLEFT, margin_5) :
        scrolled_sizer->Add(p->sizer_presets, 0, wxEXPAND | wxLEFT, margin_5);
    scrolled_sizer->Add(p->sizer_params, 1, wxEXPAND | wxLEFT, margin_5);
    scrolled_sizer->Add(p->object_info, 0, wxEXPAND | wxTOP | wxLEFT, margin_5);
    scrolled_sizer->Add(p->sliced_info, 0, wxEXPAND | wxTOP | wxLEFT, margin_5);
    
    // Buttons underneath the scrolled area
    
    // rescalable bitmap buttons "Send to printer" and "Remove device" 

    auto init_scalable_btn = [this](ScalableButton** btn, const std::string& icon_name, wxString tooltip = wxEmptyString)
    {
#ifdef __APPLE__
        int bmp_px_cnt = 16;
#else
        int bmp_px_cnt = 32;
#endif //__APPLE__
        ScalableBitmap bmp = ScalableBitmap(this, icon_name, bmp_px_cnt);
        *btn = new ScalableButton(this, wxID_ANY, bmp, "", wxBU_EXACTFIT);
        (*btn)->SetToolTip(tooltip);
        (*btn)->Hide();
    };
    
    init_scalable_btn(&p->btn_send_gcode   , "export_gcode", _(L("Send to printer")) + "\tCtrl+Shift+G");
    init_scalable_btn(&p->btn_remove_device, "eject_sd"       , _(L("Remove device")) + "\tCtrl+T");
    init_scalable_btn(&p->btn_export_gcode_removable, "export_to_sd", _(L("Export to SD card / Flash drive")) + "\tCtrl+U");
    
    // regular buttons "Slice now" and "Export G-code" 

    const int scaled_height = p->btn_remove_device->GetBitmapHeight() + 4;
    auto init_btn = [this](wxButton **btn, wxString label, const int button_height) {
        *btn = new wxButton(this, wxID_ANY, label, wxDefaultPosition,
                            wxSize(-1, button_height), wxBU_EXACTFIT);
        (*btn)->SetFont(wxGetApp().bold_font());
    };
    
    init_btn(&p->btn_export_gcode, _(L("Export G-code")) + dots , scaled_height);
    init_btn(&p->btn_reslice     , _(L("Slice now"))            , scaled_height);
    
    enable_buttons(false);
    
    auto *btns_sizer = new wxBoxSizer(wxVERTICAL);
    
    auto* complect_btns_sizer = new wxBoxSizer(wxHORIZONTAL);
    complect_btns_sizer->Add(p->btn_export_gcode, 1, wxEXPAND);
    complect_btns_sizer->Add(p->btn_send_gcode);
    complect_btns_sizer->Add(p->btn_export_gcode_removable);
    complect_btns_sizer->Add(p->btn_remove_device);
    
    
    btns_sizer->Add(p->btn_reslice, 0, wxEXPAND | wxTOP, margin_5);
    btns_sizer->Add(complect_btns_sizer, 0, wxEXPAND | wxTOP, margin_5);
    
    auto *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(p->scrolled, 1, wxEXPAND);
    sizer->Add(btns_sizer, 0, wxEXPAND | wxLEFT, margin_5);
    SetSizer(sizer);
    
    // Events
    p->btn_export_gcode->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { p->plater->export_gcode(false); });
    p->btn_reslice->Bind(wxEVT_BUTTON, [this](wxCommandEvent&)
                         {
                             const bool export_gcode_after_slicing = wxGetKeyState(WXK_SHIFT);
                             if (export_gcode_after_slicing)
                                 p->plater->export_gcode();
                             else
                                 p->plater->reslice();
                             p->plater->select_view_3D("Preview");
                         });
    p->btn_send_gcode->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { p->plater->send_gcode(); });
    p->btn_remove_device->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { p->plater->eject_drive(); });
    p->btn_export_gcode_removable->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { p->plater->export_gcode(true); });
}

Sidebar::~Sidebar() {}

void Sidebar::init_filament_combo(PresetComboBox **combo, const int extr_idx) {
    *combo = new PresetComboBox(p->presets_panel, Slic3r::Preset::TYPE_FILAMENT);
    //         # copy icons from first choice
    //         $choice->SetItemBitmap($_, $choices->[0]->GetItemBitmap($_)) for 0..$#presets;
    
    (*combo)->set_extruder_idx(extr_idx);
    
    auto combo_and_btn_sizer = new wxBoxSizer(wxHORIZONTAL);
    combo_and_btn_sizer->Add(*combo, 1, wxEXPAND);
    combo_and_btn_sizer->Add((*combo)->edit_btn, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT,
                             int(0.3*wxGetApp().em_unit()));
    
    auto /***/sizer_filaments = this->p->sizer_filaments;
    sizer_filaments->Add(combo_and_btn_sizer, 1, wxEXPAND | wxBOTTOM, 1);
}

void Sidebar::remove_unused_filament_combos(const size_t current_extruder_count)
{
    if (current_extruder_count >= p->combos_filament.size())
        return;
    auto sizer_filaments = this->p->sizer_filaments;
    while (p->combos_filament.size() > current_extruder_count) {
        const int last = p->combos_filament.size() - 1;
        sizer_filaments->Remove(last);
        (*p->combos_filament[last]).Destroy();
        p->combos_filament.pop_back();
    }
}

void Sidebar::update_all_preset_comboboxes()
{
    PresetBundle &preset_bundle = *wxGetApp().preset_bundle;
    const auto print_tech = preset_bundle.printers.get_edited_preset().printer_technology();
    
    // Update the print choosers to only contain the compatible presets, update the dirty flags.
    if (print_tech == ptFFF)
        preset_bundle.prints.update_plater_ui(p->combo_print);
    else {
        preset_bundle.sla_prints.update_plater_ui(p->combo_sla_print);
        preset_bundle.sla_materials.update_plater_ui(p->combo_sla_material);
    }
    // Update the printer choosers, update the dirty flags.
    preset_bundle.printers.update_plater_ui(p->combo_printer);
    // Update the filament choosers to only contain the compatible presets, update the color preview,
    // update the dirty flags.
    if (print_tech == ptFFF) {
        for (size_t i = 0; i < p->combos_filament.size(); ++i)
            preset_bundle.update_plater_filament_ui(i, p->combos_filament[i]);
    }
}

void Sidebar::update_presets(Preset::Type preset_type)
{
    PresetBundle &preset_bundle = *wxGetApp().preset_bundle;
    const auto print_tech = preset_bundle.printers.get_edited_preset().printer_technology();
    
    switch (preset_type) {
    case Preset::TYPE_FILAMENT:
    {
        const size_t extruder_cnt = print_tech != ptFFF ? 1 :
                                                          dynamic_cast<ConfigOptionFloats*>(preset_bundle.printers.get_edited_preset().config.option("nozzle_diameter"))->values.size();
        const size_t filament_cnt = p->combos_filament.size() > extruder_cnt ? extruder_cnt : p->combos_filament.size();
        
        if (filament_cnt == 1) {
            // Single filament printer, synchronize the filament presets.
            const std::string &name = preset_bundle.filaments.get_selected_preset_name();
            preset_bundle.set_filament_preset(0, name);
        }
        
        for (size_t i = 0; i < filament_cnt; i++) {
            preset_bundle.update_plater_filament_ui(i, p->combos_filament[i]);
        }
        
        break;
    }
        
    case Preset::TYPE_PRINT:
        preset_bundle.prints.update_plater_ui(p->combo_print);
        break;
        
    case Preset::TYPE_SLA_PRINT:
        preset_bundle.sla_prints.update_plater_ui(p->combo_sla_print);
        break;
        
    case Preset::TYPE_SLA_MATERIAL:
        preset_bundle.sla_materials.update_plater_ui(p->combo_sla_material);
        break;
        
    case Preset::TYPE_PRINTER:
    {
        update_all_preset_comboboxes();
        p->show_preset_comboboxes();
        break;
    }
        
    default: break;
    }
    
    // Synchronize config.ini with the current selections.
    wxGetApp().preset_bundle->export_selections(*wxGetApp().app_config);
}

void Sidebar::update_mode_sizer() const
{
    p->mode_sizer->SetMode(m_mode);
}

void Sidebar::update_reslice_btn_tooltip() const
{
    wxString tooltip = wxString("Slice") + " [" + GUI::shortkey_ctrl_prefix() + "R]";
    if (m_mode != comSimple)
        tooltip += wxString("\n") + _(L("Hold Shift to Slice & Export G-code"));
    p->btn_reslice->SetToolTip(tooltip);
}

void Sidebar::msw_rescale()
{
    SetMinSize(wxSize(40 * wxGetApp().em_unit(), -1));
    
    p->mode_sizer->msw_rescale();
    
    // Rescale preset comboboxes in respect to the current  em_unit ...
    for (PresetComboBox* combo : std::vector<PresetComboBox*> { p->combo_print,
                                                               p->combo_sla_print,
                                                               p->combo_sla_material,
                                                               p->combo_printer } )
        combo->msw_rescale();
    for (PresetComboBox* combo : p->combos_filament)
        combo->msw_rescale();
    
    // ... then refill them and set min size to correct layout of the sidebar
    update_all_preset_comboboxes();
    
    p->frequently_changed_parameters->msw_rescale();
    p->object_list->msw_rescale();
    p->object_manipulation->msw_rescale();
    p->object_settings->msw_rescale();
    p->object_layers->msw_rescale();
    
    p->object_info->msw_rescale();
    
    p->btn_send_gcode->msw_rescale();
    p->btn_remove_device->msw_rescale();
    p->btn_export_gcode_removable->msw_rescale();
    const int scaled_height = p->btn_remove_device->GetBitmap().GetHeight() + 4;
    p->btn_export_gcode->SetMinSize(wxSize(-1, scaled_height));
    p->btn_reslice     ->SetMinSize(wxSize(-1, scaled_height));
    
    p->scrolled->Layout();
}

ObjectManipulation* Sidebar::obj_manipul()
{
    return p->object_manipulation;
}

ObjectList* Sidebar::obj_list()
{
    return p->object_list;
}

ObjectSettings* Sidebar::obj_settings()
{
    return p->object_settings;
}

ObjectLayers* Sidebar::obj_layers()
{
    return p->object_layers;
}

wxScrolledWindow* Sidebar::scrolled_panel()
{
    return p->scrolled;
}

wxPanel* Sidebar::presets_panel()
{
    return p->presets_panel;
}

ConfigOptionsGroup* Sidebar::og_freq_chng_params(const bool is_fff)
{
    return p->frequently_changed_parameters->get_og(is_fff);
}

wxButton* Sidebar::get_wiping_dialog_button()
{
    return p->frequently_changed_parameters->get_wiping_dialog_button();
}

void Sidebar::update_objects_list_extruder_column(size_t extruders_count)
{
    p->object_list->update_objects_list_extruder_column(extruders_count);
}

void Sidebar::show_info_sizer()
{
    if (!p->plater->is_single_full_object_selection() ||
        m_mode < comExpert ||
        p->plater->model().objects.empty()) {
        p->object_info->Show(false);
        return;
    }
    
    int obj_idx = p->plater->get_selected_object_idx();
    
    const ModelObject* model_object = p->plater->model().objects[obj_idx];
    // hack to avoid crash when deleting the last object on the bed
    if (model_object->volumes.empty())
    {
        p->object_info->Show(false);
        return;
    }
    
    auto size = model_object->bounding_box().size();
    p->object_info->info_size->SetLabel(wxString::Format("%.2f x %.2f x %.2f",size(0), size(1), size(2)));
    p->object_info->info_materials->SetLabel(wxString::Format("%d", static_cast<int>(model_object->materials_count())));
    
    const auto& stats = model_object->get_object_stl_stats();//model_object->volumes.front()->mesh.stl.stats;
    p->object_info->info_volume->SetLabel(wxString::Format("%.2f", stats.volume));
    p->object_info->info_facets->SetLabel(wxString::Format(_(L("%d (%d shells)")), static_cast<int>(model_object->facets_count()), stats.number_of_parts));
    
    int errors = stats.degenerate_facets + stats.edges_fixed + stats.facets_removed +
                 stats.facets_added + stats.facets_reversed + stats.backwards_edges;
    if (errors > 0) {
        wxString tooltip = wxString::Format(_(L("Auto-repaired (%d errors)")), errors);
        p->object_info->info_manifold->SetLabel(tooltip);
        
        tooltip += ":\n" + wxString::Format(_(L("%d degenerate facets, %d edges fixed, %d facets removed, "
                                                "%d facets added, %d facets reversed, %d backwards edges")),
                                            stats.degenerate_facets, stats.edges_fixed, stats.facets_removed,
                                            stats.facets_added, stats.facets_reversed, stats.backwards_edges);
        
        p->object_info->showing_manifold_warning_icon = true;
        p->object_info->info_manifold->SetToolTip(tooltip);
        p->object_info->manifold_warning_icon->SetToolTip(tooltip);
    }
    else {
        p->object_info->info_manifold->SetLabel(_(L("Yes")));
        p->object_info->showing_manifold_warning_icon = false;
        p->object_info->info_manifold->SetToolTip("");
        p->object_info->manifold_warning_icon->SetToolTip("");
    }
    
    p->object_info->show_sizer(true);
    
    if (p->plater->printer_technology() == ptSLA) {
        for (auto item: p->object_info->sla_hidden_items)
            item->Show(false);
    }
}

void Sidebar::update_sliced_info_sizer()
{
    if (p->sliced_info->IsShown(size_t(0)))
    {
        if (p->plater->printer_technology() == ptSLA)
        {
            const SLAPrintStatistics& ps = p->plater->sla_print().print_statistics();
            wxString new_label = _(L("Used Material (ml)")) + ":";
            const bool is_supports = ps.support_used_material > 0.0;
            if (is_supports)
                new_label += from_u8((boost::format("\n    - %s\n    - %s") % _utf8(L("object(s)")) % _utf8(L("supports and pad"))).str());
            
            wxString info_text = is_supports ?
                                     wxString::Format("%.2f \n%.2f \n%.2f", (ps.objects_used_material + ps.support_used_material) / 1000,
                                                      ps.objects_used_material / 1000,
                                                      ps.support_used_material / 1000) :
                                     wxString::Format("%.2f", (ps.objects_used_material + ps.support_used_material) / 1000);
            p->sliced_info->SetTextAndShow(siMateril_unit, info_text, new_label);
            
            wxString str_total_cost = "N/A";
            
            DynamicPrintConfig* cfg = wxGetApp().get_tab(Preset::TYPE_SLA_MATERIAL)->get_config();
            if (cfg->option("bottle_cost")->getFloat() > 0.0 &&
                cfg->option("bottle_volume")->getFloat() > 0.0)
            {
                double material_cost = cfg->option("bottle_cost")->getFloat() / 
                                       cfg->option("bottle_volume")->getFloat();
                str_total_cost = wxString::Format("%.3f", material_cost*(ps.objects_used_material + ps.support_used_material) / 1000);                
            }
            p->sliced_info->SetTextAndShow(siCost, str_total_cost, "Cost");
            
            wxString t_est = std::isnan(ps.estimated_print_time) ? "N/A" : get_time_dhms(float(ps.estimated_print_time));
            p->sliced_info->SetTextAndShow(siEstimatedTime, t_est, _(L("Estimated printing time")) + ":");
            
            // Hide non-SLA sliced info parameters
            p->sliced_info->SetTextAndShow(siFilament_m, "N/A");
            p->sliced_info->SetTextAndShow(siFilament_mm3, "N/A");
            p->sliced_info->SetTextAndShow(siFilament_g, "N/A");
            p->sliced_info->SetTextAndShow(siWTNumbetOfToolchanges, "N/A");
        }
        else
        {
            const PrintStatistics& ps = p->plater->fff_print().print_statistics();
            const bool is_wipe_tower = ps.total_wipe_tower_filament > 0;
            
            wxString new_label = _(L("Used Filament (m)"));
            if (is_wipe_tower)
                new_label += from_u8((boost::format(":\n    - %1%\n    - %2%") % _utf8(L("objects")) % _utf8(L("wipe tower"))).str());
            
            wxString info_text = is_wipe_tower ?
                                     wxString::Format("%.2f \n%.2f \n%.2f", ps.total_used_filament / 1000,
                                                      (ps.total_used_filament - ps.total_wipe_tower_filament) / 1000,
                                                      ps.total_wipe_tower_filament / 1000) :
                                     wxString::Format("%.2f", ps.total_used_filament / 1000);
            p->sliced_info->SetTextAndShow(siFilament_m,    info_text,      new_label);
            
            p->sliced_info->SetTextAndShow(siFilament_mm3,  wxString::Format("%.2f", ps.total_extruded_volume));
            p->sliced_info->SetTextAndShow(siFilament_g,    ps.total_weight == 0.0 ? "N/A" : wxString::Format("%.2f", ps.total_weight));
            
            new_label = _(L("Cost"));
            if (is_wipe_tower)
                new_label += from_u8((boost::format(":\n    - %1%\n    - %2%") % _utf8(L("objects")) % _utf8(L("wipe tower"))).str());
            
            info_text = ps.total_cost == 0.0 ? "N/A" :
                                               is_wipe_tower ?
                                               wxString::Format("%.2f \n%.2f \n%.2f", ps.total_cost,
                                                                (ps.total_cost - ps.total_wipe_tower_cost),
                                                                ps.total_wipe_tower_cost) :
                                               wxString::Format("%.2f", ps.total_cost);
            p->sliced_info->SetTextAndShow(siCost, info_text,      new_label);
            
            if (ps.estimated_normal_print_time == "N/A" && ps.estimated_silent_print_time == "N/A")
                p->sliced_info->SetTextAndShow(siEstimatedTime, "N/A");
            else {
                new_label = _(L("Estimated printing time")) +":";
                info_text = "";
                wxString str_color = _(L("Color"));
                wxString str_pause = _(L("Pause"));
                
                auto fill_labels = [str_color, str_pause](const std::vector<std::pair<CustomGcodeType, std::string>>& times, 
                                                          wxString& new_label, wxString& info_text)
                {
                    int color_change_count = 0;
                    for (auto time : times)
                        if (time.first == cgtColorChange)
                            color_change_count++;
                    
                    for (int i = (int)times.size() - 1; i >= 0; --i)
                    {
                        if (i == 0 || times[i - 1].first == cgtPausePrint)
                            new_label += from_u8((boost::format("\n      - %1%%2%") % (std::string(str_color.ToUTF8()) + " ") % color_change_count).str());
                        else if (times[i - 1].first == cgtColorChange)
                            new_label += from_u8((boost::format("\n      - %1%%2%") % (std::string(str_color.ToUTF8()) + " ") % color_change_count--).str());
                        
                        if (i != (int)times.size() - 1 && times[i].first == cgtPausePrint)
                            new_label += from_u8((boost::format(" -> %1%") % std::string(str_pause.ToUTF8())).str());
                        
                        info_text += from_u8((boost::format("\n%1%") % times[i].second).str());
                    }
                };
                
                if (ps.estimated_normal_print_time != "N/A") {
                    new_label += from_u8((boost::format("\n   - %1%") % _utf8(L("normal mode"))).str());
                    info_text += from_u8((boost::format("\n%1%") % ps.estimated_normal_print_time).str());
                    fill_labels(ps.estimated_normal_custom_gcode_print_times, new_label, info_text);
                }
                if (ps.estimated_silent_print_time != "N/A") {
                    new_label += from_u8((boost::format("\n   - %1%") % _utf8(L("stealth mode"))).str());
                    info_text += from_u8((boost::format("\n%1%") % ps.estimated_silent_print_time).str());
                    fill_labels(ps.estimated_silent_custom_gcode_print_times, new_label, info_text);
                }
                p->sliced_info->SetTextAndShow(siEstimatedTime,  info_text,      new_label);
            }
            
            // if there is a wipe tower, insert number of toolchanges info into the array:
            p->sliced_info->SetTextAndShow(siWTNumbetOfToolchanges, is_wipe_tower ? wxString::Format("%.d", ps.total_toolchanges) : "N/A");
            
            // Hide non-FFF sliced info parameters
            p->sliced_info->SetTextAndShow(siMateril_unit, "N/A");
        }
    }
}

void Sidebar::show_sliced_info_sizer(const bool show)
{
    wxWindowUpdateLocker freeze_guard(this);
    
    p->sliced_info->Show(show);
    if (show)
        update_sliced_info_sizer();
    
    Layout();
    p->scrolled->Refresh();
}

void Sidebar::enable_buttons(bool enable)
{
    p->btn_reslice->Enable(enable);
    p->btn_export_gcode->Enable(enable);
    p->btn_send_gcode->Enable(enable);
    p->btn_remove_device->Enable(enable);
    p->btn_export_gcode_removable->Enable(enable);
}

bool Sidebar::show_reslice(bool show)         const { return p->btn_reslice->Show(show); }
bool Sidebar::show_export(bool show)          const { return p->btn_export_gcode->Show(show); }
bool Sidebar::show_send(bool show)            const { return p->btn_send_gcode->Show(show); }
bool Sidebar::show_disconnect(bool show)      const { return p->btn_remove_device->Show(show); }
bool Sidebar::show_export_removable(bool show)const { return p->btn_export_gcode_removable->Show(show); }

bool Sidebar::is_multifilament()
{
    return p->combos_filament.size() > 1;
}


void Sidebar::update_mode()
{
    m_mode = wxGetApp().get_mode();
    
    update_reslice_btn_tooltip();
    update_mode_sizer();
    
    wxWindowUpdateLocker noUpdates(this);
    
    p->object_list->get_sizer()->Show(m_mode > comSimple);
    
    p->object_list->unselect_objects();
    p->object_list->update_selections();
    p->object_list->update_object_menu();
    
    Layout();
}

std::vector<PresetComboBox*>& Sidebar::combos_filament()
{
    return p->combos_filament;
}

void Sidebar::set_btn_label(const ActionButtonType btn_type, const wxString& label) const
{
    switch (btn_type)
    {
    case ActionButtonType::abReslice:   p->btn_reslice->SetLabelText(label);        break;
    case ActionButtonType::abExport:    p->btn_export_gcode->SetLabelText(label);   break;
    case ActionButtonType::abSendGCode: /*p->btn_send_gcode->SetLabelText(label);*/     break;
    }
}

}} // namespace Slic3r::GUI
