#ifndef slic3r_GUI_ObjectTexture_hpp_
#define slic3r_GUI_ObjectTexture_hpp_

#if ENABLE_TEXTURED_VOLUMES

#include "GUI_ObjectSettings.hpp"

namespace Slic3r {

class ModelObject;

namespace GUI {

class ObjectTexture : public OG_Settings
{
    ScalableBitmap m_bmp_delete;
    ScalableBitmap m_bmp_delete_focus;
    ScalableBitmap m_bmp_delete_disabled;

    wxBoxSizer* m_main_sizer{ nullptr };
    wxBoxSizer* m_tex_sizer{ nullptr };
    wxTextCtrl* m_tex_string{ nullptr };
    ScalableButton* m_tex_delete_btn{ nullptr };

public:
    ObjectTexture(wxWindow* parent);

    void msw_rescale();
    void sys_color_changed();

    void UpdateAndShow(const bool show) override;

private:
    void update();
    std::pair<int, ModelObject*> get_model_object();
};

} // namespace GUI
} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES

#endif // slic3r_GUI_ObjectTexture_hpp_