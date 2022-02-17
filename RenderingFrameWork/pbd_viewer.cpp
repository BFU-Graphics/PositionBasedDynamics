/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "pbd_viewer.h"

#include <utility>
#include "src/mouse_callback.h"

namespace HINAVIEWER
{
    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> g_geometry;
    std::vector<unsigned int> g_id;
}

HINAVIEWER::PBDViewer::PBDViewer(int width, int height, Eigen::Vector4f color) : width_(width), height_(height), color_(std::move(color))
{
    viewer_.core().background_color = color_;
    viewer_.core().is_animating = true;
    ffmpeg = nullptr;
}

HINAVIEWER::PBDViewer::~PBDViewer()
{
    fclose(ffmpeg);
}

void HINAVIEWER::PBDViewer::launch_rendering(const std::string &window_name)
{
    if (enable_menu) setup_menu();
    if (enable_custom_mouse_callback) setup_callback();
    if (enable_inspector) setup_inspector();
    viewer_.launch_init(true, is_full_screen, window_name, width_, height_);
    viewer_.launch_rendering(true);
}

void HINAVIEWER::PBDViewer::update_vertex_positions(unsigned int id, Eigen::Ref<const Eigen::MatrixXd> V)
{
    viewer_.data_list[id].V = V;
    viewer_.data_list[id].compute_normals();
    viewer_.data_list[id].dirty |= igl::opengl::MeshGL::DIRTY_POSITION;

    if (focus_object_ID > -1 && focus_object_ID < viewer_.data_list.size())
        viewer_.core().align_camera_center(viewer_.data_list[focus_object_ID].V);
}

igl::opengl::glfw::Viewer &HINAVIEWER::PBDViewer::viewer()
{
    return viewer_;
}

int HINAVIEWER::PBDViewer::record(HINAVIEWER::RENDERABLE::Renderable *renderable)
{
    int ID;
    if (object_list_.empty())
        ID = 0;
    else
        ID = viewer_.append_mesh();

    viewer_.data(ID).set_mesh(renderable->V_, renderable->F_);
    viewer_.data(ID).set_colors(Eigen::RowVector3d(244, 165, 130) / 255.);

    renderable->is_recorded_ = true;
    renderable->ID_ = ID;

    return ID;
}

void HINAVIEWER::PBDViewer::track(HINAVIEWER::INSPECTOR::Trackable *trackable, int index)
{
    inspector_list_.emplace_back((new HINAVIEWER::INSPECTOR::ScalarTimeValueInspector())->track(trackable, index));
}

HINAVIEWER::PBDViewer &HINAVIEWER::PBDViewer::set_background_color(const Eigen::Vector4f &color)
{
    viewer_.core().background_color = color;
    return *this;
}

HINAVIEWER::PBDViewer &HINAVIEWER::PBDViewer::set_max_fps(double fps)
{
    viewer_.core().animation_max_fps = fps;
    return *this;
}

HINAVIEWER::PBDViewer &HINAVIEWER::PBDViewer::set_full_screen()
{
    is_full_screen = true;
    return *this;
}

HINAVIEWER::PBDViewer &HINAVIEWER::PBDViewer::focus_object(int ID)
{
    focus_object_ID = ID;
    return *this;
}

HINAVIEWER::PBDViewer &HINAVIEWER::PBDViewer::show_inspector()
{
    enable_inspector = true;
    return *this;
}


void HINAVIEWER::PBDViewer::setup_menu()
{
    viewer_.plugins.push_back(&menu_);

    menu_.callback_draw_viewer_menu = [&]()
    {
        ImGuiStyle &style = ImGui::GetStyle();
        style.WindowRounding = 5.3f;
        style.FrameRounding = 2.3f;
        style.ScrollbarRounding = 0;

        style.Colors[ImGuiCol_Text] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
        style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
        style.Colors[ImGuiCol_ChildBg] = ImVec4(0.72f, 0.72f, 0.72f, 1.00f);
        style.Colors[ImGuiCol_PopupBg] = ImVec4(0.05f, 0.05f, 0.10f, 0.85f);
        style.Colors[ImGuiCol_Border] = ImVec4(0.70f, 0.70f, 0.70f, 0.65f);
        style.Colors[ImGuiCol_BorderShadow] = ImVec4(1.00f, 0.00f, 0.00f, 0.00f);
        style.Colors[ImGuiCol_FrameBg] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
        style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.90f, 0.80f, 0.80f, 0.40f);
        style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.90f, 0.65f, 0.65f, 0.45f);
        style.Colors[ImGuiCol_TitleBg] = ImVec4(0.960f, 0.960f, 0.960f, 1.0f);
        style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.960f, 0.960f, 0.960f, 1.0f);
        style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.960f, 0.960f, 0.960f, 1.0f);
        style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.01f, 0.01f, 0.02f, 0.80f);
        style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.20f, 0.25f, 0.30f, 0.60f);
        style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.55f, 0.53f, 0.55f, 0.51f);
        style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.56f, 1.00f);
        style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.56f, 0.56f, 0.56f, 0.91f);
        style.Colors[ImGuiCol_CheckMark] = ImVec4(0.90f, 0.90f, 0.90f, 0.83f);
        style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.70f, 0.70f, 0.70f, 0.62f);
        style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.30f, 0.30f, 0.30f, 0.84f);
        style.Colors[ImGuiCol_Button] = ImVec4(0.48f, 0.72f, 0.89f, 1.00f);
        style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.50f, 0.69f, 0.99f, 1.00f);
        style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.80f, 0.50f, 0.50f, 1.00f);
        style.Colors[ImGuiCol_Header] = ImVec4(0.44f, 0.61f, 0.86f, 1.00f);
        style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.44f, 0.61f, 0.86f, 1.00f);
        style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.44f, 0.61f, 0.86f, 1.00f);
        style.Colors[ImGuiCol_Separator] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
        style.Colors[ImGuiCol_SeparatorHovered] = ImVec4(0.70f, 0.60f, 0.60f, 1.00f);
        style.Colors[ImGuiCol_SeparatorActive] = ImVec4(0.90f, 0.70f, 0.70f, 1.00f);
        style.Colors[ImGuiCol_ResizeGrip] = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);
        style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);
        style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);
        style.Colors[ImGuiCol_PlotLines] = ImVec4(0.00f, 1.00f, 0.00f, 1.00f);
        style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
        style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
        style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
        style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.00f, 0.00f, 1.00f, 0.35f);
        style.Colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.20f, 0.20f, 0.20f, 0.35f);

        if (ImGui::CollapsingHeader("Rendering Runtime Info", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;

            ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        }

        if (ImGui::CollapsingHeader("Physics Runtime Info", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;

            std::string simulation_runtime = std::to_string(HINAVIEWER::INSPECTOR::Timeable::simulation_time_);
            simulation_runtime.resize(5);
            simulation_runtime = "Total: " + simulation_runtime + "s";
            ImGui::Text("%s", simulation_runtime.c_str());

            ImGui::SameLine(0, p);

            std::string physics_runtime = std::to_string(HINAVIEWER::INSPECTOR::Timeable::physics_runtime);
            physics_runtime = "Current: " + physics_runtime + "ms";
            ImGui::Text("%s", physics_runtime.c_str());
        }

        // Draw parent menu content
        menu_.draw_viewer_menu();
    };
}

void HINAVIEWER::PBDViewer::setup_callback()
{
    viewer_.callback_mouse_down = MOUSE_CALLBACK::mouse_down;
    viewer_.callback_mouse_up = MOUSE_CALLBACK::mouse_up;
    viewer_.callback_mouse_move = MOUSE_CALLBACK::mouse_move;
}

void HINAVIEWER::PBDViewer::setup_inspector()
{
    menu_.callback_draw_custom_window = [&]()
    {
        ImVec2 min = ImGui::GetWindowContentRegionMin();
        ImVec2 max = ImGui::GetWindowContentRegionMax();

        max.x = (max.x - min.x) / 2;
        max.y -= min.y + ImGui::GetTextLineHeightWithSpacing() * 3;

        for (int i = 0; i < inspector_list_.size(); ++i)
        {
            if (inspector_list_[i]->trackable_->tracked_state_.empty())
                continue; // fix the case rendering loop is faster than simulation loop
            inspector_list_[i]->plot(1280 - 600 - 1, i * 150, 600, 150);
        }
    };
}

HINAVIEWER::PBDViewer &HINAVIEWER::PBDViewer::save_to_mp4(const std::string &ffmpeg_path, const std::string &filename)
{
    std::string cmd_temp = ffmpeg_path + " -r 60 -f rawvideo -pix_fmt rgba -s 1280x800 -i - "
                                         "-threads 0 -preset fast -y -pix_fmt yuv420p -crf 21 -vf vflip " + filename;
    const char *cmd = cmd_temp.c_str();
    ffmpeg = _popen(cmd, "wb");
    return *this;
}

void HINAVIEWER::PBDViewer::write_current_frame()
{
    if (ffmpeg == nullptr)
        return;

    auto *pixels = new char[1280 * 800 * 4];
    glReadPixels(0, 0, 1280, 800, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    fwrite(pixels, sizeof(int) * 1280 * 800, 1, ffmpeg);
}
