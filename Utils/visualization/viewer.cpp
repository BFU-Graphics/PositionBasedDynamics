/**
 * @author YC XIANG.
 * @date 01/29, 2022
 */

#include "viewer.h"

//#define STB_IMAGE_IMPLEMENTATION
//
//#include "stb/stb_image.h"
namespace pbd_viewer
{

    igl::opengl::glfw::Viewer g_viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;

    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi> > g_geometry;
    std::vector<unsigned int> g_id;

    Eigen::VectorXd const *g_q;
    Eigen::VectorXd const *g_qdot;

    //picking variables
    std::vector<unsigned int> g_picked_vertices;
    unsigned int g_selected_obj;
    //mouse UI state variables
    bool g_mouse_dragging = false;
    Eigen::Vector3d g_mouse_win; //mouse window coordinates
    Eigen::Vector3d g_mouse_drag; //last mouse drag vector
    Eigen::Vector3d g_mouse_world;
    Eigen::Vector3d g_mouse_drag_world; //mouse drag delta in the world space

}

igl::opengl::glfw::Viewer &pbd_viewer::viewer()
{ return g_viewer; }

void pbd_viewer::setup(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, bool ps_plot)
{
    g_q = &q;
    g_qdot = &qdot;

    g_viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_menu = [&]()
    {
        ImGuiStyle &style = ImGui::GetStyle();
        style.WindowRounding = 5.3f;
        style.FrameRounding = 2.3f;
        style.ScrollbarRounding = 0;

        style.Colors[ImGuiCol_Text] = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
        style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
//            style.Colors[ImGuiCol_ChildWindowBg]         = ImVec4(0.72f, 0.72f, 0.72f, 1.00f);
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

        g_viewer.core().background_color.setConstant(1.0);
        g_viewer.core().is_animating = true;
        // Draw parent menu content
        menu.draw_viewer_menu();
    };
}

void pbd_viewer::add_object_to_scene(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::RowVector3d color)
{
    if (g_geometry.size() == 0)
    {
        g_id.push_back(0);
    } else
    {
        g_id.push_back(g_viewer.append_mesh());
    }

    g_viewer.data().set_mesh(V, F);
    g_viewer.data().set_colors(color);

    //add mesh to geometry vector
    g_geometry.push_back(std::make_pair(V, F));
}

bool pbd_viewer::mouse_down(igl::opengl::glfw::Viewer &viewer, int x, int y)
{
    g_mouse_win = Eigen::Vector3d(g_viewer.current_mouse_x, viewer.core().viewport(3) - g_viewer.current_mouse_y, 0.);
    igl::unproject(
            g_mouse_win,
            g_viewer.core().view,
            g_viewer.core().proj,
            g_viewer.core().viewport,
            g_mouse_world);

    if (pick_nearest_vertices(g_picked_vertices, g_mouse_win,
                              g_viewer.core().view, g_viewer.core().proj, g_viewer.core().viewport,
                              g_viewer.data().V, g_geometry[0].second, 0.1))
    {

        g_selected_obj = 0;
        g_mouse_dragging = true;
    }
    return false;
}

bool pbd_viewer::mouse_up(igl::opengl::glfw::Viewer &viewer, int x, int y)
{
    g_mouse_dragging = false;
    g_picked_vertices.clear();
    g_mouse_drag_world.setZero();
    return false;
}

bool pbd_viewer::mouse_move(igl::opengl::glfw::Viewer &viewer, int x, int y)
{

    g_mouse_drag = Eigen::Vector3d(g_viewer.current_mouse_x, viewer.core().viewport(3) - g_viewer.current_mouse_y, 0.) - g_mouse_win;
    g_mouse_win = Eigen::Vector3d(g_viewer.current_mouse_x, viewer.core().viewport(3) - g_viewer.current_mouse_y, 0.);

    igl::unproject(
            g_mouse_win,
            g_viewer.core().view,
            g_viewer.core().proj,
            g_viewer.core().viewport,
            g_mouse_drag_world);


    g_mouse_drag_world -= g_mouse_world;

    //std::cout<<"Test: "<<g_mouse_drag_world.transpose()<<"\n";
    igl::unproject(
            g_mouse_win,
            g_viewer.core().view,
            g_viewer.core().proj,
            g_viewer.core().viewport,
            g_mouse_world);


    if (g_mouse_dragging && g_picked_vertices.size() > 0)
    {
        return true;
    }

    return false;
}

const Eigen::Vector3d &pbd_viewer::mouse_world()
{
    return g_mouse_world;
}

const Eigen::Vector3d &pbd_viewer::mouse_drag_world()
{
    return g_mouse_drag_world;
}

bool pbd_viewer::is_mouse_dragging()
{
    return g_mouse_dragging;
}

const std::vector<unsigned int> & pbd_viewer::picked_vertices() {
    return g_picked_vertices;
}