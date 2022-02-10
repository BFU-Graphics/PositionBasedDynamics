/**
 * @author YC XIANG.
 * @date 01/29, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_VIEWER_H
#define POSITIONBASEDDYNAMICS_VIEWER_H

#include <igl/opengl/glfw/Viewer.h>

#define IMGUI_DEFINE_MATH_OPERATORS

#include <igl/unproject.h>
#include "pick_nearest_vertices.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <vector>
#include <array>
#include <deque>

namespace pbd_viewer
{
    /// set up igl viewer system
    void setup(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, bool ps_plot = false);

    /// int object
    void add_object_to_scene(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::RowVector3d color);

    /// update rendering vertices
    void update_vertex_positions(unsigned int id, Eigen::Ref<const Eigen::VectorXd> pos);

    /// get igl viewer
    igl::opengl::glfw::Viewer &viewer();

    /// mouse callback
    bool mouse_down(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_up(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_move(igl::opengl::glfw::Viewer &viewer, int x, int y);

    const Eigen::Vector3d &mouse_world();

    const Eigen::Vector3d &mouse_drag_world();

    const std::vector<unsigned int> &picked_vertices();

    bool is_mouse_dragging();

    // Custom Plot Canvas

    template<int size, typename T = double>
    class Trackable
    {
    public:
        virtual std::deque<std::array<T, size>>& track(int index) = 0;
    };

    template<int size, typename T = double>
    class ScalarTimeValueInspector
    {
    public:
        ScalarTimeValueInspector<size, T> *track(std::deque<std::array<T, size>> &state, int index);

        void plot(const char *label);

    public:
        std::deque<std::array<T, size>> *state_ptr_;
        int index_;
    };

    template<int size, typename T>
    ScalarTimeValueInspector<size, T> *ScalarTimeValueInspector<size, T>::track(std::deque<std::array<T, size>> &state, int index)
    {
        state_ptr_ = &state;
        index_ = index;
        return this;
    }

    template<int size, typename T>
    void ScalarTimeValueInspector<size, T>::plot(const char *label)
    {
        using namespace ImGui;

        ImGuiContext &g = *GImGui;
        const ImGuiStyle &style = g.Style;

        const ImGuiStyle &Style = GetStyle();
        const ImGuiIO &IO = GetIO();
        ImDrawList *DrawList = GetWindowDrawList();
        ImGuiWindow *Window = GetCurrentWindow();

        if (Window->SkipItems)
            return;

        Dummy(ImVec2(0, 3));

        ImVec2 avail = GetContentRegionAvail();
        ImVec2 Canvas(ImMin(avail.x, avail.y), ImMin(avail.x, avail.y));
        Canvas = CalcItemSize(Canvas, style.FramePadding.x * 2.0f, style.FramePadding.y * 2.0f);
        ImRect bb(Window->DC.CursorPos, Window->DC.CursorPos + Canvas);

        const ImGuiID id = Window->GetID(label);
        RenderFrame(bb.Min, bb.Max, GetColorU32(ImGuiCol_FrameBg, 1), true, Style.FrameRounding);
    }

    template<int size, typename T = double>
    void track_scalar(std::deque<std::array<T, size>> &state);
}

#endif //POSITIONBASEDDYNAMICS_VIEWER_H
