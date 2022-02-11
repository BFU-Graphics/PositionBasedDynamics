/**
 * @author YC XIANG.
 * @date 02/10, 2022
 */

#include "inspector.h"

#define IMGUI_DEFINE_MATH_OPERATORS

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <algorithm>
#include <limits>
#include <string>

double pbd_inspector::Timeable::simulation_time_ = 0.0;

void pbd_inspector::Trackable::record(double state)
{
    std::vector<double> tmp{Timeable::simulation_time_, state};
    tracked_state_.emplace_back(tmp);
    if (tracked_state_.size() > max_cache_)
        tracked_state_.pop_front();

    max = (*std::max_element(tracked_state_.begin(), tracked_state_.end(), [](const std::vector<double> &lhs, const std::vector<double> &rhs)
    { return lhs[1] < rhs[1]; }))[1];
    min = (*std::min_element(tracked_state_.begin(), tracked_state_.end(), [](const std::vector<double> &lhs, const std::vector<double> &rhs)
    { return lhs[1] < rhs[1]; }))[1];
}

pbd_inspector::Inspector *pbd_inspector::Inspector::track(pbd_inspector::Trackable *trackable, int index)
{
    trackable_ = trackable;
    index_ = index;
    return this;
}

void pbd_inspector::Inspector::plot(const char *label)
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

//    Dummy(ImVec2(0, 3)); // placeholder

    ImVec2 avail = GetContentRegionAvail();
//    ImVec2 Canvas(ImMin(avail.x, avail.y), ImMin(avail.x, avail.y)); // rectangle
    ImVec2 Canvas(avail.x, avail.y);
    Canvas = CalcItemSize(Canvas, style.FramePadding.x * 2.0f, style.FramePadding.y * 2.0f);
    ImRect bb(Window->DC.CursorPos, Window->DC.CursorPos + Canvas);

    const ImGuiID id = Window->GetID(label);
    RenderFrame(bb.Min, bb.Max, GetColorU32(ImGuiCol_FrameBg, 1), true, Style.FrameRounding);


    auto pix_to_normalized = [&bb, &Canvas](ImVec2 pix)
    { return ImVec2((pix.x - bb.Min.x) / Canvas.x, (pix.y - bb.Min.y) / Canvas.y); };

    auto normalized_to_pix = [&bb, &Canvas](ImVec2 norm)
    { return ImVec2(norm.x * Canvas.x + bb.Min.x, norm.y * Canvas.y + bb.Min.y); };

//    for (float i = 0.f; i <= 1.f; i += 1.f / static_cast<float>(trackable_->max_cache_ - 1))
//    for (int i = 0; i < trackable_->max_cache_ - 1; ++i)
//    {
//        DrawList->AddLine(
//                normalized_to_pix(ImVec2(static_cast<float>(i) / static_cast<float>(trackable_->max_cache_), 0.f)),
//                normalized_to_pix(ImVec2(static_cast<float>(i) / static_cast<float>(trackable_->max_cache_), 1.f)),
//                GetColorU32(ImGuiCol_TextDisabled), 1.2);
//        std::string text = std::to_string(trackable_->tracked_state_[i][0]);
//        text.resize(5);
//        DrawList->AddText(normalized_to_pix(ImVec2(static_cast<float>(i) / static_cast<float>(trackable_->max_cache_), 0.97f)), GetColorU32(ImGuiCol_Text), text.c_str());
//    }

    for (float i = 0.f; i <= 1.f; i += 1.f / static_cast<float>(11 - 1))
    {
        DrawList->AddLine(
                normalized_to_pix(ImVec2(0.f, i)),
                normalized_to_pix(ImVec2(1.f, i)),
                GetColorU32(ImGuiCol_TextDisabled), 1.2);
    }

    for (int i = 1; i < trackable_->max_cache_; ++i)
    {
        if (i < trackable_->tracked_state_.size())
        {
            double max = trackable_->max;
            double min = trackable_->min;
            double data_range = max - min;

            float x1 = static_cast<float>(i - 1) / static_cast<float>(trackable_->max_cache_);
            float y1 = static_cast<float>((trackable_->tracked_state_[i - 1][index_] - min) / data_range);
            float x2 = static_cast<float>(i) / static_cast<float>(trackable_->max_cache_);
            float y2 = static_cast<float>((trackable_->tracked_state_[i][index_] - min) / data_range);

            ImVec2 n1 = ImVec2(x1, y1);
            ImVec2 n2 = ImVec2(x2, y2);

            n1.x = ImMin(ImMax(n1.x, 0.f), 1.f);
            n1.y = ImMin(ImMax(n1.y, 0.f), 1.f);
            n2.x = ImMin(ImMax(n2.x, 0.f), 1.f);
            n2.y = ImMin(ImMax(n2.y, 0.f), 1.f);

            ImVec2 p1 = normalized_to_pix(n1);
            ImVec2 p2 = normalized_to_pix(n2);

            DrawList->AddLine(p1, p2, GetColorU32(ImGuiCol_SeparatorActive), 2);
        }
    }
}

void pbd_inspector::ScalarTimeValueInspector::plot(const char *label)
{
    Inspector::plot(label);
}