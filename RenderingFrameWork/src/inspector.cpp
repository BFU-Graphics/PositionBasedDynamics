/**
 * @author YC XIANG.
 * @date 02/15, 2022
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

double HINAVIEWER::INSPECTOR::Timeable::simulation_time_ = 0.0;
double HINAVIEWER::INSPECTOR::Timeable::physics_rate = 0.0;
long long HINAVIEWER::INSPECTOR::Timeable::physics_runtime = 0;

void HINAVIEWER::INSPECTOR::Trackable::record(double state)
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

HINAVIEWER::INSPECTOR::Inspector *HINAVIEWER::INSPECTOR::Inspector::track(HINAVIEWER::INSPECTOR::Trackable *trackable, int index)
{
    trackable_ = trackable;
    index_ = index;
    return this;
}

void HINAVIEWER::INSPECTOR::Inspector::plot(const char *label, float start_pos_x, float start_pos_y, float width, float height)
{
    using namespace ImGui;

    SetNextWindowPos(ImVec2(start_pos_x, start_pos_y), ImGuiCond_FirstUseEver);
    SetNextWindowSize(ImVec2(width, height), ImGuiCond_FirstUseEver);

    Begin(
            label, nullptr,
            ImGuiWindowFlags_NoSavedSettings
    );

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

    for (int i = 0; i < trackable_->max_cache_; i += trackable_->max_cache_ / 10)
    {
        DrawList->AddLine(
                normalized_to_pix(ImVec2(static_cast<float>(i) / static_cast<float>(trackable_->max_cache_), 0.97f)),
                normalized_to_pix(ImVec2(static_cast<float>(i) / static_cast<float>(trackable_->max_cache_), 1.f)),
                GetColorU32(ImGuiCol_TextDisabled), 1.2);
    }

    double max = trackable_->max;
    double min = trackable_->min;
    double data_range = max - min;


    for (int i = 1; i < trackable_->max_cache_; ++i)
    {
        if (i < trackable_->tracked_state_.size())
        {
            float x1 = static_cast<float>(i - 1) / static_cast<float>(trackable_->max_cache_);
            float y1 = static_cast<float>((trackable_->tracked_state_[i - 1][index_] - min) / data_range);
            float x2 = static_cast<float>(i) / static_cast<float>(trackable_->max_cache_);
            float y2 = static_cast<float>((trackable_->tracked_state_[i][index_] - min) / data_range);

            ImVec2 n1 = ImVec2(x1, 1.f - y1);
            ImVec2 n2 = ImVec2(x2, 1.f - y2);

            n1.x = ImMin(ImMax(n1.x, 0.f), 1.f);
            n1.y = ImMin(ImMax(n1.y, 0.f), 1.f);
            n2.x = ImMin(ImMax(n2.x, 0.f), 1.f);
            n2.y = ImMin(ImMax(n2.y, 0.f), 1.f);

            ImVec2 p1 = normalized_to_pix(n1);
            ImVec2 p2 = normalized_to_pix(n2);

            DrawList->AddLine(p1, p2, GetColorU32(ImGuiCol_SeparatorActive), 2);
        }
    }

    for (int i = 0; i < 11; ++i)
    {
        DrawList->AddLine(
                normalized_to_pix(ImVec2(0.f, static_cast<float>(i) / static_cast<float>(11))),
                normalized_to_pix(ImVec2(1.f, static_cast<float>(i) / static_cast<float>(11))),
                GetColorU32(ImGuiCol_TextDisabled), 1.2);
        double res = min + static_cast<double>(i) / static_cast<double>(11) * data_range;
        int scale = 0;
        if (res != 0)
            while (res < 1.0 && res > -1.0)
            {
                res *= 10;
                ++scale;
            }
        std::string text = std::to_string(res);
        text.resize(5);
        text.append("e-");
        text.append(std::to_string(scale));
        DrawList->AddText(normalized_to_pix(ImVec2(0, static_cast<float>(10 - i) / static_cast<float>(11))), GetColorU32(ImGuiCol_Text), text.c_str());
    }

    End();
}

void HINAVIEWER::INSPECTOR::ScalarTimeValueInspector::plot(const char *label, float start_pos_x, float start_pos_y, float width, float height)
{
    Inspector::plot(label, start_pos_x, start_pos_y, width, height);
}
