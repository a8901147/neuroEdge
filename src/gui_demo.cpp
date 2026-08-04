// Graphical counterpart to src/main.cpp: same <1,6> wearable-fusion engine,
// same CSV fixture, same 1kHz real-time pacing and NoHeapGuard discipline —
// but rendered as real ImPlot line charts in a native GLFW/OpenGL window
// instead of an ASCII terminal bar. Per PRD 4.3, this is turnkey/off-the-shelf
// (ImGui + ImPlot via CMake FetchContent): no custom web frontend, no DOM.

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <implot.h>
#include <string>
#include <thread>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/providers/csv_signal_provider.hpp"

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
#include "edgeneuro/no_heap_guard.hpp"
#endif

using namespace edgeneuro;

namespace {

constexpr std::size_t kEmgChannels = 1;
constexpr std::size_t kImuChannels = 6;
constexpr std::size_t kWindowSize = 50;
constexpr std::size_t kMaxSamples = 8192;
constexpr std::size_t kNumFeatures = kEmgChannels + kImuChannels;
constexpr std::size_t kNumClasses = 2;

using Provider = CsvSignalProvider<float, kEmgChannels, kImuChannels, kMaxSamples>;
using Classifier = LdaClassifier<float, kNumFeatures, kNumClasses>;
using Engine = EdgeNeuro<
    float, kEmgChannels, kImuChannels, kWindowSize, Provider,
    IirFilter<float>, PassThroughFilter<float>,
    MavFeature<float, kWindowSize>, Classifier>;

Engine build_engine(const std::string& csv_path) {
    std::array<IirFilter<float>, kEmgChannels> emg_filters{IirFilter<float>(0.8f, -1.6f, 0.8f, -1.56f, 0.64f)};
    std::array<PassThroughFilter<float>, kImuChannels> imu_filters{};
    const std::array<std::array<float, kNumFeatures>, kNumClasses> weights{{
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    }};
    const std::array<float, kNumClasses> bias{2.0f, 0.0f};
    return Engine(Provider(csv_path), emg_filters, imu_filters, MavFeature<float, kWindowSize>{}, Classifier(weights, bias));
}

// Fixed-capacity scrolling buffer for plotting: same "no reallocation once
// running" discipline as the DSP engine, applied to the visualization side.
template <std::size_t Capacity>
struct ScrollingBuffer {
    std::array<float, Capacity> xs{};
    std::array<float, Capacity> ys{};
    std::size_t count{0};
    std::size_t head{0};

    void push(float x, float y) {
        xs[head] = x;
        ys[head] = y;
        head = (head + 1) % Capacity;
        count = std::min(count + 1, Capacity);
    }

    // ImPlot can draw directly from a ring-buffered array given an offset,
    // avoiding any copy into a contiguous scratch buffer.
    int offset() const { return count < Capacity ? 0 : static_cast<int>(head); }
};

void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

} // namespace

int main(int argc, char** argv) {
    const std::string csv_path = argc > 1 ? argv[1] : "data/wearable_1emg_6imu.csv";

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE); // required on macOS

    GLFWwindow* window = glfwCreateWindow(1280, 720, "EdgeNeuro Phase 1 -- Turnkey Scientific Oscilloscope", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    Engine engine = build_engine(csv_path);

    constexpr std::size_t kPlotCapacity = 2000;
    ScrollingBuffer<kPlotCapacity> emg_mav_plot;
    float t = 0.0f;

    std::size_t out_class = 0;
    std::size_t gesture_class = 0;
    std::size_t sample_index = 0;
    double total_tick_us = 0.0;
    double max_tick_us = 0.0;
    double last_tick_us = 0.0;
    bool stream_active = true;

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    NoHeapGuard::reset_count();
#endif

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Advance the 1kHz stream by a handful of samples per rendered
        // frame (paced to real time), same discipline as the terminal demo:
        // NoHeapGuard is armed ONLY around engine.tick().
        for (int i = 0; i < 20 && stream_active; ++i) {
            const auto start = std::chrono::steady_clock::now();
            bool ok;
            {
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
                NoHeapGuard guard;
#endif
                ok = engine.tick(out_class);
            }
            const auto end = std::chrono::steady_clock::now();
            if (!ok) {
                stream_active = false;
                break;
            }

            last_tick_us = std::chrono::duration<double, std::micro>(end - start).count();
            ++sample_index;
            total_tick_us += last_tick_us;
            max_tick_us = std::max(max_tick_us, last_tick_us);
            if (engine.has_result()) gesture_class = out_class;

            t += 0.001f; // 1ms per sample
            emg_mav_plot.push(t, engine.last_features()[0]);

            std::this_thread::sleep_for(std::chrono::microseconds(1000) -
                                         std::chrono::duration_cast<std::chrono::microseconds>(end - start));
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin("EdgeNeuro", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove);

        ImGui::Text("EdgeNeuro Phase 1 -- Turnkey Scientific Oscilloscope");
        ImGui::Text("Mode: EdgeNeuro<1,6> wearable fusion (1x EMG + 6-axis IMU @ 1kHz)");
        ImGui::Separator();

        ImGui::TextColored(gesture_class == 1 ? ImVec4(0.9f, 0.3f, 0.3f, 1.0f) : ImVec4(0.3f, 0.9f, 0.3f, 1.0f),
                            "Decoded gesture: %s", gesture_class == 1 ? "GRASP" : "rest");
        ImGui::Text("Latency (tick): last=%.3f us  avg=%.3f us  max=%.3f us  [target < 100 us]",
                    last_tick_us, sample_index ? total_tick_us / static_cast<double>(sample_index) : 0.0, max_tick_us);
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
        ImGui::Text("malloc_count: %zu  [must stay 0]", NoHeapGuard::count());
#endif
        ImGui::Text("samples played: %zu", sample_index);
        if (!stream_active) {
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.3f, 1.0f), "Stream finished -- close the window to exit.");
        }

        if (ImPlot::BeginPlot("EMG MAV (per-window feature)", ImVec2(-1, 400))) {
            // AutoFit on both axes: without it, ImPlot only fits the axis
            // range once (during the first frame or two, when the scrolling
            // buffer barely has any points in it) and never re-fits as more
            // data streams in, leaving the plot stuck showing just the
            // first ~20ms of a 2-second run.
            ImPlot::SetupAxes("time (s)", "MAV", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            if (emg_mav_plot.count > 0) {
                ImPlot::PlotLine("emg0_mav", emg_mav_plot.xs.data(), emg_mav_plot.ys.data(),
                                  static_cast<int>(emg_mav_plot.count), 0, emg_mav_plot.offset());
            }
            ImPlot::EndPlot();
        }

        ImGui::End();

        ImGui::Render();
        int display_w = 0, display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
