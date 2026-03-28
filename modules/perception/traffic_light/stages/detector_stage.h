#pragma once

#include "modules/perception/traffic_light/interface/stage.h"
#include <vector>

namespace apollo {
namespace perception {
namespace traffic_light {

// Step 2: 视觉捕获 (Vision Detection & Recognition)
// 职责: 根据 Prompt ROI 裁剪图像喂给模型，输出单帧检出的实体 (2D Bbox, Color, Shape)
class DetectorStage : public BaseStage {
public:
    DetectorStage() = default;
    ~DetectorStage() override = default;

    std::string Name() const override { return "DetectorStage"; }

    bool Init(const StageConfig& config) override {
        // Init neural network engine (e.g. TensorRT)
        return true;
    }

    bool Process(PipelineContext* context) override {
        if (!context) return false;
        std::lock_guard<std::mutex> lock(context->rw_mutex);

        std::vector<VisualLight> all_detected_lights;

        for (const auto& prompt : context->prompts) {
            // Pseudo code for Neural Network Inference
            // std::vector<VisualLight> detected = RunModelEngine(context->image_tele, prompt.roi_box);

            // Mock: Found a green circle
            if (prompt.source == 1) { // High confidence area
                 VisualLight v_light;
                 v_light.bbox = prompt.roi_box; // Suppose model localized it precisely
                 v_light.color = LightColor::GREEN;
                 v_light.shape = LightShape::CIRCLE;
                 v_light.confidence = 0.95f;
                 all_detected_lights.push_back(v_light);
            }
        }

        // Apply NMS to remove overlapping bounds from different prompts
        // NMS(all_detected_lights);

        context->visual_lights = std::move(all_detected_lights);
        return true;
    }
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo
