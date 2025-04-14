#include <NvInfer.h>

#include <iostream>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "external/com_github_foxglove_schemas/ImageAnnotations_generated.h"
#include "opencv2/core.hpp"
//#include "opencv2/highgui.hpp"
#include "opencv2/dnn/dnn.hpp"

#include "HalideBuffer.h"
#include "HalideRuntime.h"
#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/util/file.h"
#include "frc/orin/cuda.h"
#include "frc/orin/resize_normalize.h"
#include "frc/vision/coral_detection_static.h"
#include "frc/vision/cuda_camera_image_callback.h"
#include "frc/vision/vision_generated.h"

using frc::vision::CudaCameraImageCallback;

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");

ABSL_FLAG(std::string, engine_path, "", "Path to the TensorRT engine to use.");

// Set max age on image for processing at 20 ms.  For 60Hz, we should be
// processing at least every 16.7ms
ABSL_FLAG(uint32_t, max_image_age_ms, 50,
          "Max age of an image to process.  For 60hz, we should be processing "
          "an image every 16.7ms, plus ISP/transport delay.");

ABSL_FLAG(uint32_t, skip, 1,
          "Number of images to skip to reduce the framerate of inference to "
          "reduce GPU load.");

ABSL_FLAG(float, confidence, 0.1,
          "Confidence for bounding boxes.");

ABSL_FLAG(float, iou_threshold, 0.5, "IOU threshold");

// /usr/src/tensorrt/bin/trtexec --fp16 --onnx=best.onnx --saveEngine=best416x.engine --useSpinWait --noDataTransfer --useCudaGraph
//
// Export with:
//  yolo export model=~/local/frc4646/yolo/runs/detect/train21/weights/best.pt format="onnx" imgsz=[416,512] data=../../coral.yaml

namespace yolo {

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char *msg) noexcept override {
    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        LOG(FATAL) << msg;
        break;
      case Severity::kERROR:
        LOG(ERROR) << msg;
        break;
      case Severity::kWARNING:
        LOG(WARNING) << msg;
        break;
      case Severity::kINFO:
        LOG(INFO) << msg;
        break;
      case Severity::kVERBOSE:
        VLOG(1) << msg;
        break;
    }
  }
};

class ModelInference {
 public:
  ModelInference(const std::string_view engine_path)
      : engine_(nullptr), context_(nullptr) {
    InitializeEngine(engine_path);
  }

  ~ModelInference() {
    for (void *buf : device_buffers_) {
      cudaFree(buf);
    }
    if (stream_) cudaStreamDestroy(stream_);
  }

  void Infer(const float *input, float *output) {
    // Copy input to device
    CHECK_CUDA(cudaMemcpyAsync(device_buffers_[0], input, input_size_,
                               cudaMemcpyHostToDevice, stream_));

    // Execute inference
    CHECK_EQ(engine_->getNbIOTensors(), 2);
    {
      const char *tensor_name = engine_->getIOTensorName(0);
      context_->setTensorAddress(tensor_name, const_cast<float *>(input));
    }
    {
      const char *tensor_name = engine_->getIOTensorName(1);
      context_->setTensorAddress(tensor_name, output);
    }

    if (!context_->enqueueV3(stream_)) {
      LOG(FATAL) << "Error running inference: enqueueV3 failed!" << std::endl;
    }

    // Synchronize stream
    CHECK_CUDA(cudaStreamSynchronize(stream_));
  }

  nvinfer1::Dims input_dims() const { return input_dims_; }
  nvinfer1::Dims output_dims() const { return output_dims_; }
  size_t input_size() const { return input_size_; }
  size_t output_size() const { return output_size_; }

 private:
  bool InitializeEngine(const std::string_view engine_path) {
    // Read engine file
    std::string engine_data = aos::util::ReadFileToStringOrDie(engine_path);

    // Create runtime and engine
    runtime_ = std::unique_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      LOG(FATAL) << "Error creating TensorRT runtime" << std::endl;
      return false;
    }

    engine_ =
        std::unique_ptr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
            engine_data.data(), engine_data.size()));
    if (!engine_) {
      LOG(FATAL) << "Error deserializing CUDA engine" << std::endl;
      return false;
    }

    context_ = std::unique_ptr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext());
    if (!context_) {
      LOG(FATAL) << "Error creating execution context" << std::endl;
      return false;
    }

    // Create CUDA stream
    CHECK_CUDA(cudaStreamCreate(&stream_));

    // Allocate device buffers
    LOG(INFO) << "Has " << engine_->getNbIOTensors() << " tensors";
    for (int i = 0; i < engine_->getNbIOTensors(); i++) {
      const char *tensor_name = engine_->getIOTensorName(i);
      nvinfer1::Dims dims = engine_->getTensorShape(tensor_name);
      size_t size = 1;
      for (int j = 0; j < dims.nbDims; j++) {
        size *= dims.d[j];
      }
      size *= sizeof(float);  // Assuming float32 data type

      void *deviceBuffer;
      CHECK_CUDA(cudaMalloc(&deviceBuffer, size));
      device_buffers_.push_back(deviceBuffer);

      if (engine_->getTensorIOMode(tensor_name) ==
          nvinfer1::TensorIOMode::kINPUT) {
        input_size_ = size;
        input_dims_ = dims;
      } else {
        output_size_ = size;
        output_dims_ = dims;
      }
    }

    return true;
  }

  Logger logger_;

  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_;
  std::vector<void *> device_buffers_;
  size_t input_size_;
  size_t output_size_;
  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
};

class YoloApplication {
 public:
  static constexpr size_t kNormalizedWidth = 512;
  static constexpr size_t kNormalizedHeight = 416;

  YoloApplication(aos::EventLoop *event_loop, std::string_view engine_path)
      : event_loop_(event_loop),
        inference_(engine_path),
        output_(inference_.output_size()),
        callback_(
            event_loop_, "/camera1/gray",
            [this](const frc::vision::CameraImage &image,
                   aos::monotonic_clock::time_point eof) {
              if (skip_ != 0) {
                --skip_;
                return;
              } else {
                skip_ = absl::GetFlag(FLAGS_skip);
              }
              DetectImage(image, eof);
            },
            std::chrono::milliseconds(absl::GetFlag(FLAGS_max_image_age_ms))),
        image_annotations_sender_(
            event_loop->MakeSender<foxglove::ImageAnnotations>(
                "/camera1/coral")),
        detections_sender_(
            event_loop->MakeSender<frc::vision::BoundingBoxesStatic>(
                "/camera1/coral")) {
    CHECK_EQ(inference_.input_size(),
             kNormalizedWidth * kNormalizedHeight * 3 * sizeof(float));

    {
      nvinfer1::Dims d = inference_.input_dims();
      LOG(INFO) << "Input: " << d.nbDims << " [" << d.d[0] << " " << d.d[1] << " "
              << d.d[2] << " " << d.d[3] << "]";

      d = inference_.output_dims();
      LOG(INFO) << "Output: " << d.nbDims << " [" << d.d[0] << " " << d.d[1]
              << " " << d.d[2] << "]";
    }
  }

  void PinMemory(aos::ShmEventLoop *shm_event_loop) {
    callback_.PinMemory(shm_event_loop);
  }

 private:
  void Preprocess(const frc::vision::CameraImage &camera_image,
                  float *normalized_image) {
    std::array<halide_dimension_t, 2> image_dimensions{{
        {
            /*.min =*/0,
            /*.extent =*/
            static_cast<int32_t>(camera_image.cols()),
            /*.stride =*/1,
            /*.flags =*/0,
        },
        {
            /*.min = */ 0,
            /*.extent =*/
            static_cast<int32_t>(camera_image.rows()),
            /*.stride =*/
            static_cast<int32_t>(camera_image.cols()),
            /*.flags =*/0,
        },
    }};

    Halide::Runtime::Buffer<const uint8_t, 2> image(
        reinterpret_cast<const uint8_t *>(camera_image.data()->data()),
        image_dimensions.size(), image_dimensions.data());

    std::array<halide_dimension_t, 3> normalized_dimensions{{
        {
            /*.min =*/0,
            /*.extent =*/
            static_cast<int32_t>(kNormalizedWidth),
            /*.stride =*/1,
            /*.flags =*/0,
        },
        {
            /*.min = */ 0,
            /*.extent =*/
            static_cast<int32_t>(kNormalizedHeight),
            /*.stride =*/
            static_cast<int32_t>(kNormalizedWidth),
            /*.flags =*/0,
        },
        {
            /*.min = */ 0,
            /*.extent =*/
            static_cast<int32_t>(3),
            /*.stride =*/
            static_cast<int32_t>(kNormalizedWidth * kNormalizedHeight),
            /*.flags =*/0,
        },
    }};

    Halide::Runtime::Buffer<float, 3> normalized(normalized_image,
                                                 normalized_dimensions.size(),
                                                 normalized_dimensions.data());

    aos::monotonic_clock::time_point start = aos::monotonic_clock::now();
    resize_normalize(image, normalized);
    VLOG(1) << "Took: "
            << std::chrono::duration<double, std::milli>(
                   aos::monotonic_clock::now() - start)
                   .count()
            << "ms";
  }

  struct Detection {
    float x0;
    float y0;
    float x1;
    float y1;
    float confidence;

    float width() const { return x1 - x0; }
    float height() const { return y1 - y0; }
  };

  void DetectImage(const frc::vision::CameraImage &camera_image,
                   aos::monotonic_clock::time_point /*eof*/) {
    Preprocess(camera_image,
               reinterpret_cast<float *>(normalized_image_device_.get()));
    /*
    float *dat = reinterpret_cast<float *>(normalized_image_device_.get());

    cv::Mat bgr(kNormalizedHeight, kNormalizedWidth, CV_8UC3);

    for (size_t j = 0; j < kNormalizedHeight; ++j) {
      for (size_t i = 0; i < kNormalizedWidth; ++i) {
        float v = dat[i + j * kNormalizedWidth];
        uint8_t vb = static_cast<uint8_t>(v * 255.0f);

        bgr.at<cv::Vec3b>(j, i) = cv::Vec3b(vb, vb, vb);
        //LOG(INFO) << "img[" << i << ", " << j << "] -> " << dat[i + j * kNormalizedWidth];
      }
    }
    cv::imwrite("/tmp/img.png", bgr);
    */

    inference_.Infer(normalized_image_device_.get(), output_.get());

    nvinfer1::Dims d = inference_.output_dims();
    float c = 0;
    float confidence_threshold = absl::GetFlag(FLAGS_confidence);

    std::vector<Detection> detections;

    for (int j = 0; j < d.d[2]; ++j) {
      const float confidence = output_.get()[j + d.d[2] * 4];
      const float xc = output_.get()[j + d.d[2] * 0] * 3.0 + 64;
      const float yc = output_.get()[j + d.d[2] * 1] * 3.0;
      const float w = output_.get()[j + d.d[2] * 2] * 3.0;
      const float h = output_.get()[j + d.d[2] * 3] * 3.0;
   
      Detection detection{
          .x0 = xc - w / 2.0f,
          .y0 = yc - h / 2.0f,
          .x1 = xc + w / 2.0f,
          .y1 = yc + h / 2.0f,
          .confidence = confidence,
      };

      c = std::max(confidence, c);
      if (confidence < confidence_threshold) {
        continue;
      }

      VLOG(1) << j << " -> [" << xc << ", " << yc << ", " << w << ", " << h
                << ", " << confidence << "]";

      detections.emplace_back(detection);
    }

    std::vector<Detection> nms = NMSBoxes(detections);
    VLOG(1) << "Found " << nms.size() << " detections";

    {
      aos::Sender<frc::vision::BoundingBoxesStatic>::StaticBuilder builder =
          detections_sender_.MakeStaticBuilder();
      auto boxes = builder->add_boxes();
      CHECK(boxes->reserve(nms.size()));
      for (const Detection detection : nms) {
        // TODO(austin): Undistort.

        frc::vision::BoundingBoxStatic *box = boxes->emplace_back();
        box->set_class_id(0);
        box->set_confidence(detection.confidence);
        box->set_x0(detection.x0);
        box->set_y0(detection.y0);
        box->set_width(detection.width());
        box->set_height(detection.height());
      }
      builder.CheckOk(builder.Send());
    }

    auto builder = image_annotations_sender_.MakeBuilder();
    std::vector<flatbuffers::Offset<foxglove::PointsAnnotation>>
        foxglove_corners;
    for (const Detection detection : nms) {
      const struct timespec now_t =
          aos::time::to_timespec(event_loop_->context().monotonic_event_time);
      foxglove::Time time{static_cast<uint32_t>(now_t.tv_sec),
                          static_cast<uint32_t>(now_t.tv_nsec)};

      const flatbuffers::Offset<foxglove::Color> color_offset =
          foxglove::CreateColor(*builder.fbb(), 1.0, 1.0, 0.0, 0.5);

      std::vector<flatbuffers::Offset<foxglove::Point2>> points_offsets;
      points_offsets.push_back(
          foxglove::CreatePoint2(*builder.fbb(), detection.x0, detection.y0));
      points_offsets.push_back(
          foxglove::CreatePoint2(*builder.fbb(), detection.x0, detection.y1));
      points_offsets.push_back(
          foxglove::CreatePoint2(*builder.fbb(), detection.x1, detection.y1));
      points_offsets.push_back(
          foxglove::CreatePoint2(*builder.fbb(), detection.x1, detection.y0));

      const flatbuffers::Offset<
          flatbuffers::Vector<flatbuffers::Offset<foxglove::Point2>>>
          points_offset = builder.fbb()->CreateVector(points_offsets);

      std::vector<flatbuffers::Offset<foxglove::Color>> color_offsets(
          points_offsets.size(), color_offset);

      auto colors_offset = builder.fbb()->CreateVector(color_offsets);
      foxglove::PointsAnnotation::Builder points_builder(*builder.fbb());
      points_builder.add_timestamp(&time);
      points_builder.add_type(foxglove::PointsAnnotationType::LINE_LOOP);
      points_builder.add_points(points_offset);
      points_builder.add_outline_color(color_offset);
      points_builder.add_outline_colors(colors_offset);
      points_builder.add_thickness(5);
      foxglove_corners.push_back(points_builder.Finish());
    }
    const auto corners_offset = builder.fbb()->CreateVector(foxglove_corners);
    foxglove::ImageAnnotations::Builder annotation_builder(*builder.fbb());
    annotation_builder.add_points(corners_offset);
    builder.CheckOk(builder.Send(annotation_builder.Finish()));
    VLOG(1) << "Max confidence: " << c;
  }

  std::vector<Detection> NMSBoxes(const std::vector<Detection> &detections) {
    std::vector<cv::Rect> boxes;
    boxes.reserve(detections.size());
    std::vector<float> confidences;
    confidences.reserve(detections.size());

    for (const Detection detection : detections) {
      confidences.push_back(detection.confidence);
      boxes.push_back(cv::Rect(static_cast<int>(detection.x0),
                               static_cast<int>(detection.y0),
                               static_cast<int>(detection.width()),
                               static_cast<int>(detection.height())));
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, absl::GetFlag(FLAGS_confidence),
                      absl::GetFlag(FLAGS_iou_threshold), nms_result);

    std::vector<Detection> result;
    result.reserve(nms_result.size());
    for (int idx : nms_result) {
      result.emplace_back(Detection{
          .x0 = static_cast<float>(boxes[idx].x),
          .y0 = static_cast<float>(boxes[idx].y),
          .x1 = static_cast<float>(boxes[idx].x + boxes[idx].width),
          .y1 = static_cast<float>(boxes[idx].y + boxes[idx].height),
          .confidence = confidences[idx],
      });
    }

    return result;
  }

  aos::EventLoop *event_loop_;

  ModelInference inference_;

  frc::apriltag::UnifiedMemory<float> output_;

  CudaCameraImageCallback callback_;

  frc::apriltag::UnifiedMemory<float> normalized_image_device_{
      3 * kNormalizedWidth * kNormalizedHeight};

  size_t skip_ = 0;

  aos::Sender<foxglove::ImageAnnotations> image_annotations_sender_;
  aos::Sender<frc::vision::BoundingBoxesStatic> detections_sender_;
};

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());
  event_loop.SetRuntimeRealtimePriority(6);

  YoloApplication yolo(&event_loop, absl::GetFlag(FLAGS_engine_path));

  // yolo.PinMemory(&event_loop);

  event_loop.Run();
}

}  // namespace yolo

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  yolo::Main();
  return 0;
}
