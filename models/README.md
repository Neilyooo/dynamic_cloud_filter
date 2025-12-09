# CenterPoint/PointPillar Models

This directory contains the trained PointPillar model files.

## Model Files

The following files are included (copied from TRLO):

- **rpn_centerhead_sim.plan** (13MB) - TensorRT inference engine
  - Combined RPN + Detection Head
  - Optimized for TensorRT 8.5+

- **rpn_centerhead_sim.onnx** (24MB) - ONNX format
  - Original ONNX model (for reference)
  - Can be used to regenerate .plan file for different GPU

- **centerpoint.scn.onnx** (5.2MB) - Sparse convolution network
  - ONNX format sparse backbone
  - Used with spconv library

## Usage

These models are automatically loaded when you run:

```bash
roslaunch dynamic_cloud_filter complete_pipeline.launch
```

The default model path is: `$(find dynamic_cloud_filter)/models/`

## Model Info

- **Training Dataset**: nuScenes / KITTI
- **Input**: Point cloud (max 300,000 points)
- **Output**: 3D bounding boxes for 10 object classes
- **Classes**: 
  - 0: car
  - 1: truck
  - 2: construction_vehicle
  - 3: bus
  - 4: trailer
  - 5: barrier
  - 6: motorcycle
  - 7: bicycle
  - 8: pedestrian
  - 9: traffic_cone

## Performance

- Inference time: ~15-20ms on RTX 3080
- GPU memory: ~2.5GB
- Detection range: 54m × 54m × 8m

## Regenerating TensorRT Engine

If you need to regenerate the .plan file for your specific GPU:

```bash
# Use trtexec tool
trtexec --onnx=rpn_centerhead_sim.onnx \
        --saveEngine=rpn_centerhead_sim.plan \
        --workspace=4096 \
        --fp16
```

## Source

Models are from TRLO (Tightly-Coupled Lidar Odometry) project:
- Original Authors: Yanpeng Jia et al.
- Shenyang Institute of Automation, Chinese Academy of Sciences

## License

See main project LICENSE file.
