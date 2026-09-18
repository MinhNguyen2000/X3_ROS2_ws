import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import numpy as np
import cv2
import onnxruntime as ort
import os

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # --- Parameters 
        self.declare_parameter('agent_name', 'agent0')
        self.declare_parameter('model_name', 'yolov8_n_widerface_01')
        self.declare_parameter('confidence_threshold', 0.70)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('input_hw', [640, 640])      # expected hw ratio by YOLO model
        self.declare_parameter('use_trt', True)
        self.declare_parameter('assumed_face_width', 0.14)  # average adult face width (m) for fallback incase depth not available
        self.declare_parameter('use_depth_estimate', True)  # prefer sampled Z from depth camera over face width estimate        
        self.declare_parameter('depth_patch_radius', 3)	    # half-width (px) of the median_sampling patch
        self.declare_parameter('min_valid_depth_pixels', 5) # min valid px in cropped patch to trust depth
        self.declare_parameter('depth_max_staleness', 0.15) # max time between depth frame vs color frame
        self.declare_parameter('depth_scale', 0.001)        # depth given in (mm), convert to (m)
        self.declare_parameter('publish_depth_crop_debug', False) # publish depth face crop for RViz comparison
        self.declare_parameter('depth_debug_min_m', 0.3)    # fixed colormap range (m) - near clip
        self.declare_parameter('depth_debug_max_m', 8.0)    # fixed colormap range(m) - far clip

        self.agent_name     = self.get_parameter('agent_name').value
        model_name          = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold  = self.get_parameter('nms_threshold').value
        input_hw            = self.get_parameter('input_hw').value
        use_trt             = self.get_parameter('use_trt').value
        self.assumed_face_width = self.get_parameter('assumed_face_width').value
        self.use_depth_estimate  = self.get_parameter('use_depth_estimate').value
        self.depth_patch_radius  = self.get_parameter('depth_patch_radius').value
        self.min_valid_depth_pixels = self.get_parameter('min_valid_depth_pixels').value
        self.depth_max_staleness = self.get_parameter('depth_max_staleness').value
        self.depth_scale         = self.get_parameter('depth_scale').value
        self.publish_depth_crop_debug = self.get_parameter('publish_depth_crop_debug').value
        self.depth_debug_min_m  = self.get_parameter('depth_debug_min_m').value
        self.depth_debug_max_m  = self.get_parameter('depth_debug_max_m').value
        self.input_h, self.input_w = input_hw

        # --- Locate the ONNX model
        pkg_dir = get_package_share_directory('x3_visual')
        model_dir = os.path.join(pkg_dir, 'models', 'face_detection')
        model_path = os.path.join(model_dir, f'{model_name}.onnx')

        # --- Build ONNXRuntime session with TRT or CUDA execution provider
        self.session = self._load_session(model_path, use_trt)

        # --- Cache input/output binding names for the session
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_name = self.session.get_outputs()[0].name
        self.output_shape = self.session.get_outputs()[0].shape

        self.get_logger().info(
            f'Face detection node ready \n'
            f'  model: {model_name} \n'
            f'  input: {self.input_name} {self.input_shape} \n'
            f'  output: {self.output_name} {self.output_shape} \n'
            f'  provider: {self.session.get_providers()}'
        )

        # --- ROS2 interfaces
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.camera_info: CameraInfo | None = None
        self.camera_info_sub    = self.create_subscription(CameraInfo,  f'color/camera_info',   self.camera_info_callback,  10)
        self.image_sub          = self.create_subscription(Image,       f'color/image_raw',     self.image_callback,        qos_profile = qos)
        self.depth_sub          = self.create_subscription(Image,       f'depth/image_raw',     self.depth_callback,        qos_profile = qos) 

        self.crop_pub       = self.create_publisher(Image,              f'color/face_crop',     qos_profile=qos)
        # (DEBUG) colorized depth crop over the same bbox as color image for visually confirming color/depth
        # alignment in RViz. Toggle off via publish_depth_crop_debug once alignment is trusted
        self.depth_crop_pub = self.create_publisher(Image,              f'depth/face_crop',     qos_profile=qos)
        # self.detection_pub  = self.create_publisher(Detection2DArray,   f'color/face_detection', qos_profile = qos)
        self.face_pose_pub  = self.create_publisher(PoseStamped,        f'color/face_pose',      10)

        # --- Declare variables
        self.face_x_smooth = 0.0
        self.face_y_smooth = 0.0
        self.face_z_smooth = 0.0
        self.smooth_alpha = 0.9

        # Cache of most recent data from the subscriptions
        self.latest_depth_image: np.ndarray | None = None
        self.latest_depth_stamp: float | None = None

    def _load_session(self, model_path: str, use_trt: bool) -> ort.InferenceSession:
        "Build an ONNXRuntime Inference Session with TensorRT (by default) or CUDA EP"

        if use_trt:
            providers = [
                ('TensorrtExecutionProvider', {
                    'device_id':                        0,
                    'trt_max_workspace_size':           128 * 1024 * 1024,
                    'trt_fp16_enable':                  True,
                    'trt_engine_cache_enable':          True,
                    'trt_engine_cache_path':            os.path.join('/X3_ROS2_ws', 'src', 'x3_visual', 'models', 'face_detection'),
                    'trt_force_sequential_engine_build': False
                }),
                ('CUDAExecutionProvider', {'device_id': 0}),
                'CPUExecutionProvider'
            ]
        else:
            providers = [
                ('CUDAExecutionProvider', {'device_id': 0}),
                'CPUExecutionProvider'
            ]

        session_options = ort.SessionOptions()
        session_options.log_severity_level = 3

        session = ort.InferenceSession(
            model_path,
            sess_options=session_options,
            providers=providers
        )
        
        return session

    def _preprocess(self, cv_image: np.ndarray) -> np.ndarray:
        '''
        Resize, normalize, convert from BGR to RGB, and convert from
        (height, width, channel) to (channel, height, width)
        Return shape (1, 3, H, W) for ORT inference
        '''
        img = cv2.resize(cv_image, (self.input_w, self.input_h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # color conversion from CV2's
        img = img.astype(np.float32) / 255.0        # pixel value normalization
        img = img.transpose(2, 0, 1)                # (H,W,C) => (C,H,W)
        img = np.ascontiguousarray(img)             
        img = np.expand_dims(img, axis=0)           # add batch dimension
        
        return img

    def _postprocess(self, raw_output: np.ndarray, image_h: int, image_w: int):
        '''
        Decode raw YOLOv5 output

        Args:
            raw_ouput: shape (1, 5, 8400) representing the (batch, [x,y,w,h,conf], anchor)
            image_h: original image height before preprocessing
            image_w: original image width before preprocessing
        
        Returns:
            list of ((x1, y1, w, h), confidence) tuples in original image coordinations
        '''

        detections = raw_output[0].T    # reshape from (5,8400)=>(8400,5) to iterate through anchors

        boxes = []
        confidences = []

        for det in detections:
            x_c, y_c, w, h, conf = det

            if conf < self.conf_threshold:
                continue

            # Scale coordinates from model input space back to original image dimensions
            x_c = x_c / self.input_w * image_w
            y_c = y_c / self.input_h * image_h
            w = w / self.input_w * image_w
            h = h / self.input_h * image_h

            x1 = int(x_c - w/2)
            y1 = int(y_c - h/2)
            boxes.append([x1, y1, int(w), int(h)])
            confidences.append(float(conf))
        
        if len(boxes) == 0:
            return []
        
        # Remove overlapping boxes for the same face
        indices = cv2.dnn.NMSBoxes(
            boxes,
            confidences,
            self.conf_threshold,
            self.nms_threshold
        )

        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                results.append((boxes[i], confidences[i]))

        return results

    def _select_primary_face(self, detections, image_h, image_w):
        '''
        Select the primary face in the scene by considering bbox size and detection confidence
        
        Args:
            detections: list of ((x1, y1, w, h), confidence) tuple of bboxes
            image_h, image_w: dimensions of the original image for normalization
        
        Returns:
            One primary detection tuple
        '''
        if not detections:
            return None
        if len(detections) == 1:
            return detections[0]
        
        image_area = image_h * image_w

        def score(detection):
            (x1, y1, w, h), conf = detection
            area_norm = w * h /image_area
            return 0.4 * area_norm + 0.6 * conf
        
        return max(detections, key=score)

    def _estimate_face_pose(
            self, 
            bbox_cx: float, bbox_cy: float, bbox_w: float, 
            stamp,
            measured_z: float | None = None) -> PoseStamped | None:
        '''
        Monocular depth estimate of the 3D position of the face in the camera
        optical frame using the pinhole camera model and an assumed physical 
        face width.
        
        The camera optical frame convention:
            X: right
            Y: down
            Z: forward (into the scene)
        '''
        fx_camera = self.camera_info.k[0]
        fy_camera = self.camera_info.k[4]
        cx_camera = self.camera_info.k[2]
        cy_camera = self.camera_info.k[5]

        if bbox_w <= 0:
            return None

        if measured_z is not None:  
            # Depth camera distance measurement
            z = measured_z
        else:
            # Face width heuristic approximation   
            z = self.assumed_face_width * fx_camera / bbox_w

        x = (bbox_cx - cx_camera) / fx_camera * z
        y = (bbox_cy - cy_camera) / fy_camera * z

        self.face_x_smooth = self.smooth_alpha * self.face_x_smooth + (1-self.smooth_alpha) * x if self.face_x_smooth else x
        self.face_y_smooth = self.smooth_alpha * self.face_y_smooth + (1-self.smooth_alpha) * y if self.face_y_smooth else y
        self.face_z_smooth = self.smooth_alpha * self.face_z_smooth + (1-self.smooth_alpha) * z if self.face_z_smooth else z

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = f'{self.agent_name}_color_optical_frame'
        pose.pose.position.x     = float(self.face_x_smooth)
        pose.pose.position.y     = float(self.face_y_smooth)
        pose.pose.position.z     = float(self.face_z_smooth)
        pose.pose.orientation.w  = 1.0     # identity — orientation not estimated

        return pose

    def image_callback(self, msg: Image):
        # --- Wait for camera info
        if self.camera_info is None:
            self.get_logger().info('Waiting for camera information')
            return
        
        fx_camera = self.camera_info.k[0]
        cx_camera = self.camera_info.k[2]

        # --- Convert ROS image message to OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_h, image_w = cv_image.shape[:2]

        # --- DEBUG - confirm images are arriving
        self.get_logger().debug(f'Image received: {image_w}x{image_h}')

        # --- Preprocess the image => (1, 3, H, W) float32
        img = self._preprocess(cv_image)

        # DEBUG 2 — confirm preprocessed shape and value range
        self.get_logger().debug(
            f'Preprocessed: shape={img.shape} '
            f'min={img.min():.3f} max={img.max():.3f}'
        )

        # --- Run the inference using ORT
        raw_output = self.session.run([self.output_name], {self.input_name: img})[0]

        # --- Postprocess to decode the boxes and apply non-maximal suppression (NMS)
        detections = self._postprocess(raw_output, image_h, image_w)

        primary_face = self._select_primary_face(detections, image_h, image_w)

        if primary_face is None:
            return
        
        # --- Crop the primary face and publish
        (x1, y1, w, h), _ = primary_face


        x1c = max(x1, 0)
        y1c = max(y1, 0)
        x2c = min(image_w, x1 + w)
        y2c = min(image_h, y1 + h)

        crop = cv_image[y1c:y2c, x1c:x2c]
        if crop.size > 0:
            crop_msg = self.bridge.cv2_to_imgmsg(crop, encoding='bgr8')
            crop_msg.header = msg.header
            self.crop_pub.publish(crop_msg)
            # crop_w = x2c - x1c; crop_h = y2c - y1c
            # self.get_logger().info(f'Published a cropped image ({crop_w:4d},{crop_h:4d})')

        # --- Debug: crop the SAME bbox out of the latest depth frame and
        # publish alongside color/face_crop for visual alignment checking.
        # Relies on depth_registration:=true (same (u,v) = same physical ray
        # in both streams) -- if the two crops don't visually line up on the
        # same face features, that's a registration/calibration problem, not
        # a bug in this node.
        if self.publish_depth_crop_debug and self.latest_depth_image is not None:
            dh, dw = self.latest_depth_image.shape[:2]
            # re-clip against depth's own shape in case it ever differs from color's
            dx2c, dy2c = min(dw, x2c), min(dh, y2c)
            depth_crop = self.latest_depth_image[y1c:dy2c, x1c:dx2c]
            if depth_crop.size > 0:
                depth_crop_colorized = self._colorize_depth_crop(depth_crop)
                depth_crop_msg = self.bridge.cv2_to_imgmsg(depth_crop_colorized, encoding='bgr8')
                depth_crop_msg.header = msg.header    # same stamp/frame as the color crop, for side-by-side comparison
                self.depth_crop_pub.publish(depth_crop_msg)

        # --- Package into the Dection2DArray and publish
        det_array_msg = Detection2DArray()
        det_array_msg.header = msg.header
        
        bbox_cx = float(x1 + w/2)
        bbox_cy = float(y1 + h/2)
        det = Detection2D()
        det.bbox.center.position.x = bbox_cx
        det.bbox.center.position.y = bbox_cy
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        det_array_msg.detections.append(det)

        # self.detection_pub.publish(det_array_msg)
        # self.get_logger().info(f'Published {len(detections)} face detections')

        # --- Sample depth measurement at bbox center
        # (face width heuristic as fallback method inside _estimate_face_pose if unavailable/stale/invalid)
        measured_z = None
        if self.use_depth_estimate:
            color_stamp = self._stamp_to_sec(msg.header.stamp)
            depth_fresh = (
                self.latest_depth_stamp is not None
                and abs(color_stamp - self.latest_depth_stamp) <= self.depth_max_staleness
            )

            if depth_fresh:
                measured_z = self._sample_depth(int(round(bbox_cx)), int(round(bbox_cy)))
                if measured_z is None:
                    self.get_logger().info('Depth patch invalid at bbox center, falling back to face width heuristic')
            else:
                self.get_logger().info('No fresh depth frame cached, falling back to width heuristic')

        # --- Estimate face pose and publish
        pose = self._estimate_face_pose(
            bbox_cx=bbox_cx, bbox_cy=bbox_cy, bbox_w=w, 
            stamp=msg.header.stamp, 
            measured_z=measured_z
        )
        if pose is not None:
            self.face_pose_pub.publish(pose)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def depth_callback(self, msg: Image):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.latest_depth_stamp = self._stamp_to_sec(msg.header.stamp)
        
    def _sample_depth(self, u: int, v: int) -> float | None:
        '''
        Median sample a small path around pixel (u, ) in the most recent depth image. 
        Returns depth in meters, or none if no depth frame is cached or the patch has
        too few valid (nonzero) returns.

        (u,v) must be in color-image piixel space, and the depth stream must be registered
        to color (depth_registration:=true) for this to be a valid lookup
        '''
        if self.latest_depth_image is None:
            return None

        h, w = self.latest_depth_image.shape[:2]
        r = self.depth_patch_radius
        u0, u1 = max(u-r, 0), min(u+r+1, w)
        v0, v1 = max(v-r, 0), min(v+r+1, h)

        patch = self.latest_depth_image[v0:v1, u0: u1].astype(np.float32)
        valid = patch[patch > 0]
        if valid.size < self.min_valid_depth_pixels:
            return None
        return float(np.median(valid)) * self.depth_scale

    def _colorize_depth_crop(self, depth_crop: np.ndarray) -> np.ndarray:
        '''
        Normalize a raw 16UC1 (mm) depth crop into a FIXED-range colormap
        (depth_debug_min_m..depth_debug_max_m), not a per-frame min/max.
        Fixed range means the same color always means the same physical
        distance across frames/publishes -- lets you actually compare
        depth crops over time instead of every crop auto-contrast-stretching
        to fill the color range regardless of real distance.
 
        Invalid (zero) pixels are forced to black rather than mapped into
        the near-range color, so "no return" is visually distinct from
        "close".
        '''
        depth_m = depth_crop.astype(np.float32) * self.depth_scale
        valid = depth_m > 0
 
        span = self.depth_debug_max_m - self.depth_debug_min_m
        normed = np.clip((depth_m - self.depth_debug_min_m) / span, 0.0, 1.0)
        normed_u8 = (normed * 255).astype(np.uint8)
        normed_u8[~valid] = 0
 
        return cv2.applyColorMap(normed_u8, cv2.COLORMAP_JET)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

def main():

    rclpy.init()
    node = FaceDetectionNode()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
