import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from message_filters import ApproximateTimeSynchronizer, Subscriber

import onnxruntime as ort
import numpy as np
import cv2
import os, json

class EmotionRecognitionNode(Node):
    def __init__(self):
        super().__init__('emotion_recognition_node')

        # --- Parameters
        self.declare_parameter('model_name', 'fer2013_ResNet50_0')
        # self.declare_parameter('sad_confidence_threshold', 0.20)
        self.declare_parameter('use_trt', True)

        self.model_name = self.get_parameter('model_name').value
        # self.sad_threshold = self.get_parameter('sad_confidence_threshold').value
        use_trt = self.get_parameter('use_trt').value

        # --- Load ONNX model
        pkg_dir = get_package_share_directory('x3_visual')
        model_dir = os.path.join(pkg_dir, 'models', 'emotion_recognition', self.model_name)
        model_path = os.path.join(model_dir, f'model.onnx')
        onnx_config_path = os.path.join(model_dir, 'onnx_config.json')

        # Error handling
        if not os.path.exists(onnx_config_path):
            raise FileNotFoundError(
                f"onnx_config.json not found at {onnx_config_path}."
                f"Re-export the model with export_onnx() to generate it"
            )

        # Load ONNX model config
        with open(onnx_config_path) as f:
            onnx_config = json.load(f)

        self.input_h, self.input_w = onnx_config['input_hw']
        self.color_mode            = onnx_config['color_mode']
        self.normalize_mean        = onnx_config['normalize_mean']  # None or [r,g,b]
        self.normalize_std         = onnx_config['normalize_std']   # None or [r,g,b]
        self.output_is_logits      = onnx_config['output_is_logits']
        self.class_names           = onnx_config['class_names']

        # --- Build ONNXRuntime session with TRT or CUDA execution provider
        self.session = self._load_session(model_path, use_trt)

        # --- Cache input/output binding names for the session
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_name = self.session.get_outputs()[0].name
        self.output_shape = self.session.get_outputs()[0].shape

        self.get_logger().info(
            f'Emotion recognition node ready \n'
            f'  model: {self.model_name} \n'
            f'  input: {self.input_name} {self.input_shape} \n'
            f'  output: {self.output_name} {self.output_shape} \n'
            f'  provider: {self.session.get_providers()}'
        )

        # --- ROS2 Interfaces
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.bridge = CvBridge()

        # Use the ApproximateTimeSynchronizer to align crop and pose by timestamp. 
        # This ensure the face crop and pose belong to a similar detection frame
        self.face_crop_sub = Subscriber(self, Image, '/face_crop', qos_profile=qos)
        self.face_pose_sub = Subscriber(self, PoseStamped, '/face_pose', 10)

        self.sync = ApproximateTimeSynchronizer(
            [self.face_crop_sub, self.face_pose_sub],
            queue_size=10,
            slop=0.1            # 100ms tolerance for timestamp matching
        )
        self.sync.registerCallback(self.synced_callback)

        self.distress_face_pose_pub = self.create_publisher(PoseStamped, '/distress_face_pos', 10)

    def _load_session(self, model_path: str, use_trt: bool) -> ort.InferenceSession:
        if use_trt:
            providers = [
                ('TensorrtExecutionProvider', {
                    'device_id':                0,
                    'trt_max_workspace_size':   512 * 1024 * 1024,
                    'trt_fp16_enable':          True,
                    'trt_engine_cache_enable':  True,
                    'trt_engine_cache_path':    os.path.join('/X3_ROS2_ws', 'src', 'x3_visual', 'models', 'emotion_recognition', self.model_name)
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
        Resize, normalize, convert from BGR to GRAYSCALE, and convert from
        (height, width, channel) to (channel, height, width)
        Return shape (1, 3, H, W) for ORT inference
        '''

        # TODO - make preprocess more robust to the model loaded
        img = cv2.resize(cv_image, (self.input_w, self.input_h))

        if self.color_mode == "grayscale":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # color conversion of MobileViT trained on FER2013
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB) # color conversion of MobileViT trained on FER2013
        else: # rgb
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = img.astype(np.float32) / 255.0        # pixel value normalization

        # Image normalization (for example, ImageNet normalization during training)
        if self.normalize_mean is not None:
            mean    = np.array(self.normalize_mean, dtype=np.float32)
            std     = np.array(self.normalize_std,  dtype=np.float32)
            img     = (img - mean) / std

        img = img.transpose(2, 0, 1)                # from (H,W,C) => (C,H,W)
        img = np.ascontiguousarray(img)
        img = np.expand_dims(img, axis=0)           # add batch dimension

        return img

    def synced_callback(self, crop_msg: Image, pose_msg: PoseStamped):
        # --- Convert ROS image message to OpenCV BGR
        cv_face_crop = self.bridge.imgmsg_to_cv2(crop_msg, desired_encoding="bgr8")
        
        if cv_face_crop.size == 0:
            return
        
        img = self._preprocess(cv_face_crop)

        # --- Run the inference using ORT
        logits = self.session.run([self.output_name],{self.input_name: img})[0]
        
        # softmax the logits
        # TODO - check if exported models already have softmax baked into the logit outputs
        logits = logits[0]      # remove the batch dimension

        if self.output_is_logits:
            exp_logits = np.exp(logits - logits.max())  # numerically stable softmax
            probs = exp_logits / exp_logits.sum()
        else:
            probs = logits

        pred_idx = int(np.argmax(probs))
        pred_label = self.class_names[pred_idx]
        pred_conf = float(probs[pred_idx])

        probs_formatted = list(f"{self.class_names[i][0]}: {prob:5.3f}" for i, prob in enumerate(probs))

        self.get_logger().info(
            f'Detected Emotion: {pred_label} ({pred_conf:.2f}) \n'
            f'Probabilities: {probs_formatted} \n'
            f'Face @ ({pose_msg.pose.position.x:.2f}, '
            f'{pose_msg.pose.position.y:.2f}, '
            f'{pose_msg.pose.position.z:.2f})'
        )

        # --- Republish the face pose if detected sad
        if pred_label in ('Angry', 'Disgust', 'Fear', 'Sad'):
            distress_face_pose = pose_msg
            distress_face_pose.header.stamp = self.get_clock().now().to_msg()
            self.latest_pose = distress_face_pose
            self.distress_face_pose_pub.publish(distress_face_pose)

            # self.get_logger().info(f'{pred_label} face published at distance {pose_msg.pose.position.z:.2f}m')

def main():
    rclpy.init()
    node = EmotionRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()