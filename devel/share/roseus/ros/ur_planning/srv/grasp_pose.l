;; Auto-generated. Do not edit!


(when (boundp 'ur_planning::grasp_pose)
  (if (not (find-package "UR_PLANNING"))
    (make-package "UR_PLANNING"))
  (shadow 'grasp_pose (find-package "UR_PLANNING")))
(unless (find-package "UR_PLANNING::GRASP_POSE")
  (make-package "UR_PLANNING::GRASP_POSE"))
(unless (find-package "UR_PLANNING::GRASP_POSEREQUEST")
  (make-package "UR_PLANNING::GRASP_POSEREQUEST"))
(unless (find-package "UR_PLANNING::GRASP_POSERESPONSE")
  (make-package "UR_PLANNING::GRASP_POSERESPONSE"))

(in-package "ROS")





(defclass ur_planning::grasp_poseRequest
  :super ros::object
  :slots (_grasppose_x _grasppose_y _grasppose_z _grasppose_R _grasppose_P _grasppose_Y ))

(defmethod ur_planning::grasp_poseRequest
  (:init
   (&key
    ((:grasppose_x __grasppose_x) 0.0)
    ((:grasppose_y __grasppose_y) 0.0)
    ((:grasppose_z __grasppose_z) 0.0)
    ((:grasppose_R __grasppose_R) 0.0)
    ((:grasppose_P __grasppose_P) 0.0)
    ((:grasppose_Y __grasppose_Y) 0.0)
    )
   (send-super :init)
   (setq _grasppose_x (float __grasppose_x))
   (setq _grasppose_y (float __grasppose_y))
   (setq _grasppose_z (float __grasppose_z))
   (setq _grasppose_R (float __grasppose_R))
   (setq _grasppose_P (float __grasppose_P))
   (setq _grasppose_Y (float __grasppose_Y))
   self)
  (:grasppose_x
   (&optional __grasppose_x)
   (if __grasppose_x (setq _grasppose_x __grasppose_x)) _grasppose_x)
  (:grasppose_y
   (&optional __grasppose_y)
   (if __grasppose_y (setq _grasppose_y __grasppose_y)) _grasppose_y)
  (:grasppose_z
   (&optional __grasppose_z)
   (if __grasppose_z (setq _grasppose_z __grasppose_z)) _grasppose_z)
  (:grasppose_R
   (&optional __grasppose_R)
   (if __grasppose_R (setq _grasppose_R __grasppose_R)) _grasppose_R)
  (:grasppose_P
   (&optional __grasppose_P)
   (if __grasppose_P (setq _grasppose_P __grasppose_P)) _grasppose_P)
  (:grasppose_Y
   (&optional __grasppose_Y)
   (if __grasppose_Y (setq _grasppose_Y __grasppose_Y)) _grasppose_Y)
  (:serialization-length
   ()
   (+
    ;; float64 _grasppose_x
    8
    ;; float64 _grasppose_y
    8
    ;; float64 _grasppose_z
    8
    ;; float64 _grasppose_R
    8
    ;; float64 _grasppose_P
    8
    ;; float64 _grasppose_Y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _grasppose_x
       (sys::poke _grasppose_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _grasppose_y
       (sys::poke _grasppose_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _grasppose_z
       (sys::poke _grasppose_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _grasppose_R
       (sys::poke _grasppose_R (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _grasppose_P
       (sys::poke _grasppose_P (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _grasppose_Y
       (sys::poke _grasppose_Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _grasppose_x
     (setq _grasppose_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _grasppose_y
     (setq _grasppose_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _grasppose_z
     (setq _grasppose_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _grasppose_R
     (setq _grasppose_R (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _grasppose_P
     (setq _grasppose_P (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _grasppose_Y
     (setq _grasppose_Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass ur_planning::grasp_poseResponse
  :super ros::object
  :slots (_success ))

(defmethod ur_planning::grasp_poseResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur_planning::grasp_pose
  :super ros::object
  :slots ())

(setf (get ur_planning::grasp_pose :md5sum-) "8a5978aa25b0b059ee08ab4584d6c721")
(setf (get ur_planning::grasp_pose :datatype-) "ur_planning/grasp_pose")
(setf (get ur_planning::grasp_pose :request) ur_planning::grasp_poseRequest)
(setf (get ur_planning::grasp_pose :response) ur_planning::grasp_poseResponse)

(defmethod ur_planning::grasp_poseRequest
  (:response () (instance ur_planning::grasp_poseResponse :init)))

(setf (get ur_planning::grasp_poseRequest :md5sum-) "8a5978aa25b0b059ee08ab4584d6c721")
(setf (get ur_planning::grasp_poseRequest :datatype-) "ur_planning/grasp_poseRequest")
(setf (get ur_planning::grasp_poseRequest :definition-)
      "float64 grasppose_x
float64 grasppose_y
float64 grasppose_z
float64 grasppose_R
float64 grasppose_P
float64 grasppose_Y
---
bool success


")

(setf (get ur_planning::grasp_poseResponse :md5sum-) "8a5978aa25b0b059ee08ab4584d6c721")
(setf (get ur_planning::grasp_poseResponse :datatype-) "ur_planning/grasp_poseResponse")
(setf (get ur_planning::grasp_poseResponse :definition-)
      "float64 grasppose_x
float64 grasppose_y
float64 grasppose_z
float64 grasppose_R
float64 grasppose_P
float64 grasppose_Y
---
bool success


")



(provide :ur_planning/grasp_pose "8a5978aa25b0b059ee08ab4584d6c721")


