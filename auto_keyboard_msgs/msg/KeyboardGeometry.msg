# Pose/warp of the keyboard in image space
std_msgs/Header header            # frame_id = "camera_optical_frame"
int32 image_width
int32 image_height
float64[9] h_img_to_rect          # 3x3 homography (row-major): image -> rectified
float64[9] h_rect_to_img          # inverse homography: rectified -> image
geometry_msgs/Point32[4] kb_quad_px  # TL, TR, BR, BL in image pixels (x=u, y=v)
string rect_frame_id              # e.g., "keyboard_rectified"

