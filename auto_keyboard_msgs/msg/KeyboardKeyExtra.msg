# Per-key extra fields aligned with /keyboard/detections
std_msgs/Header header
string[] id                                   # key labels (A, ENTER, etc.)
geometry_msgs/Point32[] center_rect_px        # centers in rectified plane (pixels)
geometry_msgs/Polygon[] poly_img_px           # optional: 4-pt polygon in image px
float32[] depth_m                             # optional depth per key (NaN if unknown)
int32[] row                                   # optional row index
int32[] col                                   # optional col index
