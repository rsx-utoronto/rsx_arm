import numpy as np

class KeyboardCalibration:
    """
    Handles keyboard calibration using 4 ArUco corner tags.
    Computes 3D positions of all keys based on corner positions and keyboard measurements.
    """

    def __init__(self):
        # Aruco marker IDs for the 4 corners
        self.CORNER_IDS = {
            'top_left': 0,
            'top_right': 1,
            'bottom_left': 2,
            'bottom_right': 3
        }

        # Store detected corner positions (3D in camera frame)
        self.corner_positions_3d = {}
        self.is_calibrated = False

        # Store computed 3D positions of all keys
        self.key_positions_3d = {}

    def update_corner_detection(self, marker_id, position_3d):
        """
        Update detected corner position.

        Args:
            marker_id: Aruco marker ID
            position_3d: (x, y, z) position in camera frame (meters)
        """

        for corner_name, corner_id in self.CORNER_IDS.items():
            if corner_id == marker_id:
                self.corner_positions_3d[corner_name] = np.array(position_3d)
                break

    def compute_key_positions(self):
        """
        Compute 3D positions of all keys using interpolation
        based on the 4 corner positions.
        """

        if len(self.corner_positions_3d) != 4:
            return # Not all corners detected

        tl = self.corner_positions_3d['top_left']
        tr = self.corner_positions_3d['top_right']
        bl = self.corner_positions_3d['bottom_left']
        br = self.corner_positions_3d['bottom_right']

        # Compute 3D position for each key
        #some loop
            #self.key_positions_3d[key_name] = position_3d

        self.is_calibrated = True

    def get_key_position(self, key_name):
        """
        Get the 3D position of a specific key.

        Args:
            key_name: Name of the key

        Returns:
            np.array: (x, y, z)
        """

        if not self.is_calibrated:
            return None

        return self.key_positions_3d.get(key_name)
