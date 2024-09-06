# Compact Tool-Mounted Optical Tracking System

- Overall Accuracy: 1.2 ± 0.1 mm (RMSE)
- Developed with the support of Dr. Mariana Bernardes and Dr. Junichi Tokuda @ Brigham and Women's Hospital and Harvard Medical School
- Research conducted as part of the 2024 Boston University Research in Science and Engineering program

## Setup

1. Install dependencies in `requirements.txt`.
1. Open Slicer with OpenIGTLink modules installed.
1. Create new OpenIGTLinkIF Connector on default port `18944`.
1. Run `tracker.py`
1. Use `Position` topic as driver for Volume Reslice Driver.

### Note on `requirements.txt`:

Do not `pip install pywin32` on non-Windows machines (i.e., remove this from `requirements.txt`). You may need to `pip install opencv-contrib-python`.

### Calibration

Three calibration notebooks and a 14x10 calibration grid are provided under `calib/` based on OpenCV and `scikit-surgery` documentation tutorials. Innodisk EV2U-SGR1 camera was calibrated using `calib.ipynb` and provided grid. `calib/` also includes `generate_arucoboard.ipynb` if you need to generate a board of ArUco fiducials.
