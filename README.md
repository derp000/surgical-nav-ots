# Compact Surgical Navigation Optical Tracking System

Developed under guidance of Dr. Junichi Tokuda @ Brigham and Women's Hospital/Harvard Medical School

Todo description.

## Setup

1. Create a `venv` and install dependencies in `requirements.txt`.
1. Open Slicer with OpenIGTLink modules installed.
1. Create new OpenIGTLinkIF Connector on default port `18944`.
1. Run `tracker.py`
1. Use `Position` topic as driver for Volume Reslice Driver.

### Note on `requirements.txt`:

Do not `pip install pywin32` on non-Windows machines (i.e., remove this from `requirements.txt`). You may need to `pip install opencv-contrib-python`.

### Calibration

Three calibration notebooks and a 14x10 calibration grid are provided under `calib/` based on OpenCV and `scikit-surgery` documentation tutorials. Innodisk `EV2U-SGR1` camera was calibrated using `calib.ipynb` and `calibration-grid-6mm.pdf`.
