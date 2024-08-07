# Compact Tool-Mounted Optical Tracking System

Image-guided intraoperative navigation plays an important role in minimally invasive surgical procedures. Such *in-situ* visualization improves lesion targeting while avoiding vital surrounding structures, thereby reducing the risk of complications and need for revision surgeries. Conventional optical tracking systems (OTSs) require a bulky stereo near-infrared camera placed above the surgical field to localize passive optical tools. However, such techniques require direct line of sight between the camera and surgical tools. Specifically, conventional OTSs are often unable to track tools when even a single fiducial is obstructed. Surgeons attempting to avoid this issue may find their workspaces limited and procedure times lengthened. We present an alternative, tool-mounted wide-angle visible-light camera system that eliminates typical line-of-sight issues. Our approach tracks many redundant fiducials surrounding an incision, helping maintain accuracy even when several fiducials are obstructed by surgical tools and/or surgeons’ hands. To further demonstrate how the novel OTS may be integrated into real-time surgical navigation software, we use the computed tool tip pose to resample a preoperative MRI image in the medical visualization platform 3D Slicer. Tool tip accuracy was evaluated geometrically using a known grid of points relative to world origin, in which millimeter accuracy was obtained. Presented results show the potential in practically reducing the size of OTSs, which may address the line-of-sight issue and shorten operation time.

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
