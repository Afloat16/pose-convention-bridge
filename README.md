# pose-convention-bridge

Explicit rigid camera-pose conversion between OpenCV/OpenGL and world-to-camera/camera-to-world conventions.

[中文](README.zh-CN.md) · [Demand evidence](docs/EVIDENCE.md) · [License](LICENSE)

Independent, experimental utility maintained by **Afloat16**. Original implementation; no upstream endorsement is implied.

## Install

Python 3.10+.

```bash
git clone https://github.com/Afloat16/pose-convention-bridge.git
cd pose-convention-bridge
python -m venv .venv
# Activate the virtual environment for your OS, then:
python -m pip install -e '.[dev]'
```

The core package uses the Python standard library only. No GPU, ROS runtime, API key, hosted service or model weights are required for the core offline workflow.

## First run

```bash
pose-convention-bridge examples/poses.json --input-direction c2w --input-axes opencv --format nerfstudio --output transforms.json
pose-convention-bridge images.txt --input-format colmap-images --input-direction w2c --input-axes opencv --output cameras.json
pose-convention-bridge trajectory.txt --input-format tum --input-direction c2w --input-axes opencv --image-map images.json --intrinsics intrinsics.json --format nerfstudio --output transforms.json
```

## Inputs and checks

JSON `{frames:[{id,file_path,matrix}]}` (also transform_matrix), COLMAP images.txt, or TUM `timestamp tx ty tz qx qy qz qw`. COLMAP is explicitly w2c/OpenCV with wxyz quaternions; conflicting declarations fail. TUM preserves timestamp strings and uses xyzw. `--image-map` maps exact timestamps to real filenames; names and intrinsics are never invented.

Explicit input/output direction and camera axes; basis changes preserve world centers. Validates finite rigid matrices, orthonormality, determinant +1 and homogeneous rows. `--world-similarity` accepts `{matrix:rigid4x4,scale:s}` with p_out=s*R*p_in+t. `--dataparser-transform` explicitly accepts Nerfstudio `{transform:3x4,scale:s}` with p_out=s*(R*p_in+t). These options are mutually exclusive.

## Boundaries

No automatic up-axis orientation, centering, scale estimation, RTAB-Map extrinsic inference or automatic applied_transform application. Scene points/meshes must receive the same world mapping separately. Nerfstudio output requires c2w/OpenGL, real filenames and explicit fl_x,fl_y,cx,cy,w,h, globally or per frame. Intrinsics/distortion are carried through, not calibrated/undistorted; no rendering-quality guarantee. Limits: 64 MiB input, 100,000 poses.

## Output and development

Audit commands exit 0 for no error findings, 1 for errors (also warnings with --strict), and 2 for malformed input/report-write failure. Pose conversion exits 0 for a valid conversion and 2 for input/write errors. --output creates a new JSON file exclusively; existing files are never overwritten. Detailed audit findings are capped at 200 while total severity counts remain accurate.

```bash
python -m unittest discover -s tests -v
python -m compileall -q src tests
python -m build
```

The configured CI matrix covers Ubuntu, Windows and macOS with Python 3.10/3.13; inspect Actions for actual results. Fixtures are synthetic. No private captures or upstream user attachments are redistributed.

Report problems with command/version, expected/actual result and a minimal synthetic reproduction. Keep source attribution and compatible licensing. MIT license, copyright Afloat16.
