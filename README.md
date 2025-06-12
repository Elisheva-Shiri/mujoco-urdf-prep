# Robot Model Simulation and Data Collection Project

This project implements a workflow for converting robot models from XACRO to URDF format, simulating them in MuJoCo, and collecting/organizing simulation data.

## Project Structure
```
mujoco_proj/
├── model/                      # Contains all model files
│   ├── mjmodel.xml            # MuJoCo model configuration
│   ├── shadow_hand_processed.urdf  # Processed URDF model
│   └── *.stl                   # Mesh files (git-ignored)
├── data/                       # Simulation and analysis data
│   ├── arm/                    # Arm-related data
│   │   ├── T0/                # Run folders
│   │   │   ├── MJDATA.TXT     # MuJoCo simulation data (git-ignored)
│   │   │   ├── screenshot.png # Screenshots (git-ignored)
│   │   │   └── MUJOCO_LOG.TXT # Log files (git-ignored)
│   │   ├── T1/
│   │   └── ...
│   ├── palm/                   # Palm-related data
│   ├── WR/                     # Wrist-related data
│   ├── data_angles.csv        # Extracted joint angle data (git-ignored)
│   └── MJMODEL.TXT            # MuJoCo model data (git-ignored)
├── ur_description/            # UR robot description files (git-ignored)
├── sr_description/            # Shadow Robot description files (git-ignored)
├── extract_files.py           # Script for extracting model files
├── extract_angle_indirect(in Process).py  # Script for extracting QPOS data
├── extract_mesh.py            # Script for mesh extraction
├── interactive_joint_control.py # Interactive joint control interface
├── urdf_editor.py             # URDF file editing utilities
├── utils.py                   # Utility functions
├── .gitignore                # Git ignore rules
└── pyproject.toml             # Project dependencies
```

## Version Control
The project uses Git for version control. The following files and directories are ignored:
- Generated data files (*.csv, MJDATA.TXT, etc.)
- Model files (*.stl, *.mjb)
- Screenshots and logs
- Python cache and build files
- Virtual environment directories
- IDE configuration files

See `.gitignore` for complete list of ignored files.

## Workflow

1. **Model Conversion (XACRO → URDF)**
   - Source models are stored in `ur_description/` and `sr_description/`
   - Conversion process handled by `urdf_editor.py`
   - Output stored as `shadow_hand.urdf` and processed versions

2. **MuJoCo Simulation**
   - MuJoCo model configuration in `model/mjmodel.xml`
   - Interactive control through `interactive_joint_control.py`
   - Simulation data collected and stored in `data/` directory

3. **Data Collection and Organization**
   - Joint angle data stored in `data/data_angles.csv`
   - Separate directories for different robot components (arm, palm, wrist)
   - Data extraction utilities in `extract_angle_indirect(in Process).py`

4. **QPOS Data Extraction**
   - Uses `extract_angle_indirect(in Process).py` to process simulation data
   - Extracts 29 joint position values from MJDATA.TXT files
   - Organizes data by category and run into CSV format
   - Outputs to `data/data_angles.csv` with columns:
     - `category`: Component name (arm, palm, wrist)
     - `run`: Run identifier (T0, T1, etc.)
     - 29 joint position columns (e.g., `ra_shoulder_pan_joint`, `rh_WRJ2`, etc.)

## Key Components

### Model Files
- `shadow_hand.urdf`: Base URDF model
- `shadow_hand_processed.urdf`: Processed version for MuJoCo
- Various `.stl` files: Mesh models for robot components

### Python Scripts
- `urdf_editor.py`: Main utility for URDF processing
- `interactive_joint_control.py`: MuJoCo simulation interface
- `extract_angle_indirect(in Process).py`: QPOS data extraction
- `extract_mesh.py`: Mesh file processing
- `utils.py`: Common utility functions

### Data Organization
- Structured data storage in `data/` directory
- Component-specific subdirectories (arm, palm, wrist)
- Run-specific folders (T0, T1, etc.) containing MJDATA.TXT
- CSV format for joint angle data in `data_angles.csv`
- Raw MuJoCo model data in MJMODEL.TXT

## Dependencies
Project dependencies are managed through `pyproject.toml`. Key requirements include:
- MuJoCo
- ROS (for XACRO processing)
- Python 3.x
- pandas
- numpy
- Additional dependencies listed in pyproject.toml

## Notes
- Backup files are maintained for critical model files
- Mesh files are stored in both .dae and .stl formats
- QPOS extraction script will overwrite existing `data_angles.csv`
- Script verifies exactly 29 QPOS values in each MJDATA.TXT file
- Progress is reported during data extraction
