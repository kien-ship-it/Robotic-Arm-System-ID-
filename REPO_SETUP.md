# Repository Setup Guide

This document explains how to set up and publish the `robot-sysid` repository to GitHub.

## Repository Structure

The repository is organized as follows:

```
robot-sysid/
├── robot_sysid/          # Main package source code
│   ├── __init__.py       # Package initialization
│   ├── cli.py            # Command-line interface
│   ├── parser.py         # MuJoCo XML model parser
│   ├── trajectory.py     # Trajectory generation
│   ├── regressor.py      # Regressor computation
│   ├── simulator.py      # MuJoCo simulation
│   ├── solver.py         # Parameter identification
│   └── export.py         # Result exporters
├── examples/
│   └── kinova/           # Kinova arm reference example
│       ├── model/        # MuJoCo XML models and meshes
│       ├── validate_*.py # Validation scripts
│       └── README.md     # Example documentation
├── tests/                # Test suite
│   ├── test_parser.py
│   ├── test_trajectory.py
│   ├── test_solver.py
│   └── test_solver_integration.py
├── pyproject.toml        # Package metadata and dependencies
├── README.md             # Main documentation
├── LICENSE               # MIT License
├── MANIFEST.in           # Package manifest
├── .gitignore            # Git ignore rules
└── .gitattributes        # Git attributes

```

## Files Included in the Repository

### Core Package Files
- `robot_sysid/` - All Python modules for the system identification tool
- `pyproject.toml` - Package configuration with dependencies and entry points
- `README.md` - Comprehensive documentation
- `LICENSE` - MIT License (Copyright 2024 Yuze Cai)

### Examples
- `examples/kinova/` - Complete Kinova arm example with:
  - MuJoCo XML models (base, ground truth, guessed parameters)
  - STL mesh files for visualization
  - Validation scripts demonstrating the pipeline
  - Detailed README with usage examples

### Tests
- `tests/` - Unit tests and integration tests (22 tests total)
- All tests pass with pytest

### Configuration Files
- `.gitignore` - Excludes `.kiro/`, `__pycache__/`, build artifacts, etc.
- `.gitattributes` - Git line ending configuration
- `MANIFEST.in` - Ensures examples and tests are included in distributions

## Files Excluded from Repository

The following are excluded via `.gitignore`:
- `.kiro/` - Kiro IDE specification files
- `Ragtime Arm/` - Unrelated project (not part of robot-sysid)
- `kinova/` - Original development files (replaced by `examples/kinova/`)
- `__pycache__/`, `*.pyc` - Python bytecode
- `.DS_Store` - macOS system files
- `sysid_output/` - Generated output directories
- `verify_modules.py` - Temporary test script
- Build artifacts: `dist/`, `build/`, `*.egg-info/`

## Publishing to GitHub

### 1. Create a New Repository on GitHub

1. Go to https://github.com/new
2. Repository name: `robot-sysid`
3. Description: "A generic robotic arm system identification tool for identifying inertial parameters and friction from MuJoCo simulation data"
4. Choose: Public
5. **Do NOT** initialize with README, .gitignore, or license (we already have these)
6. Click "Create repository"

### 2. Initialize Git and Push (if starting fresh)

If you haven't initialized git yet:

```bash
cd /path/to/robot-sysid
git init
git add .
git commit -m "Initial commit: robot-sysid v0.1.0"
git branch -M main
git remote add origin https://github.com/YuzeCai/robot-sysid.git
git push -u origin main
```

### 3. If Git is Already Initialized

If you already have a git repository:

```bash
# Remove files that should be ignored
git rm -r --cached .kiro/
git rm -r --cached "Ragtime Arm/"
git rm -r --cached kinova/
git rm --cached verify_modules.py
git rm -r --cached __pycache__/
git rm --cached .DS_Store

# Add all new files
git add .
git commit -m "Prepare repository for publication: robot-sysid v0.1.0"

# Set remote and push
git remote add origin https://github.com/YuzeCai/robot-sysid.git
git push -u origin main
```

### 4. Create a Release (Optional but Recommended)

1. Go to your repository on GitHub
2. Click "Releases" → "Create a new release"
3. Tag version: `v0.1.0`
4. Release title: `robot-sysid v0.1.0`
5. Description:
   ```
   Initial release of robot-sysid, a generic robotic arm system identification tool.
   
   Features:
   - Robot-agnostic system identification for terminal link inertial parameters
   - Automatic terminal joint detection
   - Multi-frequency sinusoidal trajectory generation
   - Symbolic regressor derivation via SymPy
   - Friction identification (Coulomb + viscous)
   - Export to JSON, updated MuJoCo XML, and Damiao motor CSV
   - Complete Kinova arm example
   - 22 passing unit and integration tests
   ```
6. Click "Publish release"

## Repository Settings (Recommended)

### Topics/Tags
Add these topics to help people find your repository:
- `robotics`
- `system-identification`
- `mujoco`
- `dynamics`
- `parameter-estimation`
- `robot-arm`
- `python`

### About Section
- Description: "A generic robotic arm system identification tool for identifying inertial parameters and friction from MuJoCo simulation data"
- Website: (leave blank or add documentation site later)
- Topics: (add the tags above)

### Branch Protection (Optional)
If you plan to have contributors:
1. Go to Settings → Branches
2. Add rule for `main` branch
3. Enable "Require pull request reviews before merging"

## Installation Verification

After publishing, verify the installation works:

```bash
# Clone the repository
git clone https://github.com/YuzeCai/robot-sysid.git
cd robot-sysid

# Install
pip install -e .

# Run the example
robot-sysid examples/kinova/model/kinova_fullinertia_guess.xml

# Run tests
pytest tests/
```

## Publishing to PyPI (Future)

When ready to publish to PyPI:

```bash
# Install build tools
pip install build twine

# Build distribution
python -m build

# Upload to PyPI (requires PyPI account)
twine upload dist/*
```

Then users can install with:
```bash
pip install robot-sysid
```

## Maintenance

### Version Updates
Update version in `pyproject.toml` and `robot_sysid/__init__.py` for new releases.

### Documentation Updates
Keep `README.md` and `examples/kinova/README.md` up to date with new features.

### Testing
Run tests before each release:
```bash
pytest tests/ -v
```

## Support

For issues or questions, users can:
- Open an issue on GitHub: https://github.com/YuzeCai/robot-sysid/issues
- Check the documentation in README.md
- Review the Kinova example in examples/kinova/

## License

MIT License - See LICENSE file for details.
Copyright (c) 2024 Yuze Cai
