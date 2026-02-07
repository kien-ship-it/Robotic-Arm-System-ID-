# Pull Request for Upstream: Fix broken install, harden XML parsing, improve CLI validation and efficiency

## Summary

This PR addresses several critical issues and improvements to the robot-sysid project:

1. **Critical bug fix** — Fixes broken `pip install` due to misconfigured `pyproject.toml`
2. **Security hardening** — Replaces unsafe XML parsing with `defusedxml`
3. **CLI usability** — Adds `--version` and `--verbose` flags plus comprehensive input validation
4. **Performance optimization** — Pre-allocates arrays in hot simulation loop
5. **Better logging** — Replaces `print()` statements with proper logging module
6. **Test coverage** — Adds 9 new CLI validation tests

## Detailed Changes

### 1. Critical: Fix broken `pyproject.toml`

**Problem:** The `dependencies` array was incorrectly placed after `[project.urls]`, causing TOML to interpret it as `project.urls.dependencies` (which expects a string URL) instead of `project.dependencies` (which expects an array). This completely broke `pip install`.

**Solution:** Moved `dependencies` before `[project.urls]` section.

**Files changed:**
- `pyproject.toml`

**Before:**
```toml
[project.urls]
Homepage = "..."
Repository = "..."
Issues = "..."

dependencies = [
    "numpy",
    ...
]
```

**After:**
```toml
dependencies = [
    "numpy",
    ...
]

[project.urls]
Homepage = "..."
```

### 2. Security: Use `defusedxml` for XML parsing

**Problem:** Standard library `xml.etree.ElementTree.parse()` is vulnerable to XML External Entity (XXE) injection attacks when parsing untrusted XML files.

**Solution:** Replaced `ET.parse()` with `defusedxml.ElementTree.parse()` in `parser.py` and `export.py`. The standard `ET` module is still used for `SubElement`, `Comment`, `indent`, and type hints (which are safe operations).

**Files changed:**
- `pyproject.toml` — Added `defusedxml` dependency
- `robot_sysid/parser.py` — Replaced `ET.parse()` with `SafeET.parse()`
- `robot_sysid/export.py` — Replaced `ET.parse()` with `SafeET.parse()`

**Import pattern:**
```python
import xml.etree.ElementTree as ET  # Still used for safe operations
import defusedxml.ElementTree as SafeET  # Used for parsing

# Parsing (unsafe) → use defusedxml
tree = SafeET.parse(xml_path)

# Construction (safe) → use standard library
element = ET.SubElement(parent, 'tag')
```

### 3. CLI usability improvements

**New flags:**
- `--version` — Display version number and exit
- `--verbose` / `-v` — Enable debug-level logging output

**Input validation:**
- File extension must be `.xml` or `.mjcf`
- `--duration` must be positive
- `--sample-rate` must be positive
- `--export-damiao` requires at least one `--motor-type`

**Files changed:**
- `robot_sysid/cli.py` — Added flags and validation logic
- `robot_sysid/__init__.py` — Made `__version__` available for `--version` flag
- `tests/test_cli.py` — New test file with 9 validation tests

**Example validation error:**
```bash
$ robot-sysid model.txt
error: Expected an XML model file (.xml or .mjcf), got '.txt'
```

### 4. Efficiency: Pre-allocate arrays in simulator

**Problem:** The simulator was building regressor matrix `Y` and torque vector `tau` by appending to Python lists in a loop, then converting to NumPy arrays at the end. This pattern is O(n) in append operations + O(n) in conversion = 2n operations.

**Solution:** Pre-allocate NumPy arrays once at the correct size, then fill them in-place.

**Files changed:**
- `robot_sysid/simulator.py`

**Before:**
```python
Y_list = []
tau_list = []
for i in range(n_samples):
    Y_list.append(Y_row)
    tau_list.append(tau_mj)
Y = np.array(Y_list)
tau = np.array(tau_list)
```

**After:**
```python
n_cols = 12 if include_friction else 10
Y = np.zeros((n_samples, n_cols))
tau = np.zeros(n_samples)
for i in range(n_samples):
    Y[i, :] = Y_row
    tau[i] = tau_mj
```

**Performance impact:** Reduces memory allocations from O(n) to O(1) and eliminates list-to-array conversion overhead.

### 5. Logging: Replace print() with logging module

**Problem:** The code used bare `print()` statements for progress and status messages, making it impossible to control verbosity or integrate with logging frameworks.

**Solution:** 
- Imported `logging` module in affected files
- Created module-level `logger` instances
- Replaced `print()` calls with appropriate `logger.info()` or `logger.debug()` calls
- Configured logging in CLI with `--verbose` flag support

**Files changed:**
- `robot_sysid/cli.py` — Configure `logging.basicConfig()` based on `--verbose` flag
- `robot_sysid/regressor.py` — Replace `print()` with `logger.info()`
- `robot_sysid/simulator.py` — Replace `print()` with `logger.info()`
- `robot_sysid/parser.py` — Add `logger` (for future use)
- `robot_sysid/export.py` — Add `logger` (for future use)

**Example:**
```python
# Before
print("Generating symbolic regressor (one-time cost)...")

# After
logger.info("Generating symbolic regressor (one-time cost)...")
```

### 6. Tests: Add CLI validation tests

Added comprehensive test coverage for CLI argument validation.

**New test file:** `tests/test_cli.py` (70 lines)

**Test cases:**
1. `test_version_flag` — Verify `--version` displays version
2. `test_rejects_non_xml_file` — Reject `.txt` files
3. `test_accepts_xml_extension` — Accept `.xml` files
4. `test_accepts_mjcf_extension` — Accept `.mjcf` files
5. `test_rejects_zero_duration` — Reject `--duration 0`
6. `test_rejects_negative_duration` — Reject `--duration -5`
7. `test_rejects_zero_sample_rate` — Reject `--sample-rate 0`
8. `test_rejects_negative_sample_rate` — Reject `--sample-rate -100`
9. `test_damiao_requires_motor_type` — Require `--motor-type` with `--export-damiao`

**All tests pass:**
```bash
$ pytest tests/test_cli.py -v
======================== 9 passed in 0.42s ========================
```

## Testing

All existing tests continue to pass. The new CLI validation tests have been added to prevent regressions.

```bash
# Run all tests
$ pytest tests/
======================== 29 passed in 2.31s ========================

# Run only new CLI tests
$ pytest tests/test_cli.py -v
======================== 9 passed in 0.42s ========================
```

## Breaking Changes

None. All changes are backward compatible.

## Checklist

- [x] Fixed critical `pyproject.toml` bug preventing installation
- [x] Added `defusedxml` for secure XML parsing
- [x] Added `--version` and `--verbose` CLI flags
- [x] Added comprehensive CLI input validation
- [x] Optimized array allocation in simulator
- [x] Replaced `print()` with `logging` module
- [x] Added 9 new CLI validation tests
- [x] All tests pass (29 total)
- [x] No breaking changes
