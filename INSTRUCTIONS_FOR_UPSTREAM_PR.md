# Instructions for Submitting PR to Upstream Repository

This document provides step-by-step instructions for submitting the improvements from this fork to the original upstream repository at `https://github.com/curryabalone/Robotic-Arm-System-ID-`.

## Summary of Changes

The changes from PR #1 in this repository include critical bug fixes, security improvements, and usability enhancements that should be contributed back to the upstream repository. See [`UPSTREAM_PR.md`](./UPSTREAM_PR.md) for the full PR description.

## Prerequisites

Before you can submit a PR to the upstream repository, you need:

1. A GitHub account with access to create pull requests
2. Git installed on your local machine
3. The changes from this repository that need to be submitted

## Option 1: Submit PR via GitHub Web Interface (Easiest)

If you are the owner of this fork, you can submit a PR directly from GitHub:

1. **Navigate to your fork on GitHub:**
   - Go to `https://github.com/kien-ship-it/Robotic-Arm-System-ID-`

2. **Click "Contribute" button:**
   - Above the file list, you should see a "Contribute" button
   - Click it and select "Open pull request"

3. **Configure the PR:**
   - Base repository: `curryabalone/Robotic-Arm-System-ID-`
   - Base branch: `main`
   - Head repository: `kien-ship-it/Robotic-Arm-System-ID-`
   - Compare branch: Choose the branch containing the changes (e.g., `main` or a feature branch)

4. **Use the PR template:**
   - Copy the content from [`UPSTREAM_PR.md`](./UPSTREAM_PR.md)
   - Paste it as your PR description
   - Adjust the title to: "Fix broken install, harden XML parsing, improve CLI validation and efficiency"

5. **Submit the PR:**
   - Review your changes
   - Click "Create pull request"

## Option 2: Submit PR via Command Line (More Control)

If you need more control or prefer the command line:

### Step 1: Set up your local repository

```bash
# Clone the upstream repository
git clone https://github.com/curryabalone/Robotic-Arm-System-ID-.git upstream-repo
cd upstream-repo

# Add your fork as a remote
git remote add fork https://github.com/kien-ship-it/Robotic-Arm-System-ID-.git

# Fetch your fork's branches
git fetch fork
```

### Step 2: Create a feature branch

```bash
# Create a branch from upstream's main branch
git checkout -b improvements-from-kien-fork upstream/main

# Cherry-pick the commits from your fork
# (You'll need to identify the commit SHAs from your fork)
git cherry-pick <commit-sha-from-fork>

# Or merge the changes from your fork
git merge fork/main
```

### Step 3: Push the branch to your fork

```bash
# Push to your fork (not upstream, since you likely don't have write access)
git push fork improvements-from-kien-fork
```

### Step 4: Create the PR via GitHub CLI or Web Interface

**Using GitHub CLI (gh):**

```bash
gh pr create \
  --repo curryabalone/Robotic-Arm-System-ID- \
  --head kien-ship-it:improvements-from-kien-fork \
  --title "Fix broken install, harden XML parsing, improve CLI validation and efficiency" \
  --body-file UPSTREAM_PR.md
```

**Or via web interface:**
1. Go to `https://github.com/curryabalone/Robotic-Arm-System-ID-`
2. You should see a banner saying "Compare & pull request" for your recently pushed branch
3. Click it and follow the steps from Option 1

## Option 3: Creating a Patch File (If No Direct Access)

If you cannot create a PR directly (e.g., if the upstream repo doesn't accept PRs from forks), you can create a patch file:

```bash
# From this repository
cd /home/runner/work/Robotic-Arm-System-ID-/Robotic-Arm-System-ID-

# Create a patch file containing all the changes
git format-patch <base-commit>..<head-commit> --stdout > upstream-improvements.patch

# Or, to include all commits from PR #1:
git format-patch <commit-before-PR-1>..HEAD --stdout > upstream-improvements.patch
```

Then:
1. Email the patch file to the upstream maintainer
2. Or create an issue in the upstream repository and attach the patch file
3. Or share it via another communication channel

## Specific Commits to Include

The changes that should be included in the upstream PR come from the merged PR #1. The key changes are:

### Files Modified:
1. `pyproject.toml` — Fix dependencies order, add defusedxml
2. `robot_sysid/cli.py` — Add --version, --verbose, input validation
3. `robot_sysid/parser.py` — Use defusedxml for safe parsing
4. `robot_sysid/export.py` — Use defusedxml for safe parsing
5. `robot_sysid/regressor.py` — Replace print() with logging
6. `robot_sysid/simulator.py` — Pre-allocate arrays, use logging

### Files Added:
7. `tests/test_cli.py` — 9 new CLI validation tests

### Files Modified (Version):
8. `robot_sysid/__init__.py` — Expose __version__ for CLI

## After Submitting the PR

1. **Monitor the PR:**
   - Respond to any feedback from the upstream maintainers
   - Make requested changes if needed

2. **Be Patient:**
   - Upstream maintainers may take time to review
   - They may request changes or have questions

3. **Be Respectful:**
   - Follow the upstream project's contribution guidelines
   - Be professional and courteous in all interactions

## What to Include in the PR

- Use the title: **"Fix broken install, harden XML parsing, improve CLI validation and efficiency"**
- Use the body from [`UPSTREAM_PR.md`](./UPSTREAM_PR.md)
- Reference any relevant issues in the upstream repository (if applicable)
- Mention that this has been tested in your fork with 29 passing tests

## Troubleshooting

### "You don't have permission to create a PR"
- Make sure you're creating a PR from your fork to the upstream repo
- Ensure the upstream repository accepts pull requests from forks
- Check if you need to sign a CLA (Contributor License Agreement)

### "Conflicts detected"
- The upstream repository may have changed since you forked
- You'll need to rebase your changes on top of the latest upstream main branch:
  ```bash
  git fetch upstream
  git rebase upstream/main
  ```

### "Commits missing or out of order"
- Make sure you're including all the commits from PR #1
- Consider creating a clean branch with just the necessary commits

## Need Help?

If you encounter issues submitting the PR:
1. Check the upstream repository's CONTRIBUTING.md (if it exists)
2. Open an issue in the upstream repository asking for guidance
3. Review GitHub's documentation on creating pull requests

---

**Remember:** The detailed PR description is in [`UPSTREAM_PR.md`](./UPSTREAM_PR.md). Use that as your PR body when submitting to upstream.
