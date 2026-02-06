#!/bin/bash

# Script to commit robot-sysid to GitHub
# Current repo: https://github.com/curryabalone/7dof.git
# Target repo: https://github.com/curryabalone/Robotic-Arm-System-ID.git

echo "🚀 Preparing to commit robot-sysid to GitHub..."
echo ""

# Check if we're in the right directory
if [ ! -f "pyproject.toml" ]; then
    echo "❌ Error: pyproject.toml not found. Are you in the right directory?"
    exit 1
fi

# Check current remote
CURRENT_REMOTE=$(git remote get-url origin 2>/dev/null)
echo "📍 Current remote: $CURRENT_REMOTE"
echo ""

if [[ "$CURRENT_REMOTE" == *"7dof"* ]]; then
    echo "⚠️  Current remote is still pointing to '7dof' repository"
    echo ""
    echo "📝 To use this as the robot-sysid repository, you have two options:"
    echo ""
    echo "Option 1 (Recommended): Rename the repository on GitHub"
    echo "   1. Go to: https://github.com/curryabalone/7dof/settings"
    echo "   2. Change repository name from '7dof' to 'Robotic-Arm-System-ID'"
    echo "   3. Run: git remote set-url origin https://github.com/curryabalone/Robotic-Arm-System-ID.git"
    echo "   4. Run this script again"
    echo ""
    echo "Option 2: Create a new repository"
    echo "   1. Create new repo at: https://github.com/new"
    echo "   2. Name it: Robotic-Arm-System-ID"
    echo "   3. Run: git remote set-url origin https://github.com/curryabalone/Robotic-Arm-System-ID.git"
    echo "   4. Run this script again"
    echo ""
    echo "See REPURPOSE_REPO.md for detailed instructions"
    echo ""
    read -p "Have you renamed/created the repository and updated the remote? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "❌ Exiting. Please update the repository first."
        exit 1
    fi
    # Refresh remote URL
    CURRENT_REMOTE=$(git remote get-url origin 2>/dev/null)
    echo "✅ Updated remote: $CURRENT_REMOTE"
    echo ""
fi

echo "📋 Current git status:"
git status --short | head -20
echo ""

# Add all files (respects .gitignore)
echo "➕ Adding files (respecting .gitignore)..."
git add .

echo ""
echo "📝 Files to be committed:"
git status --short
echo ""

# Show what's being excluded
echo "🚫 Excluded from repository (via .gitignore):"
echo "   - .kiro/ (Kiro IDE files)"
echo "   - kinova/ (original development code - stays private)"
echo "   - Ragtime Arm/ (unrelated project)"
echo "   - __pycache__/, *.pyc (Python bytecode)"
echo "   - Build artifacts"
echo ""

# Commit
echo "💾 Creating commit..."
git commit -m "Transform repository into robot-sysid package

- Add robot_sysid/ package with 7 modules
- Add examples/kinova/ with complete working example  
- Add comprehensive test suite (22 tests)
- Add full documentation and guides
- Exclude private development files (kinova/, .kiro/, Ragtime Arm/)
- Package ready for pip installation

Features:
- Robot-agnostic system identification tool
- Automatic terminal joint detection
- Multi-frequency trajectory generation
- Symbolic regressor derivation via SymPy
- Friction identification (Coulomb + viscous)
- Export to JSON, updated XML, and Damiao CSV
- Complete Kinova arm example
- 22 passing unit and integration tests"

echo ""
echo "✅ Commit created!"
echo ""
echo "📤 Ready to push to GitHub!"
echo ""
echo "Run the following command to push:"
echo "   git push origin main"
echo ""
echo "Or run this script with 'push' argument:"
echo "   ./commit_to_github.sh push"
echo ""

# If 'push' argument provided, push to GitHub
if [ "$1" == "push" ]; then
    echo "🚀 Pushing to GitHub..."
    git push origin main
    echo ""
    echo "✅ Successfully pushed to GitHub!"
    echo "🌐 View your repository at: $(git remote get-url origin | sed 's/\.git$//')"
fi
