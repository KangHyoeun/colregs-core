#!/bin/bash

# COLREGs-Core Integration Test Runner
# Run all integration tests with proper environment setup

set -e  # Exit on error

echo "=========================================="
echo "🧪 COLREGs-Core Integration Tests"
echo "=========================================="
echo ""

# Check if conda environment is activated
if [[ "$CONDA_DEFAULT_ENV" != "DRL-otter-nav" ]]; then
    echo "⚠️  Warning: DRL-otter-nav environment not activated"
    echo "   Activating environment..."
    eval "$(conda shell.bash hook)"
    conda activate DRL-otter-nav
fi

echo "✅ Environment: $CONDA_DEFAULT_ENV"
echo ""

# Check if poetry is available
if ! command -v poetry &> /dev/null; then
    echo "❌ Error: poetry not found"
    echo "   Please install poetry first"
    exit 1
fi

echo "✅ Poetry found"
echo ""

# Navigate to colregs-core directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
COLREGS_DIR="$(dirname "$SCRIPT_DIR")"

cd "$COLREGS_DIR"
echo "📁 Working directory: $(pwd)"
echo ""

# Run integration tests
echo "🚀 Running integration tests..."
echo "=========================================="
echo ""

poetry run python integration_tests/test_colregs_integration.py

# Check exit code
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✅ All tests passed!"
    echo "=========================================="
    echo ""
    echo "📝 Next steps:"
    echo "  1. Review the calculation outputs above"
    echo "  2. Verify CR values are reasonable"
    echo "  3. Check reward function behavior"
    echo "  4. Start DRL training if everything looks good"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "❌ Tests failed!"
    echo "=========================================="
    echo ""
    echo "🐛 Troubleshooting:"
    echo "  1. Check error messages above"
    echo "  2. Verify all dependencies are installed"
    echo "  3. Ensure irsim and PVS paths are correct"
    echo ""
    exit 1
fi
