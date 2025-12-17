#!/usr/bin/env python3
"""
NucleusESP32 Development Environment Bootstrap Script

This script sets up the development environment using SpareTools.
It provides a one-command setup for all development dependencies.

Usage:
    python scripts/bootstrap.py

Requirements:
    - Python 3.8+ (will be upgraded to 3.12.7 by SpareTools)
    - Internet connection for downloading dependencies
    - Administrative privileges may be required for some operations
"""

import os
import sys
import subprocess
import shutil
import platform
from pathlib import Path
from typing import Optional, Dict, Any


class Bootstrapper:
    """Handles the bootstrap process for NucleusESP32 development environment."""

    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.system = platform.system().lower()
        self.is_windows = self.system == "windows"
        self.is_linux = self.system == "linux"
        self.is_macos = self.system == "darwin"

        # SpareTools configuration
        self.sparetools_url = "https://github.com/sparesparrow/SpareTools.git"
        self.sparetools_dir = self.project_root / ".sparetools"

    def run_command(self, cmd: list, cwd: Optional[Path] = None,
                   check: bool = True, capture_output: bool = False) -> subprocess.CompletedProcess:
        """Run a command and return the result."""
        try:
            result = subprocess.run(
                cmd,
                cwd=cwd or self.project_root,
                check=check,
                capture_output=capture_output,
                text=True
            )
            return result
        except subprocess.CalledProcessError as e:
            print(f"Command failed: {' '.join(cmd)}")
            print(f"Error: {e}")
            if e.stdout:
                print(f"stdout: {e.stdout}")
            if e.stderr:
                print(f"stderr: {e.stderr}")
            raise

    def check_prerequisites(self) -> bool:
        """Check if all prerequisites are met."""
        print("🔍 Checking prerequisites...")

        # Check Python version
        python_version = sys.version_info
        if python_version < (3, 8):
            print(f"❌ Python {python_version.major}.{python_version.minor} is not supported.")
            print("   Please upgrade to Python 3.8 or later.")
            return False

        print(f"✅ Python {python_version.major}.{python_version.minor}.{python_version.micro} detected")

        # Check git availability
        try:
            self.run_command(["git", "--version"], capture_output=True)
            print("✅ Git is available")
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("❌ Git is not available. Please install Git.")
            return False

        # Check internet connectivity (basic check)
        try:
            self.run_command(["curl", "-s", "--head", "https://github.com"],
                           capture_output=True, check=False)
            print("✅ Internet connectivity detected")
        except (subprocess.CalledProcessError, FileNotFoundError):
            try:
                self.run_command(["ping", "-c", "1", "github.com"],
                               capture_output=True, check=False)
                print("✅ Internet connectivity detected")
            except (subprocess.CalledProcessError, FileNotFoundError):
                print("⚠️  Internet connectivity could not be verified")

        return True

    def setup_sparetools(self) -> bool:
        """Clone and set up SpareTools."""
        print("\n🔧 Setting up SpareTools...")

        if self.sparetools_dir.exists():
            print(f"   SpareTools directory already exists at {self.sparetools_dir}")
            print("   Updating SpareTools...")
            try:
                self.run_command(["git", "pull"], cwd=self.sparetools_dir)
                print("✅ SpareTools updated")
            except subprocess.CalledProcessError:
                print("⚠️  Could not update SpareTools, proceeding with existing version")
        else:
            print(f"   Cloning SpareTools to {self.sparetools_dir}")
            try:
                self.run_command(["git", "clone", self.sparetools_url, str(self.sparetools_dir)])
                print("✅ SpareTools cloned")
            except subprocess.CalledProcessError:
                print("❌ Failed to clone SpareTools")
                return False

        # Run SpareTools bootstrap
        bootstrap_script = self.sparetools_dir / "bootstrap.py"
        if not bootstrap_script.exists():
            print("❌ SpareTools bootstrap script not found")
            return False

        print("   Running SpareTools bootstrap...")
        try:
            self.run_command([sys.executable, str(bootstrap_script)])
            print("✅ SpareTools bootstrap completed")
        except subprocess.CalledProcessError:
            print("❌ SpareTools bootstrap failed")
            return False

        return True

    def install_dependencies(self) -> bool:
        """Install project dependencies using Conan and pip."""
        print("\n📦 Installing dependencies...")

        # Install Python dependencies
        requirements_file = self.project_root / "requirements-dev.txt"
        if requirements_file.exists():
            print("   Installing Python dependencies...")
            try:
                # Use pip from SpareTools bundled Python if available
                pip_cmd = [sys.executable, "-m", "pip"]
                self.run_command(pip_cmd + ["install", "-r", str(requirements_file)])
                print("✅ Python dependencies installed")
            except subprocess.CalledProcessError:
                print("❌ Failed to install Python dependencies")
                return False
        else:
            print("⚠️  requirements-dev.txt not found, skipping Python dependencies")

        # Install Conan dependencies
        conanfile = self.project_root / "conanfile.py"
        if conanfile.exists():
            print("   Installing Conan dependencies...")
            try:
                self.run_command(["conan", "install", ".", "--build=missing"])
                print("✅ Conan dependencies installed")
            except subprocess.CalledProcessError:
                print("❌ Failed to install Conan dependencies")
                return False
        else:
            print("⚠️  conanfile.py not found, skipping Conan dependencies")

        return True

    def setup_precommit_hooks(self) -> bool:
        """Set up pre-commit hooks."""
        print("\n🔗 Setting up pre-commit hooks...")

        try:
            self.run_command(["pre-commit", "install"])
            print("✅ Pre-commit hooks installed")
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("⚠️  Pre-commit not available, skipping hook installation")
            return True  # Not critical

        return True

    def setup_platformio(self) -> bool:
        """Set up PlatformIO for ESP32 development."""
        print("\n🔌 Setting up PlatformIO...")

        try:
            # Check if PlatformIO is available
            self.run_command(["pio", "--version"], capture_output=True)
            print("✅ PlatformIO is available")

            # Install ESP32 platform if not already installed
            try:
                self.run_command(["pio", "platform", "show", "espressif32"], capture_output=True)
                print("✅ ESP32 platform already installed")
            except subprocess.CalledProcessError:
                print("   Installing ESP32 platform...")
                self.run_command(["pio", "platform", "install", "espressif32"])
                print("✅ ESP32 platform installed")

        except (subprocess.CalledProcessError, FileNotFoundError):
            print("❌ PlatformIO setup failed")
            print("   Please ensure PlatformIO is installed and available in PATH")
            return False

        return True

    def create_env_file(self) -> bool:
        """Create environment configuration file."""
        print("\n📝 Creating environment configuration...")

        env_file = self.project_root / ".env"
        if env_file.exists():
            print("   .env file already exists, skipping")
            return True

        env_content = f"""# NucleusESP32 Development Environment
# Generated by bootstrap script

# Project paths
PROJECT_ROOT={self.project_root}
SPARETOOLS_DIR={self.sparetools_dir}

# Python configuration
PYTHONPATH={self.project_root}/src:{self.project_root}/test

# ESP32 configuration
PLATFORMIO_CORE_DIR={self.project_root}/.platformio

# Build configuration
BUILD_TYPE=Debug
CMAKE_BUILD_TYPE=Debug

# Testing configuration
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
"""

        try:
            with open(env_file, 'w') as f:
                f.write(env_content)
            print("✅ Environment configuration created")
        except IOError as e:
            print(f"❌ Failed to create .env file: {e}")
            return False

        return True

    def run_post_bootstrap_checks(self) -> bool:
        """Run checks to verify the bootstrap was successful."""
        print("\n✅ Running post-bootstrap checks...")

        checks_passed = 0
        total_checks = 0

        # Check Python version
        total_checks += 1
        python_version = sys.version_info
        if python_version >= (3, 12):
            print("✅ Python 3.12+ available")
            checks_passed += 1
        else:
            print(f"⚠️  Python {python_version.major}.{python_version.minor} detected (3.12+ recommended)")

        # Check Conan
        total_checks += 1
        try:
            result = self.run_command(["conan", "--version"], capture_output=True)
            print("✅ Conan available")
            checks_passed += 1
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("❌ Conan not available")

        # Check PlatformIO
        total_checks += 1
        try:
            result = self.run_command(["pio", "--version"], capture_output=True)
            print("✅ PlatformIO available")
            checks_passed += 1
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("❌ PlatformIO not available")

        # Check pytest
        total_checks += 1
        try:
            result = self.run_command(["python", "-m", "pytest", "--version"], capture_output=True)
            print("✅ pytest available")
            checks_passed += 1
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("❌ pytest not available")

        print(f"\n📊 Bootstrap checks: {checks_passed}/{total_checks} passed")

        if checks_passed == total_checks:
            print("🎉 Bootstrap completed successfully!")
            return True
        else:
            print("⚠️  Bootstrap completed with warnings. Some tools may not be available.")
            return True  # Don't fail the bootstrap for warnings

    def run(self) -> int:
        """Run the complete bootstrap process."""
        print("🚀 NucleusESP32 Development Environment Bootstrap")
        print("=" * 50)

        try:
            # Step 1: Check prerequisites
            if not self.check_prerequisites():
                return 1

            # Step 2: Set up SpareTools
            if not self.setup_sparetools():
                return 1

            # Step 3: Install dependencies
            if not self.install_dependencies():
                return 1

            # Step 4: Set up PlatformIO
            if not self.setup_platformio():
                return 1

            # Step 5: Set up pre-commit hooks
            if not self.setup_precommit_hooks():
                return 1

            # Step 6: Create environment file
            if not self.create_env_file():
                return 1

            # Step 7: Run post-bootstrap checks
            if not self.run_post_bootstrap_checks():
                return 1

            print("\n🎯 Next steps:")
            print("   1. Source the environment: source .env")
            print("   2. Build the project: pio run")
            print("   3. Run tests: pytest")
            print("   4. Start development!")

            return 0

        except KeyboardInterrupt:
            print("\n\n⚠️  Bootstrap interrupted by user")
            return 1
        except Exception as e:
            print(f"\n❌ Bootstrap failed with unexpected error: {e}")
            import traceback
            traceback.print_exc()
            return 1


def main():
    """Main entry point."""
    bootstrapper = Bootstrapper()
    return bootstrapper.run()


if __name__ == "__main__":
    sys.exit(main())