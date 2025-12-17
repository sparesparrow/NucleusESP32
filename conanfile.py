from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake
from conan.tools.layout import basic_layout


class NucleusESP32Conan(ConanFile):
    name = "nucleus-esp32"
    version = "0.1.0"
    package_type = "application"

    # Optional metadata
    license = "MIT"
    author = "NucleusESP32 Team"
    url = "https://github.com/sparesparrow/NucleusESP32"
    description = "ESP32-based multi-tool device firmware"
    topics = ("esp32", "embedded", "rf", "nfc", "iot")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    # Sources are located in the same place as this recipe
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "test/*", "platformio.ini"

    def layout(self):
        basic_layout(self, src_folder=".")

    def generate(self):
        # Generate CMake toolchain and dependencies for cross-compilation
        tc = CMakeToolchain(self)
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def build_requirements(self):
        # PlatformIO for ESP32 builds
        self.tool_requires("platformio/6.1.11")

        # Development tools from SpareTools
        self.tool_requires("cpython/3.12.7")
        self.tool_requires("shared-dev-tools/1.0.0")

    def requirements(self):
        # Testing framework
        self.test_requires("gtest/1.14.0")

        # Mock libraries for testing
        self.test_requires("mock/1.0.0")

    def package_info(self):
        # Define components for different modules
        self.cpp_info.components["rf_module"].libs = ["rf"]
        self.cpp_info.components["nfc_module"].libs = ["nfc"]
        self.cpp_info.components["ir_module"].libs = ["ir"]

        # Define executable
        self.cpp_info.exes = ["nucleus-esp32"]