from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, CMakeDeps, cmake_layout


class ShipInTransitFmusConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.requires("fmicppcore/0.5@fmicpp/stable")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["BUILD_TESTING"] = 0
        tc.variables["PY_PLOT"] = 0
        tc.variables["INCL_SOURCE"] = 0
        tc.variables["DOX_DOC"] = 0
        tc.variables["STAND_ALONE"] = 0
        tc.variables["FMU_ASSEMBLY_FOLDER"] = "tmp"
        tc.generate()
        CMakeDeps(self).generate()

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build(target="install")
