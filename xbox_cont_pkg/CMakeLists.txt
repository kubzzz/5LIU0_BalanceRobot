cmake_minimum_required(VERSION "3.22.1")
project("xbox_cont")


find_package(PkgConfig REQUIRED)
find_package(fmt REQUIRED)

pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

add_executable("xbox_cont"
    "src/main.cpp"
)

target_link_libraries(xbox_cont PRIVATE fmt::fmt)
target_link_libraries(xbox_cont PRIVATE PkgConfig::libusb)