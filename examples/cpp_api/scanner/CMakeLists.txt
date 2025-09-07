cmake_minimum_required(VERSION 3.2)
project(scanner)

# Find the package using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(CARIBOULITE REQUIRED IMPORTED_TARGET cariboulite)

# Find nlohmann
pkg_check_modules(JSON REQUIRED IMPORTED_TARGET nlohmann_json)
# Add the executable
add_executable(caribouscan main.cpp)

set_property(TARGET caribouscan PROPERTY CXX_STANDARD 20)

# Include directories from the cariboulite package
target_include_directories(caribouscan PRIVATE ${CARIBOULITE_INCLUDE_DIRS})

# Link against the cariboulite library
target_link_libraries(caribouscan PRIVATE PkgConfig::CARIBOULITE PkgConfig::JSON)
