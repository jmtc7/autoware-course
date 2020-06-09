cmake_minimum_required(VERSION 3.5)

project("SmartMeter")
find_package(Idlpp-cxx REQUIRED)
find_package(CycloneDDS REQUIRED)

if (NOT TARGET CycloneDDS_CXX_API::ddscxx)
	# Find the CycloneDDS_CXX_API package. If it is not in a default location, try
	# finding it relative to the example where it mostly likely resides.
	find_package(CycloneDDS_CXX_API REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
endif()

# This is a convenience function, provided by the CycloneDDS_CXX_API package,
# that will supply a library target related the the given idl file.
# In short, it takes the idl file, generates the source files with
# the proper data tipes and compiles them into a library.
idl_dsscxx_generate(meter_types "Meter.idl")

# Both executables have only one related source file.
add_executable(writer writer.cpp util.cpp)
add_executable(reader reader.cpp util.cpp)

# Both executables need to be linked to the idl data type library and
# the ddsc API library.
target_link_libraries(writer meter_types CycloneDDS_CSS_API::ddscxx)
target_link_libraries(reader meter_types CycloneDDS_CXX_API::ddscxx)

set_property(TARGET writer PROERTY CXX_STANDARD 11)
set_property(TARGET reader PROERTY CXX_STANDARD 11)
