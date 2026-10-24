# Runs one B-Human robot against a K1 simulated in an external MuJoCo simulation
# (see Src/Apps/BHumanMuJoCo/Main.cpp). Only configured by Make/Linux/compileMuJoCo,
# because the whole code is compiled with TARGET_MUJOCO for it.
# BOOSTER_SDK_DIR must point to a Booster Robotics SDK that contains lib/x86_64.

set(BHUMAN_MUJOCO_ROOT_DIR "${BHUMAN_PREFIX}/Src/Apps/BHumanMuJoCo")
set(BHUMAN_MUJOCO_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/BHumanMuJoCo/$<CONFIG>")

if(NOT EXISTS "${BOOSTER_SDK_DIR}/lib/x86_64/libbooster_robotics_sdk.a")
  message(FATAL_ERROR "BOOSTER_SDK_DIR (${BOOSTER_SDK_DIR}) does not contain lib/x86_64/libbooster_robotics_sdk.a")
endif()

add_library(BoosterX86::SDK STATIC IMPORTED)
set_target_properties(BoosterX86::SDK PROPERTIES
    IMPORTED_LOCATION "${BOOSTER_SDK_DIR}/lib/x86_64/libbooster_robotics_sdk.a"
    INTERFACE_INCLUDE_DIRECTORIES "${BOOSTER_SDK_DIR}/include"
    INTERFACE_LINK_LIBRARIES "pthread;dl;rt")

# BoosterProvider includes the SDK.
target_link_libraries(B-Human PRIVATE BoosterX86::SDK)

add_executable(BHumanMuJoCo "${BHUMAN_MUJOCO_ROOT_DIR}/Main.cpp")
set_property(TARGET BHumanMuJoCo PROPERTY RUNTIME_OUTPUT_DIRECTORY "${BHUMAN_MUJOCO_OUTPUT_DIR}")
target_link_libraries(BHumanMuJoCo PRIVATE B-Human)
target_link_libraries(BHumanMuJoCo PRIVATE BoosterX86::SDK)
target_link_libraries(BHumanMuJoCo PRIVATE Flags::Default)

# The SDK headers contain template code clang only accepts if it is parsed when instantiated.
set_source_files_properties("${BHUMAN_PREFIX}/Src/Modules/Infrastructure/BoosterProvider/BoosterProvider.cpp"
    "${BHUMAN_PREFIX}/Src/Threads/Motion.cpp" "${BHUMAN_MUJOCO_ROOT_DIR}/Main.cpp"
    PROPERTIES COMPILE_OPTIONS -fdelayed-template-parsing SKIP_PRECOMPILE_HEADERS ON)
