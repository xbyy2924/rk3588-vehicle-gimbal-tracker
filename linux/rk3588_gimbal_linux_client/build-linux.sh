#!/usr/bin/env bash
set -euo pipefail

target="rk3588"
build_type="Release"
enable_asan="OFF"

usage() {
    cat <<'EOF'
Usage: ./build-linux.sh [-t rk3588] [-b Release|Debug] [-m] [-r]
  -t  target SoC; this project supports rk3588
  -b  CMake build type (default: Release)
  -m  enable AddressSanitizer; use with Debug
  -r  accepted for compatibility; CPU preprocessing is already used

The script never deletes model/ or install/rk3588_linux/model/.
EOF
}

while getopts ":t:b:mrh" option; do
    case "${option}" in
        t) target="${OPTARG}" ;;
        b) build_type="${OPTARG}" ;;
        m) enable_asan="ON" ;;
        r) : ;;
        h) usage; exit 0 ;;
        :) echo "Option -${OPTARG} requires a value" >&2; usage; exit 2 ;;
        \?) echo "Unknown option: -${OPTARG}" >&2; usage; exit 2 ;;
    esac
done

if [[ "${target}" != "rk3588" ]]; then
    echo "Unsupported target: ${target}; expected rk3588" >&2
    exit 2
fi
if [[ "${build_type}" != "Release" && "${build_type}" != "Debug" ]]; then
    echo "Unsupported build type: ${build_type}; expected Release or Debug" >&2
    exit 2
fi
if [[ "${enable_asan}" == "ON" && "${build_type}" != "Debug" ]]; then
    echo "-m requires -b Debug" >&2
    exit 2
fi

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
build_dir="${project_dir}/build/build_${target}_linux"
install_dir="${project_dir}/install/${target}_linux"
source_model="${project_dir}/model/yolov8n_rk3588_int8.rknn"

# The historical GitHub backup does not contain the model. Import it once from
# a known location when available; never remove or overwrite an existing model.
if [[ ! -s "${source_model}" ]]; then
    model_candidate="${GIMBAL_MODEL_PATH:-}"
    if [[ -z "${model_candidate}" && -s "${project_dir}/../model/yolov8n_rk3588_int8.rknn" ]]; then
        model_candidate="${project_dir}/../model/yolov8n_rk3588_int8.rknn"
    fi
    if [[ -z "${model_candidate}" && -s "${HOME}/work/yuntai/model/yolov8n_rk3588_int8.rknn" ]]; then
        model_candidate="${HOME}/work/yuntai/model/yolov8n_rk3588_int8.rknn"
    fi
    if [[ -n "${model_candidate}" && -s "${model_candidate}" ]]; then
        cp -- "${model_candidate}" "${source_model}"
        echo "Imported model from: ${model_candidate}"
    fi
fi

cmake_args=(
    -S "${project_dir}"
    -B "${build_dir}"
    -DCMAKE_BUILD_TYPE="${build_type}"
    -DCMAKE_INSTALL_PREFIX="${install_dir}"
)
if [[ "${enable_asan}" == "ON" ]]; then
    cmake_args+=(
        -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer"
        -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer"
        -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"
    )
fi

echo "PROJECT_DIR=${project_dir}"
echo "BUILD_DIR=${build_dir}"
echo "INSTALL_DIR=${install_dir}"
echo "BUILD_TYPE=${build_type}"
cmake "${cmake_args[@]}"
cmake --build "${build_dir}" --parallel "$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
cmake --install "${build_dir}"

if [[ -x "${install_dir}/app" ]]; then
    echo
    echo "Build complete: ${install_dir}/app"
    if [[ -s "${install_dir}/model/yolov8n_rk3588_int8.rknn" ]]; then
        echo "Run: cd ${install_dir} && ./app"
    else
        echo "Model is still missing. Copy yolov8n_rk3588_int8.rknn to:"
        echo "  ${install_dir}/model/"
        echo "Then run: cd ${install_dir} && ./app"
    fi
fi

