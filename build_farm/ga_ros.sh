#!/usr/bin/env bash

#=================================================
#                   ros环境自动化脚本
#=================================================

# 颜色
BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

GA_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P )"
# GA_ROS_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd -P )"
GA_ROS_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P )"

function source_ga_base() {
  DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  cd "${DIR}"

  source ${DIR}/ga_utilities/ga_setup/scripts/ga_base.sh $1
  source /opt/ros/${GA_ROS_DIST}/setup.bash
}

function ga_check_system_config() {
  # check docker environment
    # 检查操作系统规格是否满足要求, 设置一些变量
  # check operating system
  OP_SYSTEM=$(uname -s)
  case $OP_SYSTEM in
    "Linux")
      echo "System check passed. Build continue ..."
      ;;
    *)
      error "Unsupported system: ${OP_SYSTEM}."
      error "Please use Linux, we recommend Ubuntu 16.04."
      exit 1
  esac

  # 检测是16.04还是18.04
  case "$(lsb_release -r | cut -f2)" in
      18.04)
        GA_ROS_DIST="dashing"
        ;;
      *)
      GA_ROS_DIST="foxy"
        ;;
      *)
      error "Unsupported ubuntu distro"
      error "Please use Linux, we recommend Ubuntu 16.04."
      exit 1
  esac
}

function check_machine_arch() {
  # the machine type, currently support x86_64, aarch64
  MACHINE_ARCH=$(uname -m)

}

#=================================================
#              Build functions
#=================================================

function build_dbg() {
  info "Start building, please wait ..."

  info "Building on $MACHINE_ARCH..."

  info "Building with $JOB_ARG for $MACHINE_ARCH"

  cd ${GA_ROS_ROOT_DIR}
  export GPUAC_COMPILE_WITH_CUDA=1
  # --force-cmake
  info "colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG}  ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=Debug"
  colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG}  ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=Debug
  if [ ${PIPESTATUS[0]} -ne 0 ]; then
    fail 'Build failed!'
  fi
  info "please source ./colcon/install/setup.bash"
  cd -
  success 'build passed!'
}

function build_rel_dbg() {
  info "Start building, please wait ..."

  info "Building on $MACHINE_ARCH..."

  MACHINE_ARCH=$(uname -m)
  info "Building with $JOB_ARG for $MACHINE_ARCH"

  cd ${GA_ROS_ROOT_DIR}
  export GPUAC_COMPILE_WITH_CUDA=1
  export CMAKE_BUILD_PARALLEL_LEVEL=10
  # --force-cmake
  info "colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG}  ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=RelWithDebInfo"
  colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG}  ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=RelWithDebInfo
  if [ ${PIPESTATUS[0]} -ne 0 ]; then
    fail 'Build failed!'
  fi
  info "please source ./install/setup.bash"
  cd -
  success 'build passed!'
}

# https://colcon.readthedocs.io/en/released/user/installation.html
function build_rel() {
  info "Start building, please wait ..."

  info "Building on $MACHINE_ARCH..."

  info "Building with $JOB_ARG for $MACHINE_ARCH"

  cd ${GA_ROS_ROOT_DIR}
  export GPUAC_COMPILE_WITH_CUDA=1
  # --force-cmake
  info "colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG}  ${CMAKE_OPT}-DCMAKE_BUILD_TYPE=Release"
  colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG} ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=Release

  # --packages-up-to
  if [ ${PIPESTATUS[0]} -ne 0 ]; then
    fail 'Build failed!'
  fi
  info "please source ./install/setup.bash"
  cd -
  success 'build passed!'
}

function dpkg_install() {
  info "Start install, please wait ..."

  info "installing on $MACHINE_ARCH..."

  cd ${GA_ROS_ROOT_DIR}/build
  sudo dpkg -i --force-overwrite *.deb
  cd -
  success 'dpkg install  passed!'
}

function pkg_rel() {
  info "Start packaging, please wait ..."

  info "packaging on $MACHINE_ARCH..."

  info "packaging with $JOB_ARG for $MACHINE_ARCH"

  cd ${GA_ROS_ROOT_DIR}
  # --force-cmake
  set -e
  export GPUAC_COMPILE_WITH_CUDA=1
  info "packaging gpuac_base"
  colcon ${JOB_ARG} --only-pkg-with-deps gpuac_base --make-args package \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_BUILD_BINARY_PACKAGE=1 \
    -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=0
  info "packaging gpuac_deps/open3d_conversions"
  colcon ${JOB_ARG} --only-pkg-with-deps open3d_conversions --make-args package \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_BUILD_BINARY_PACKAGE=1 \
    -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=0
  info "packaging gpuac_localization/hdl_localization_gpu"
  colcon ${JOB_ARG} --only-pkg-with-deps hdl_localization_gpu --make-args package \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_BUILD_BINARY_PACKAGE=1 \
    -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=0
  # info "packaging gpuac_example"
  # colcon ${JOB_ARG} --only-pkg-with-deps gpuac_example --make-args package \
  #   --cmake-args -DCMAKE_BUILD_TYPE=Release \
  #   -DCATKIN_BUILD_BINARY_PACKAGE=1 \
  #   -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
  #   -DCMAKE_EXPORT_COMPILE_COMMANDS=0
  info "packaging gpuac_localization/gpuac_ndt_gpu"
  colcon ${JOB_ARG} --only-pkg-with-deps gpuac_ndt_gpu --make-args package \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_BUILD_BINARY_PACKAGE=1 \
    -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=0
  # info "packaging gpuac_utilities/autoware_health_checker"
  # colcon ${JOB_ARG} --only-pkg-with-deps autoware_health_checker --make-args package \
  #   --cmake-args -DCMAKE_BUILD_TYPE=Release \
  #   -DCATKIN_BUILD_BINARY_PACKAGE=1 \
  #   -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
  #   -DCMAKE_EXPORT_COMPILE_COMMANDS=0
  # info "packaging gpuac_utilities/autoware_system_msgs"
  # colcon ${JOB_ARG} --only-pkg-with-deps autoware_system_msgs --make-args package \
  #   --cmake-args -DCMAKE_BUILD_TYPE=Release \
  #   -DCATKIN_BUILD_BINARY_PACKAGE=1 \
  #   -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
  #   -DCMAKE_EXPORT_COMPILE_COMMANDS=0
  # info "packaging gpuac_utilities/gpuac_system_monitor"
  # colcon --only-pkg-with-deps gpuac_system_monitor --make-args package \
  #   --cmake-args -DCMAKE_BUILD_TYPE=Release \
  #   -DCATKIN_BUILD_BINARY_PACKAGE=1 \
  #   -DCMAKE_INSTALL_PREFIX=/opt/ros/${GA_ROS_DIST} \
  #   -DCMAKE_EXPORT_COMPILE_COMMANDS=0

  if [ ${PIPESTATUS[0]} -ne 0 ]; then
    fail 'packaging failed!'
  fi
  info "please check deb file in ../../build/"
  cd -
  success 'pkg passed!'
}

# 编译测试检查
function check() {
  bash $0 build && bash $0 "test" && bash $0 lint

  if [ $? -eq 0 ]; then
    success 'Check passed!'
    return 0
  else
    fail 'Check failed!'
    return 1
  fi
}

function warn_proprietary_sw() {
  echo -e "${RED}all rights reserved, ga_team.${NO_COLOR}"
}

function run_unit_test() {
  # if [ "$MACHINE_ARCH" == 'aarch64' ]; then
  #   info "not supported in aarch64, please use it in your pc"
  #   exit 1
  # fi
  cd ${GA_ROS_ROOT_DIR}/build
  ctest
  if [ $? -eq 0 ]; then
    success 'Test passed!'
    return 0
  fi
  cd -
}

function run_cpp_lint() {
  echo "not support"
}

function run_bash_lint() {
  FILES=$(find "${APOLLO_ROOT_DIR}" -type f -name "*.sh" | grep -v ros)
  echo "${FILES}" | xargs shellcheck
}

function run_lint() {
  # Add cpplint rule to BUILD files that do not contain it.
  run_cpp_lint

  if [ $? -eq 0 ]; then
    success 'Lint passed!'
  else
    fail 'Lint failed!'
  fi
}

function clean() {
  # Remove catkin files
  cd ${GA_ROS_ROOT_DIR}
  info "Start cleaning, please wait ..."
  info "rm -rf  build/ install/ log/ devel/ src/CMakeLists.txt"
  rm -rf  ./colcon
  cd -
  success "clean passed!"
}

function gen_doc() {
  rm -rf
  doxygen apollo.doxygen
}

function check_hardware(){
  # check dabai
  info "Start check_hardware, please wait ..."
  cd ${GA_ROS_ROOT_DIR}/devel/lib/ga_ros

  ./astra_test

  if [ $? -eq 0 ]; then
    success 'check_hardware passed!'
    return 0
  else
    fail 'check_hardware failed!'
    return 1
  fi
  cd -
}

function check_algorithm(){
  # check rvga_camera3d
  info "Start check_algorithm, please wait ..."
  cd ${GA_ROS_ROOT_DIR}/devel/lib/ga_ros

  ./rvga_ultrasonic_test && ./rvga_camera3d_test && ./linear_math_test

  if [ $? -eq 0 ]; then
    success 'check_algorithm passed!'
    return 0
  else
    fail 'check_algorithm failed!'
    return 1
  fi
}

function version() {
  echo "not support"
}

function get_revision() {
  echo "not support"
}

function get_branch() {
  git branch &> /dev/null
  if [ $? = 0 ];then
    BRANCH=$(git rev-parse --abbrev-ref HEAD)
  else
    warning "Could not get the branch name, maybe this is not a git work tree." >&2
    BRANCH="unknown"
  fi
  echo "$BRANCH"
}

function config() {
  JOB_ARG="--parallel-workers 10"
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    JOB_ARG="--parallel-workers 6"
  fi
  LOG_ARG=" --log-base ./colcon/log "
  COLCON_ARG=" --build-base ./colcon/build --install-base ./colcon/install --symlink-install"
  # --symlink-install --merge-install
  CMAKE_OPT=" --cmake-args -DBUILD_TESTING=OFF -DPERFORMANCE_TEST_ICEORYX_ENABLED=ON -DPERFORMANCE_TEST_CYCLONEDDS_ENABLED=OFF -DPERFORMANCE_TEST_FASTRTPS_ENABLED=OFF -DPERFORMANCE_TEST_RCLCPP_EVENTS_EXECUTOR_ENABLED=ON"
}

function set_use_gpu() {
  echo "not support"
}

function print_usage() {
  RED='\033[0;31m'
  BLUE='\033[0;34m'
  BOLD='\033[1m'
  NONE='\033[0m'

  echo -e "\n${RED}Usage${NONE}:
  .${BOLD}/ga_ros.sh${NONE} [OPTION]"

  echo -e "\n${RED}Options${NONE}:
  ${BLUE}build_dbg${NONE}: build_dbg
  ${BLUE}build_rel_dbg${NONE}: build rel with dbg info
  ${BLUE}build_rel${NONE}: build_rel
  ${BLUE}pkg_rel${NONE}: pkg_rel
  ${BLUE}dpkg_install${NONE}: install deb files
  ${BLUE}unit_test${NONE}: run all unit tests
  ${BLUE}check_hardware${NONE}: run programs to check hardwares
  ${BLUE}check_algorithm${NONE}: run programs to check algorithms
  ${BLUE}clean${NONE}: rm -rf build/ log/ install/ devel/
  ${BLUE}doc${NONE}: generate doxygen document
  ${BLUE}usage${NONE}: print this menu
  "
}

function main() {
  source_ga_base

  check_machine_arch
  ga_check_system_config
  config

  if [ ${MACHINE_ARCH} == "x86_64" ]; then
    echo "x86_64"
  fi

  local cmd=$1
  shift

  START_TIME=$(get_now)
  case $cmd in
    build_dbg)
      build_dbg $@
      ;;
    build_rel_dbg)
      build_rel_dbg $@
      ;;
    build_rel)
      build_rel $@
      ;;
    pkg_rel)
      pkg_rel $@
      ;;
    unit_test)
      run_unit_test $@
      ;;
    dpkg_install)
      dpkg_install $@
      ;;
    check_hardware)
      check_hardware $@
      ;;
    check_algorithm)
      check_algorithm $@
      ;;
    catkin)
      catkin $@
      ;;
    config)
      config
      ;;
    doc)
      gen_doc
      ;;
    clean)
      clean
      ;;
    version)
      version
      ;;
    usage)
      print_usage
      ;;
    *)
      print_usage
      ;;
  esac
}

main $@
