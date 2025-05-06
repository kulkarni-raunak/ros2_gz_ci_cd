#!/usr/bin/env bash
#===============================================================================
# Generate a .rosinstall file for missing dependencies based on existing sources
# in a workspace, then optionally import them.
#
# Usage:
#   rosinstall_generator_from_src.sh [OPTIONS]
#
# Options:
#   --workspace <path>    Path to the top-level workspace (default: current dir)
#   --src <path>          Path to 'src' folder inside workspace (default: <workspace>/src)
#   --rosdistro <distro>  Which ROS 2 distro to use for rosinstall_generator (default: 'jazzy')
#   --exclude <packages>  Packages to exclude (default: 'RPP')
#   --extra-files-suffix <suffix>
#                         Suffix for "extra deps" files to read (default: 'extra_deps.txt')
#   --output <file>       Name for final .rosinstall (default: '<workspace>_extra_deps.rosinstall')
#   --tmpdir <dir>        Directory for temporary files (default: 'tmp')
#   --install-available   Install dependencies for existing packages via rosdep
#   --verbose             Enable verbose output from rosinstall_generator
#   --import              Automatically run "vcs import" after generating .rosinstall
#   --import-dir <path>   Directory to import the missing sources into
#   -h|--help             Show this help message
#
# This script searches for:
#   1) All "*<suffix>" files (e.g. *extra_deps.txt) to gather extra dependencies.
#   2) All "*packages_skip.txt" files to gather additional packages to exclude.
#
# It then identifies existing packages in the src directory, uses rosinstall_generator
# to produce a full .rosinstall file, and finally filters out any packages that match
# local packages in 'src'.
#
# If "--import" is specified, it will automatically import the generated .rosinstall
# into the subdirectory "src/<basename_of_ws>_extra_deps" by default, unless overridden
# with "--import-dir <path>".
#===============================================================================

set -euo pipefail

###############################################################################
# Default Configuration
###############################################################################
WORKSPACE_DIR="$(pwd)"
SRC_DIR=""
ROS_DISTRO="jazzy"
EXCLUDE_PKGS="RPP"  # Default exclude
EXTRA_FILES_SUFFIX="extra_deps.txt"
OUTPUT_FILE="$(basename "$WORKSPACE_DIR")_extra_deps.rosinstall"
TMP_DIR="tmp"
VERBOSE=false

IMPORT=false
IMPORT_DIR=""

INSTALL_AVAILABLE=false

usage() {
  grep '^# ' "$0" | sed 's/^# //'
  exit 1
}

###############################################################################
# Parse Command-Line Arguments
###############################################################################
while [[ $# -gt 0 ]]; do
  case $1 in
    --workspace)
      WORKSPACE_DIR="$2"
      shift 2
      ;;
    --src)
      SRC_DIR="$2"
      shift 2
      ;;
    --rosdistro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --exclude)
      EXCLUDE_PKGS="$2"
      shift 2
      ;;
    --extra-files-suffix)
      EXTRA_FILES_SUFFIX="$2"
      shift 2
      ;;
    --output)
      OUTPUT_FILE="$2"
      shift 2
      ;;
    --tmpdir)
      TMP_DIR="$2"
      shift 2
      ;;
    --verbose)
      VERBOSE=true
      shift
      ;;
    --import)
      IMPORT=true
      shift
      ;;
    --import-dir)
      IMPORT_DIR="$2"
      shift 2
      ;;
    --install-available)
      INSTALL_AVAILABLE=true
      shift
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown option: $1"
      usage
      ;;
  esac
done

# If user didn't override src directory, default to <workspace>/src
if [[ -z "$SRC_DIR" ]]; then
  SRC_DIR="${WORKSPACE_DIR}/src"
fi

# Create temp directory if needed
mkdir -p "${TMP_DIR}"

echo "============================================================"
echo " Workspace Dir      : ${WORKSPACE_DIR}"
echo " Src Dir            : ${SRC_DIR}"
echo " ROS Distro         : ${ROS_DISTRO}"
echo " Exclude Pkgs       : ${EXCLUDE_PKGS}"
echo " Extra Suffix       : ${EXTRA_FILES_SUFFIX}"
echo " Output File        : ${OUTPUT_FILE}"
echo " Temp Dir           : ${TMP_DIR}"
echo " Verbose            : ${VERBOSE}"
echo " Import             : ${IMPORT}"
echo " Install Available  : ${INSTALL_AVAILABLE}"
echo " Import Dir         : ${IMPORT_DIR}"
echo "============================================================"

###############################################################################
# Gather extra dependencies from files matching "*<EXTRA_FILES_SUFFIX>"
###############################################################################
shopt -s nullglob
EXTRA_DEPS_FILES=(*"${EXTRA_FILES_SUFFIX}")
shopt -u nullglob

if [ ${#EXTRA_DEPS_FILES[@]} -eq 0 ]; then
    echo "No *${EXTRA_FILES_SUFFIX} files found. No extra dependencies."
    EXTRA_DEPS=""
else
    # Combine all lines from all matching files
    EXTRA_DEPS=$(cat "${EXTRA_DEPS_FILES[@]}" | tr '\n' ' ')
fi

if [ -z "${EXTRA_DEPS// }" ]; then
    echo "No extra dependencies specified (files empty or not found)."
fi

###############################################################################
# Gather packages to skip from "*packages_skip.txt"
###############################################################################
shopt -s nullglob
PACKAGES_SKIP_FILES=(*packages_skip.txt)
shopt -u nullglob

if [ ${#PACKAGES_SKIP_FILES[@]} -eq 0 ]; then
    echo "No packages_skip.txt files found."
    SKIP_PKGS=""
else
    SKIP_PKGS=$(cat "${PACKAGES_SKIP_FILES[@]}" | tr '\n' ' ')
    echo "Skipping additional packages from file(s):"
    echo "  ${SKIP_PKGS}"
fi

# Merge skip packages with EXCLUDE_PKGS
if [ -n "${SKIP_PKGS// }" ]; then
    EXCLUDE_PKGS="${EXCLUDE_PKGS} ${SKIP_PKGS}"
fi

# Optionally remove duplicates (if desired)
# EXCLUDE_PKGS="$(echo "$EXCLUDE_PKGS" | xargs -n1 | sort -u | xargs)"

echo "Effective Exclude Packages: ${EXCLUDE_PKGS}"
echo ""

###############################################################################
# Identify packages in src directory
###############################################################################
EXISTING_PACKAGES=$(
    find "${SRC_DIR}" -name package.xml -execdir grep -m1 "<name>" {} \; \
    | sed 's/<name>\(.*\)<\/name>/\1/' \
    | tr '\n' ' '
)

echo "Existing packages in the workspace:"
echo "  ${EXISTING_PACKAGES}"
echo ""
###############################################################################
# Install dependencies for existing packages via rosdep if requested
###############################################################################
if [ "$INSTALL_AVAILABLE" = true ]; then
    # Check if rosdep is available
    if ! command -v rosdep &> /dev/null; then
        echo "ERROR: rosdep is not installed. Please install it before using --install-available."
        exit 1
    fi
    echo "============================================================"
    echo " Installing available dependencies for existing packages"
    echo "============================================================"
    set -x
    rosdep install --from-paths "${SRC_DIR}" --ignore-src --rosdistro "${ROS_DISTRO}" -y -r
    set +x
    echo "============================================================"
    echo "rosdep installation completed."
    echo ""
fi
###############################################################################
# Convert AMENT_PREFIX_PATH to a simpler ROS_PACKAGE_PATH
###############################################################################
# 1) Convert AMENT_PREFIX_PATH from colon-delimited to line-delimited
# 2) Truncate each path after "/install"
# 3) Sort uniquely so each install path appears once
# 4) Convert back to colon-delimited
# 5) Remove trailing colons
export ROS_PACKAGE_PATH=$(
  echo "${AMENT_PREFIX_PATH:-}" \
    | tr ':' '\n' \
    | sed -E 's|(.*?/install).*|\1|' \
    | sort -u \
    | tr '\n' ':' \
    | sed 's/:*$//'
)

echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
echo ""

###############################################################################
# Generate full .rosinstall (with no excludes for local packages)
###############################################################################
echo "Generating full .rosinstall for distro '${ROS_DISTRO}' ..."
ROSINSTALL_CMD="rosinstall_generator \
  --rosdistro \"${ROS_DISTRO}\" \
  --deps ${EXTRA_DEPS} ${EXISTING_PACKAGES} \
  --exclude ${EXCLUDE_PKGS}"

if [ "$VERBOSE" = true ]; then
    ROSINSTALL_CMD+=" --verbose"
fi

echo "Running command:"
echo "  $ROSINSTALL_CMD"
eval "${ROSINSTALL_CMD} > \"${TMP_DIR}/full_deps.rosinstall\""

###############################################################################
# Filter out only the local packages (retain their dependencies)
###############################################################################
echo ""
echo "Filtering out any packages that already exist in src/ ..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOVE_SCRIPT="${SCRIPT_DIR}/remove_existing_packages.sh"

if [ ! -f "${REMOVE_SCRIPT}" ]; then
    echo "ERROR: remove_existing_packages.sh not found at: ${REMOVE_SCRIPT}"
    exit 1
fi

source "${REMOVE_SCRIPT}" "${EXISTING_PACKAGES}" "${TMP_DIR}/full_deps.rosinstall" "${OUTPUT_FILE}"

echo ""
echo "============================================================"
echo "Generated '${OUTPUT_FILE}' with only missing external dependencies."
echo "============================================================"

###############################################################################
# If --import or --import-dir specified, perform vcs import
###############################################################################
if [ "$IMPORT" = true ] || [ -n "$IMPORT_DIR" ]; then
    if [ -z "$IMPORT_DIR" ]; then
        # Default import path: src/<basename_of_ws>_extra_deps
        IMPORT_DIR="src/$(basename "$WORKSPACE_DIR")_extra_deps"
    fi

    echo "Importing the missing sources into '${IMPORT_DIR}' ..."
    mkdir -p "$IMPORT_DIR"
    vcs import "$IMPORT_DIR" < "$OUTPUT_FILE"
    echo "Import completed."
else
    # If we aren't importing automatically, show instructions
    echo "To clone the missing sources manually, run:"
    echo "  vcs import src/$(basename "$WORKSPACE_DIR")_extra_deps < ${OUTPUT_FILE}"
fi

echo "Done."
