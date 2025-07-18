#!/usr/bin/env bash

BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

function info() {
    (>&2 echo -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() {
    (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() {
    (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() {
    (>&2 echo -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}

export RCFILES_DIR="/opt/apollo/rcfiles"
export APOLLO_DIST="${APOLLO_DIST:-stable}"

export PKGS_DIR="/opt/apollo/pkgs"
export SYSROOT_DIR="/opt/apollo/sysroot"

export APOLLO_PROFILE="/etc/profile.d/apollo.sh"
export APOLLO_LD_FILE="/etc/ld.so.conf.d/apollo.conf"
export DOWNLOAD_LOG="/opt/apollo/build.log"
export LOCAL_HTTP_ADDR="${LOCAL_HTTP_ADDR:-http://172.17.0.1:8388}"

if [[ "$(uname -m)" == "x86_64" ]]; then
    export SUPPORTED_NVIDIA_SMS="5.2 6.0 6.1 7.0 7.5 8.0 8.6 8.9"
else # AArch64
    export SUPPORTED_NVIDIA_SMS="5.3 6.2 7.2 7.5"
fi

function py3_version() {
    local version
    # major.minor.rev (e.g. 3.6.9) expected
    version="$(python3 --version | awk '{print $2}')"
    echo "${version%.*}"
}

function pip3_install() {
    python3 -m pip install --timeout 30 --no-cache-dir $@
}

function apt_get_update_and_install() {
    # --fix-missing
    apt-get -y update && \
        apt-get -y install --no-install-recommends "$@"
}

function apt_get_remove() {
    apt-get -y purge --autoremove "$@"
}

# Ref: https://reproducible-builds.org/docs/source-date-epoch
function source_date_epoch_setup() {
    DATE_FMT="+%Y-%m-%d"
    export SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-$(date +%s)}"
    export BUILD_DATE=$(date -u -d "@$SOURCE_DATE_EPOCH" "$DATE_FMT" 2>/dev/null \
        || date -u -r "$SOURCE_DATE_EPOCH" "$DATE_FMT" 2>/dev/null \
        || date -u "$DATE_FMT")
}

function ensure_ld_path() {
    local lib_path="$1"
    local target_file="${APOLLO_LD_FILE}"

    if [ -z "$lib_path" ]; then
        echo "Error: empty lib_path parameter"
        return 1
    fi

    if [ ! -f "$target_file" ]; then
        echo "$lib_path" >> "$target_file"
        return 0
    fi

    if ! grep -Fxq "$lib_path" "$target_file"; then
        echo "$lib_path" >> "$target_file"
    else
        echo "Path '$lib_path' is already present in $target_file"
    fi
}

function apollo_environ_setup() {
    : "${SOURCE_DATE_EPOCH:=$(source_date_epoch_setup)}"

    if [ ! -d "${PKGS_DIR}" ]; then
        mkdir -p "${PKGS_DIR}"
    fi
    if [ ! -d "${SYSROOT_DIR}" ]; then
        mkdir -p ${SYSROOT_DIR}/{bin,include,lib,share}
    fi

    # Add runtime library path
    ensure_ld_path "${SYSROOT_DIR}/lib"

    if [ ! -f "${APOLLO_PROFILE}" ] && [ -f "/opt/apollo/rcfiles/apollo.sh.sample" ]; then
        cp -f /opt/apollo/rcfiles/apollo.sh.sample "${APOLLO_PROFILE}"
        echo "add_to_path ${SYSROOT_DIR}/bin" >> "${APOLLO_PROFILE}"
    fi
    if [ ! -f "${DOWNLOAD_LOG}" ]; then
        echo "##==== Summary: Apollo Package Downloads ====##" > "${DOWNLOAD_LOG}"
        echo -e "Package\tSHA256\tDOWNLOADLINK" | tee -a "${DOWNLOAD_LOG}"
    fi
}

apollo_environ_setup

function create_so_symlink() {
    local mydir="$1"
    for mylib in $(find "${mydir}" -name "lib*.so.*" -type f); do
        mylib=$(basename "${mylib}")
        ver="${mylib##*.so.}"
        if [ -z "$ver" ]; then
            continue
        fi
        libX="${mylib%%.so*}"
        IFS='.' read -ra arr <<< "${ver}"
        IFS=" " # restore IFS
        ln -s "${mylib}" "${mydir}/${libX}.so.${arr[0]}"
        ln -s "${mylib}" "${mydir}/${libX}.so"
    done
}

function _local_http_cached() {
  curl -sfI "${LOCAL_HTTP_ADDR}/$1" >/dev/null
}

function _checksum_check_pass() {
  local pkg="$1" expected="$2"
  local actual
  actual=$(sha256sum "$pkg" | awk '{print $1}')
  if [[ "$actual" == "$expected" ]]; then
    true
  else
    warning "$(basename "$pkg"): checksum mismatch — expected $expected, got $actual"
    false
  fi
}

# We only accept predownloaded git tarballs with format
# "pkgname.git.53549ad.tgz" or "pkgname_version.git.53549ad.tgz"
function package_schema() {
  local link=$1 name=$2
  if [[ "${link##*.}" == "git" ]]; then
    echo git; return
  fi
  IFS='.' read -ra parts <<< "$name"; IFS=' '
  if [[ ${#parts[@]} -gt 3 && "${parts[-3]}" == "git" && ${#parts[-2]} -eq 7 ]]; then
    echo git
  else
    echo http
  fi
}

function download_if_not_cached() {
  local pkg="$1" expected_cs="$2" url="$3"
  echo -e "${pkg}\t${expected_cs}\t${url}" >> "${DOWNLOAD_LOG}"

  # Get from local cache
  if _local_http_cached "$pkg"; then
    info "Local cache hit: $pkg"
    wget -q --no-dns-cache -4 "${LOCAL_HTTP_ADDR}/$pkg" -O "$pkg"
    if _checksum_check_pass "$pkg" "$expected_cs"; then
      ok "Fetched $pkg from cache"
      return
    else
      rm -f "$pkg"
    fi
  fi

  local schema
  schema=$(package_schema "$url" "$pkg")

  case "$schema" in
    http)
      info "Downloading $pkg from $url"
      wget -q --tries=3 --timeout=30 --no-dns-cache -4 -N "$url" -O "$pkg"
      if ! _checksum_check_pass "$pkg" "$expected_cs"; then
        error "Checksum failed for $pkg"
      fi
      ;;
    git)
      info "Cloning git repo $url (shallow)"
      git clone --depth 1 --branch master --single-branch --recurse-submodules "$url" "$pkg.git"
      ok "Cloned $pkg"
      ;;
    *)
      error "Unknown schema '$schema' for $pkg"
      ;;
  esac

  # Node: you can enable the following lines to cache the downloaded package by
  # running a scratch container, and then use is as the local cache
  # server(LOCAL_HTTP_ADDR) to speed up future builds.
  # TODO(All): a better way to cache packages
  # After downloading, save to local cache
  # case "$schema" in
  #   http)
  #     cp -f "$pkg" "${LOCAL_CACHE_DIR}/$pkg"
  #     ;;
  #   git)
  #     tar czf "${pkg}.git.tgz" "$pkg.git"
  #     cp -f "${pkg}.git.tgz" "${LOCAL_CACHE_DIR}/"
  #     rm -rf "$pkg.git"
  #     ;;
  # esac
  # ok "Cached $pkg into $LOCAL_CACHE_DIR"
}
