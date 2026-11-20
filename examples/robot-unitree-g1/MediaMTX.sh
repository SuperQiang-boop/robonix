(
  set -euo pipefail

  case "$(uname -m)" in
    x86_64)         arch=amd64 ;;
    aarch64|arm64)  arch=arm64v8 ;;
    *) echo "不支持的架构：$(uname -m)"; exit 1 ;;
  esac

  version=v1.12.3
  archive="mediamtx_${version}_linux_${arch}.tar.gz"
  install_tmp="$(mktemp -d)"
  trap 'rm -rf "$install_tmp"' EXIT

  curl -fL --retry 3 \
    "https://github.com/bluenviron/mediamtx/releases/download/${version}/${archive}" \
    -o "$install_tmp/$archive"

  tar -xzf "$install_tmp/$archive" -C "$install_tmp"
  sudo install -m 755 "$install_tmp/mediamtx" /usr/local/bin/mediamtx
)
