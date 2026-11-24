#!/usr/bin/env bash
set -eo pipefail

mode=systemd
case "${1:-}" in
    --install-only) mode=install; shift ;;
    --container) mode=container; shift ;;
esac

if [[ $# -gt 1 || ${1:-} == --* ]]; then
    echo "Usage: bash install-remote.sh [--install-only|--container] [wheel-path-or-URL]" >&2
    exit 2
fi

prefix=${CALIB_SNAP_PREFIX:-/opt/calib-snap}
wheel=${1:-https://pip.wgists.me/dist/calib-snap/calib_snap-0.1.6+remote.1.wireless.lan-py3-none-any.whl}
if [[ ! $prefix =~ ^/[a-zA-Z0-9._/-]+$ ]]; then
    echo "CALIB_SNAP_PREFIX must be an absolute directory without spaces." >&2
    exit 2
fi
if [[ $mode == systemd && ( $EUID -ne 0 || ! -d /run/systemd/system ) ]]; then
    echo "Run with sudo on a systemd robot. Use --container with a container restart policy." >&2
    exit 2
fi

mkdir -p "$prefix"
if command -v uv >/dev/null; then
    if [[ ! -x $prefix/.venv/bin/python ]]; then
        uv venv --system-site-packages --python "$(command -v python3)" "$prefix/.venv"
    fi
    installer=(uv pip install --python "$prefix/.venv/bin/python")
else
    if [[ ! -x $prefix/.venv/bin/python ]]; then
        python3 -m venv --system-site-packages "$prefix/.venv"
    fi
    installer=("$prefix/.venv/bin/python" -m pip install)
fi
if "$prefix/.venv/bin/python" -c 'import cv2, numpy' >/dev/null 2>&1; then
    installer+=(--no-deps)
fi
"${installer[@]}" "$wheel"

cat >"$prefix/run-remote" <<'LAUNCHER'
#!/usr/bin/env bash
set -e
ros_setup=${CALIB_SNAP_ROS_SETUP:-/opt/ros/noetic/setup.bash}
if [[ -r $ros_setup ]]; then
    source "$ros_setup"
fi
exec "$(dirname "$0")/.venv/bin/calib-snap-remote" "$@"
LAUNCHER
chmod 755 "$prefix/run-remote"

if [[ $mode == container ]]; then
    exec "$prefix/run-remote"
fi
if [[ $mode == systemd ]]; then
    install -m 644 /dev/stdin /etc/systemd/system/calib-snap-remote.service <<UNIT
[Unit]
Description=calib-snap LAN capture node
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
DynamicUser=yes
StateDirectory=calib-snap
EnvironmentFile=-/etc/default/calib-snap-remote
ExecStart=$prefix/run-remote --spool /var/lib/calib-snap
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT
    systemctl daemon-reload
    systemctl enable calib-snap-remote.service
    systemctl restart calib-snap-remote.service
    systemctl is-active calib-snap-remote.service
fi
