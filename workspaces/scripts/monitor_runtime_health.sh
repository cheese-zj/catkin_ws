#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash workspaces/scripts/monitor_runtime_health.sh [options]

Purpose:
  Continuous runtime monitor for ROS + system health. Writes one session folder
  with kernel/USB logs, ROS warning stream, topic rate monitors, and periodic
  system snapshots for post-mortem analysis.

Options:
  --out-root DIR                 Session root directory.
                                 Default: /home/jameszhao2004/catkin_ws/outputs/monitor_sessions
  --sample-sec FLOAT             Fast sample period in seconds. Default: 1
  --snapshot-sec FLOAT           Heavy snapshot period in seconds. Default: 10
  --top-n INT                    Top processes to keep per snapshot. Default: 20
  --topic TOPIC                  Add rostopic hz monitor topic (repeatable).
                                 If not provided, a built-in default set is used.
  --topic-window INT             rostopic hz window size. Default: 200
  --rosout-topic TOPIC           ROS log topic to watch. Default: /rosout_agg
  --rosout-pattern REGEX         grep regex for rosout filtering.
  --disable-rosout-filter        Do not run rosout filtered stream.
  --disable-topic-hz             Do not run rostopic hz monitors.
  --help                         Show this message.

Examples:
  bash workspaces/scripts/monitor_runtime_health.sh

  bash workspaces/scripts/monitor_runtime_health.sh \
    --sample-sec 0.5 --snapshot-sec 5 \
    --topic /wrist_opp/image_raw/compressed \
    --topic /realsense_top/color/image_raw
EOF
}

OUT_ROOT="/home/jameszhao2004/catkin_ws/outputs/monitor_sessions"
SAMPLE_SEC="1"
SNAPSHOT_SEC="10"
TOP_N="20"
TOPIC_WINDOW="200"
ROSOUT_TOPIC="/rosout_agg"
ROSOUT_PATTERN="failed to set power state|RS2_USB_STATUS_BUSY|USB|usb|UVC|OPP camera|Joint pos ref clamped|MIT torque clamped|Clipped ARM joints|Exception|ERROR|FATAL|shutdown"
ENABLE_ROSOUT_FILTER="1"
ENABLE_TOPIC_HZ="1"

declare -a TOPICS=()
declare -a TOPIC_FILES=()

DEFAULT_TOPICS=(
  "/robot/arm_left/joint_states_single"
  "/robot/arm_right/joint_states_single"
  "/teleop/arm_left/joint_states_single"
  "/teleop/arm_right/joint_states_single"
  "/wrist_opp/image_raw/compressed"
  "/realsense_left/color/image_raw"
  "/realsense_right/color/image_raw"
  "/realsense_top/color/image_raw"
)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-root)
      OUT_ROOT="${2:-}"
      shift 2
      ;;
    --sample-sec)
      SAMPLE_SEC="${2:-}"
      shift 2
      ;;
    --snapshot-sec)
      SNAPSHOT_SEC="${2:-}"
      shift 2
      ;;
    --top-n)
      TOP_N="${2:-}"
      shift 2
      ;;
    --topic)
      TOPICS+=("${2:-}")
      shift 2
      ;;
    --topic-window)
      TOPIC_WINDOW="${2:-}"
      shift 2
      ;;
    --rosout-topic)
      ROSOUT_TOPIC="${2:-}"
      shift 2
      ;;
    --rosout-pattern)
      ROSOUT_PATTERN="${2:-}"
      shift 2
      ;;
    --disable-rosout-filter)
      ENABLE_ROSOUT_FILTER="0"
      shift
      ;;
    --disable-topic-hz)
      ENABLE_TOPIC_HZ="0"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "[monitor] Unknown arg: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ ${#TOPICS[@]} -eq 0 ]]; then
  TOPICS=("${DEFAULT_TOPICS[@]}")
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${OUT_ROOT}/${timestamp}"
mkdir -p "${SESSION_DIR}"

PID_CSV="${SESSION_DIR}/background_processes.csv"
SAMPLE_CSV="${SESSION_DIR}/sample_fast.csv"
HEARTBEAT_TXT="${SESSION_DIR}/heartbeat.txt"
SUMMARY_TXT="${SESSION_DIR}/summary.txt"
mkdir -p "${SESSION_DIR}/snapshots"

declare -a BG_PIDS=()
declare -a BG_LABELS=()
ROS_STREAMS_STARTED="0"
STOP_REASON="running"

cat > "${SAMPLE_CSV}" <<'EOF'
iso_ts,epoch_sec,cpu_pct,mem_used_pct,mem_used_mb,mem_total_mb,swap_used_mb,swap_total_mb,load1,load5,load15,procs_running,procs_total,piper_ctrl_proc,realsense_proc,opp_cam_proc,rviz_proc,roslaunch_proc,ros_master_up,ros_nodes,ros_topics
EOF

echo "pid,label,log_file" > "${PID_CSV}"
{
  echo "session_dir=${SESSION_DIR}"
  echo "start_iso=$(date --iso-8601=seconds)"
  echo "host=$(hostname)"
  echo "user=${USER:-unknown}"
  echo "sample_sec=${SAMPLE_SEC}"
  echo "snapshot_sec=${SNAPSHOT_SEC}"
  echo "top_n=${TOP_N}"
  echo "topic_window=${TOPIC_WINDOW}"
  echo "rosout_topic=${ROSOUT_TOPIC}"
  echo "rosout_pattern=${ROSOUT_PATTERN}"
  echo "topics_count=${#TOPICS[@]}"
  printf "topics=%s\n" "${TOPICS[*]}"
} > "${SESSION_DIR}/config.env"

uname -a > "${SESSION_DIR}/uname.txt" || true
lsusb -t > "${SESSION_DIR}/lsusb_tree_start.txt" 2>&1 || true
ip -brief link > "${SESSION_DIR}/ip_link_brief_start.txt" 2>&1 || true
ip -details link show type can > "${SESSION_DIR}/can_link_start.txt" 2>&1 || true
df -h > "${SESSION_DIR}/df_start.txt" 2>&1 || true

if command -v rosnode >/dev/null 2>&1; then
  rosnode list > "${SESSION_DIR}/rosnode_list_start.txt" 2>&1 || true
fi
if command -v rostopic >/dev/null 2>&1; then
  rostopic list > "${SESSION_DIR}/rostopic_list_start.txt" 2>&1 || true
fi
if command -v rosparam >/dev/null 2>&1; then
  rosparam get / > "${SESSION_DIR}/rosparam_tree_start.yaml" 2>&1 || true
fi

sanitize_topic() {
  local t="$1"
  t="${t#/}"
  t="${t//\//__}"
  t="${t//:/_}"
  t="${t// /_}"
  printf "%s" "${t}"
}

log() {
  local ts
  ts="$(date --iso-8601=seconds)"
  echo "[monitor][${ts}] $*"
}

register_bg_pid() {
  local pid="$1"
  local label="$2"
  local logfile="$3"
  BG_PIDS+=("${pid}")
  BG_LABELS+=("${label}")
  echo "${pid},${label},${logfile}" >> "${PID_CSV}"
}

start_journal_kernel_stream() {
  local logfile="${SESSION_DIR}/journal_kernel_follow.log"
  (
    set +e
    stdbuf -oL -eL journalctl -kf -o short-precise
  ) >> "${logfile}" 2>&1 &
  register_bg_pid "$!" "journal_kernel_follow" "${logfile}"
}

ros_master_up() {
  if ! command -v rosnode >/dev/null 2>&1; then
    return 1
  fi
  rosnode list >/dev/null 2>&1
}

start_ros_streams_if_ready() {
  if [[ "${ROS_STREAMS_STARTED}" == "1" ]]; then
    return
  fi
  if ! ros_master_up; then
    return
  fi

  if [[ "${ENABLE_ROSOUT_FILTER}" == "1" ]] && command -v rostopic >/dev/null 2>&1; then
    local rosout_log="${SESSION_DIR}/rosout_filtered.log"
    (
      set +e
      stdbuf -oL -eL rostopic echo "${ROSOUT_TOPIC}" | stdbuf -oL -eL grep --line-buffered -E "${ROSOUT_PATTERN}"
    ) >> "${rosout_log}" 2>&1 &
    register_bg_pid "$!" "rosout_filtered" "${rosout_log}"
  fi

  if [[ "${ENABLE_TOPIC_HZ}" == "1" ]] && command -v rostopic >/dev/null 2>&1; then
    for topic in "${TOPICS[@]}"; do
      local safe
      safe="$(sanitize_topic "${topic}")"
      local hz_log="${SESSION_DIR}/topic_hz_${safe}.log"
      TOPIC_FILES+=("${topic}:${hz_log}")
      (
        set +e
        stdbuf -oL -eL rostopic hz -w "${TOPIC_WINDOW}" "${topic}"
      ) >> "${hz_log}" 2>&1 &
      register_bg_pid "$!" "topic_hz:${topic}" "${hz_log}"
    done
  fi

  ROS_STREAMS_STARTED="1"
  log "ROS master detected, ROS streams started."
}

kill_bg() {
  local i
  for (( i=0; i<${#BG_PIDS[@]}; i++ )); do
    local pid="${BG_PIDS[$i]}"
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  sleep 0.5
  for pid in "${BG_PIDS[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -9 "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

snapshot_heavy() {
  local epoch iso
  epoch="$(date +%s)"
  iso="$(date --iso-8601=seconds)"
  local prefix="${SESSION_DIR}/snapshots/snap_${epoch}"
  {
    echo "### ${iso}"
    lsusb -t
    echo
  } >> "${SESSION_DIR}/usb_topology.log" 2>&1 || true
  {
    echo "### ${iso}"
    ip -details link show type can
    echo
  } >> "${SESSION_DIR}/can_status.log" 2>&1 || true
  ps -eo pid,ppid,%cpu,%mem,rss,etime,cmd --sort=-%cpu | head -n "$((TOP_N + 1))" > "${prefix}_top_cpu.txt" 2>&1 || true
  ps -eo pid,ppid,%cpu,%mem,rss,etime,cmd --sort=-%mem | head -n "$((TOP_N + 1))" > "${prefix}_top_mem.txt" 2>&1 || true
  df -h > "${prefix}_df.txt" 2>&1 || true
  if command -v rosnode >/dev/null 2>&1; then
    rosnode list > "${prefix}_rosnode_list.txt" 2>&1 || true
  fi
  if command -v rostopic >/dev/null 2>&1; then
    rostopic list > "${prefix}_rostopic_list.txt" 2>&1 || true
  fi
}

on_exit() {
  if [[ "${STOP_REASON}" == "running" ]]; then
    STOP_REASON="exit"
  fi
  log "Stopping monitor (${STOP_REASON}), collecting final snapshot."
  snapshot_heavy
  kill_bg
  {
    echo "stop_iso=$(date --iso-8601=seconds)"
    echo "stop_reason=${STOP_REASON}"
    echo "session_dir=${SESSION_DIR}"
    echo "tip=use journal_kernel_follow.log + rosout_filtered.log + sample_fast.csv first"
  } >> "${SESSION_DIR}/config.env"
  local -a key_files=(
    "${SAMPLE_CSV}"
    "${SESSION_DIR}/journal_kernel_follow.log"
    "${SESSION_DIR}/rosout_filtered.log"
    "${SESSION_DIR}/usb_topology.log"
    "${SESSION_DIR}/can_status.log"
  )
  {
    echo "Session: ${SESSION_DIR}"
    echo "Stop reason: ${STOP_REASON}"
    echo "Files:"
    for f in "${key_files[@]}"; do
      if [[ -f "${f}" ]]; then
        echo "  - ${f}"
      fi
    done
  } > "${SUMMARY_TXT}"
}

on_sigint() {
  STOP_REASON="SIGINT"
  exit 0
}

on_sigterm() {
  STOP_REASON="SIGTERM"
  exit 0
}

trap on_sigint INT
trap on_sigterm TERM
trap on_exit EXIT

log "Session directory: ${SESSION_DIR}"
start_journal_kernel_stream
start_ros_streams_if_ready

read -r _ u n s idle iow irq sirq st _ < /proc/stat
prev_total=$((u + n + s + idle + iow + irq + sirq + st))
prev_idle=$((idle + iow))

next_snapshot_epoch="$(date +%s)"

while true; do
  start_ros_streams_if_ready

  epoch_now="$(date +%s)"
  iso_now="$(date --iso-8601=seconds)"

  read -r _ u n s idle iow irq sirq st _ < /proc/stat
  total_now=$((u + n + s + idle + iow + irq + sirq + st))
  idle_now=$((idle + iow))
  total_delta=$((total_now - prev_total))
  idle_delta=$((idle_now - prev_idle))
  cpu_pct="0.00"
  if [[ "${total_delta}" -gt 0 ]]; then
    cpu_pct="$(awk -v td="${total_delta}" -v id="${idle_delta}" 'BEGIN{printf "%.2f", (100.0*(td-id))/td}')"
  fi
  prev_total="${total_now}"
  prev_idle="${idle_now}"

  mem_total_kb="$(awk '/^MemTotal:/{print $2}' /proc/meminfo)"
  mem_avail_kb="$(awk '/^MemAvailable:/{print $2}' /proc/meminfo)"
  swap_total_kb="$(awk '/^SwapTotal:/{print $2}' /proc/meminfo)"
  swap_free_kb="$(awk '/^SwapFree:/{print $2}' /proc/meminfo)"

  mem_used_kb=$((mem_total_kb - mem_avail_kb))
  swap_used_kb=$((swap_total_kb - swap_free_kb))

  mem_used_pct="$(awk -v u="${mem_used_kb}" -v t="${mem_total_kb}" 'BEGIN{if(t<=0){print "0.00"}else{printf "%.2f", (100.0*u)/t}}')"
  mem_used_mb="$(awk -v k="${mem_used_kb}" 'BEGIN{printf "%.1f", k/1024.0}')"
  mem_total_mb="$(awk -v k="${mem_total_kb}" 'BEGIN{printf "%.1f", k/1024.0}')"
  swap_used_mb="$(awk -v k="${swap_used_kb}" 'BEGIN{printf "%.1f", k/1024.0}')"
  swap_total_mb="$(awk -v k="${swap_total_kb}" 'BEGIN{printf "%.1f", k/1024.0}')"

  read -r load1 load5 load15 running_total _ < /proc/loadavg
  procs_running="${running_total%%/*}"
  procs_total="${running_total##*/}"

  piper_count="$(pgrep -fc 'piper_ctrl_node.py' || true)"
  rs_count="$(pgrep -fc 'realsense2_camera' || true)"
  opp_count="$(pgrep -fc 'opp_usb_camera_pub.py' || true)"
  rviz_count="$(pgrep -fc 'rviz' || true)"
  roslaunch_count="$(pgrep -fc 'roslaunch' || true)"

  ros_up="0"
  ros_nodes="0"
  ros_topics="0"
  if ros_master_up; then
    ros_up="1"
    ros_nodes="$(rosnode list 2>/dev/null | wc -l | tr -d ' ')"
    ros_topics="$(rostopic list 2>/dev/null | wc -l | tr -d ' ')"
  fi

  printf "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
    "${iso_now}" "${epoch_now}" "${cpu_pct}" "${mem_used_pct}" "${mem_used_mb}" "${mem_total_mb}" \
    "${swap_used_mb}" "${swap_total_mb}" "${load1}" "${load5}" "${load15}" "${procs_running}" \
    "${procs_total}" "${piper_count}" "${rs_count}" "${opp_count}" "${rviz_count}" \
    "${roslaunch_count}" "${ros_up}" "${ros_nodes}" "${ros_topics}" >> "${SAMPLE_CSV}"

  printf "%s\n" "${iso_now}" > "${HEARTBEAT_TXT}"

  if [[ "${epoch_now}" -ge "${next_snapshot_epoch}" ]]; then
    snapshot_heavy
    next_snapshot_epoch="$(awk -v now="${epoch_now}" -v step="${SNAPSHOT_SEC}" 'BEGIN{printf "%.0f", now + step}')"
  fi

  sleep "${SAMPLE_SEC}"
done
