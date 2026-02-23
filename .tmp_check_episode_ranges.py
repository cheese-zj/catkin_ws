import sys
from pathlib import Path
import rosbag

TOPICS = [
    '/robot/arm_left/joint_states_single',
    '/robot/arm_right/joint_states_single',
    '/teleop/arm_left/joint_states_single',
    '/teleop/arm_right/joint_states_single',
    '/realsense_top/color/image_raw/compressed',
    '/realsense_left/color/image_raw/compressed',
    '/realsense_right/color/image_raw/compressed',
]

def msg_time_sec(msg, bag_time):
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is not None:
        s = float(stamp.to_sec())
        if s > 0.0:
            return s
    return float(bag_time.to_sec())

session_dir = Path('/home/jameszhao2004/catkin_ws/data/rosbags/act_20260220_220456')
for i in range(6, 13):
    ep = session_dir / f'episode_{i:03d}' / 'episode.bag'
    if not ep.is_file():
        continue
    ranges = {t: [None, None, 0] for t in TOPICS}
    with rosbag.Bag(str(ep), 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=TOPICS):
            ts = msg_time_sec(msg, t)
            r = ranges[topic]
            if r[0] is None:
                r[0] = ts
            r[1] = ts
            r[2] += 1
    print(f'=== episode_{i:03d} ===')
    starts = []
    ends = []
    for topic in TOPICS:
        s, e, n = ranges[topic]
        print(f'{topic} count={n} start={s} end={e}')
        if n > 0:
            starts.append(s)
            ends.append(e)
    if starts and ends:
        print(f'overlap_start={max(starts)} overlap_end={min(ends)}')
    print()
