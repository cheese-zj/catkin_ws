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

session = Path('/home/jameszhao2004/catkin_ws/data/rosbags/act_20260220_220456')

ok = 0
bad_missing = 0
bad_overlap = 0
bad_nonmono = 0
bad_eps = []

for ep in sorted([p for p in session.iterdir() if p.is_dir() and p.name.startswith('episode_')]):
    bag_path = ep / 'episode.bag'
    if not bag_path.is_file():
        continue
    stats = {t: {'count': 0, 'start': None, 'end': None, 'last': None, 'nonmono': 0} for t in TOPICS}
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, msg, bt in bag.read_messages(topics=TOPICS):
            ts = float(bt.to_sec())
            st = stats[topic]
            if st['start'] is None:
                st['start'] = ts
            if st['last'] is not None and ts < st['last']:
                st['nonmono'] += 1
            st['last'] = ts
            st['end'] = ts
            st['count'] += 1

    missing = [t for t in TOPICS if stats[t]['count'] == 0]
    nonmono = sum(stats[t]['nonmono'] for t in TOPICS)
    if missing:
        bad_missing += 1
        bad_eps.append((ep.name, 'missing', missing))
        continue
    starts = [stats[t]['start'] for t in TOPICS]
    ends = [stats[t]['end'] for t in TOPICS]
    overlap = min(ends) - max(starts)
    if overlap <= 0:
        bad_overlap += 1
        bad_eps.append((ep.name, 'overlap', overlap))
        continue
    if nonmono > 0:
        bad_nonmono += 1
        bad_eps.append((ep.name, 'nonmono', nonmono))
        continue
    ok += 1

print(f'total_episodes={ok + bad_missing + bad_overlap + bad_nonmono}')
print(f'ok={ok}')
print(f'bad_missing={bad_missing}')
print(f'bad_overlap={bad_overlap}')
print(f'bad_nonmono={bad_nonmono}')
if bad_eps:
    print('bad_examples:')
    for row in bad_eps[:20]:
        print(row)
