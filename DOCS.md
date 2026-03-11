# Documentation Map

This repository now uses **root-level docs as the single source of truth**.

## Canonical Docs (Edit These)

- `/home/jameszhao2004/catkin_ws/README.md`: top-level router and system overview
- `/home/jameszhao2004/catkin_ws/WORKSPACES.md`: workspace architecture and build/source rules
- `/home/jameszhao2004/catkin_ws/LAUNCH_RUNBOOK.md`: daily 2-arm launch runbook (EN)
- `/home/jameszhao2004/catkin_ws/LAUNCH_RUNBOOK_CN.md`: daily 2-arm launch runbook (CN)
- `/home/jameszhao2004/catkin_ws/RUNBOOK_3ARM_INTERVENTION.md`: 3-arm intervention runbook
- `/home/jameszhao2004/catkin_ws/DOCKER_SETUP_README.md`: host/container setup
- `/home/jameszhao2004/catkin_ws/README_burst_monitor.md`: burst pipeline operations

## Compatibility Redirects (Do Not Maintain)

These files are kept only so old links still work:

- `workspaces/README.md`
- `workspaces/LAUNCH_RUNBOOK.md`
- `workspaces/LAUNCH_RUNBOOK_CN.md`
- `workspaces/RUNBOOK_3ARM_INTERVENTION.md`

They should contain only pointers to the root-level docs above.

## Working Rule

When adding/updating operational docs:

1. Update the root canonical file only.
2. If needed, add one line in `README.md` and `DOCS.md` to expose it.
3. Keep deep-path docs as pointer stubs, not full copies.
