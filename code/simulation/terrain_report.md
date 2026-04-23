# FaceHugger Terrain Survival Matrix

Course (spawn at y = -0.10, all obstacles along +Y):
- **UP_RAMP** at y = 0.20 m (tilt budget 25 deg)
- **PLATEAU** at y = 0.45 m (tilt budget 20 deg)
- **DOWN_RAMP** at y = 0.70 m (tilt budget 30 deg)
- **BUMPS** at y = 0.95 m (tilt budget 30 deg)
- **STEP_UP** at y = 1.40 m (tilt budget 45 deg)
- **FINISH** at y = 1.70 m (tilt budget 25 deg)

| Gait  | UP_RAMP | PLATEAU | DOWN_RAMP | BUMPS | STEP_UP | FINISH | Status   | Reached y (m) | Min z (m) | Max tilt | Time (s) |
|-------|---------|---------|-----------|-------|---------|--------|----------|---------------|-----------|----------|----------|
| walk  | ok      | ok      | ok        | ok    | -       | -      | TIMEOUT  | +1.10         | +0.111    | 13deg    | 60.0     |
| trot  | ok      | ok      | ok        | ok    | ok      | ok     | FINISHED | +1.70         | +0.109    | 11deg    | 13.7     |
| bound | ok      | ok      | ok        | -     | -       | -      | TIMEOUT  | +0.91         | +0.097    | 16deg    | 40.0     |

**Legend**
- `ok`       -- checkpoint crossed within tilt budget
- `! Xdeg`   -- crossed but body tilt X deg exceeded budget
- `-`        -- never reached (timeout, fall, or wrong direction)
- `FINISHED` -- reached every checkpoint upright
- `TIMEOUT`  -- ran out of sim time before finish
- `REVERSED` -- gait pushed the robot backward (-Y)
- `STUCK`    -- never left the spawn neighbourhood
- `FELL @ X` -- body tipped or dragged before crossing X
