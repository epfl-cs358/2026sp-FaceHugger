// Display formatters for the IMU orientation tile (Actions screen).
//
// Pure helpers — no React, no store, no socket — so the jest test can drive them
// with a synthetic T:10 payload without mounting the screen. The tile pulls
// pitch / roll / upsideDown out of the robot store (set by useRobotConnection
// from the T:10 stream); these helpers turn null/undefined into a stable "—"
// placeholder so old firmware that omits the orientation fields still renders
// the tile cleanly.

// "+12.3°" / "-5.7°" — always shows the sign + one decimal place. null/undefined
// (pre-IMU firmware or before the first T:10) collapses to "—".
export const formatAngle = (deg: number | null | undefined): string => {
    if (deg === null || deg === undefined || Number.isNaN(deg)) return "—";
    const sign = deg >= 0 ? "+" : "-";
    return `${sign}${Math.abs(deg).toFixed(1)}°`;
};

// Full "Pitch: +12.3°" / "Pitch: —" line.
export const formatPitch = (deg: number | null | undefined): string =>
    `Pitch: ${formatAngle(deg)}`;

export const formatRoll = (deg: number | null | undefined): string =>
    `Roll: ${formatAngle(deg)}`;

// "Upright" / "Inverted" / "—" — boolean has no canonical sign formatter, so
// null/undefined falls through to the same placeholder for visual consistency.
export const formatOrientationState = (
    upsideDown: boolean | null | undefined,
): string => {
    if (upsideDown === null || upsideDown === undefined) return "—";
    return upsideDown ? "Inverted" : "Upright";
};
