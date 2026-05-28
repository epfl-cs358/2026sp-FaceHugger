import { useEffect, useState } from "react";
import { StyleSheet, View } from "react-native";
import { IndividualSelectionButton } from "../components/IndividualSelectionButton/IndividualSelectionButton";
import { AppText } from "../components/text/AppText";
import { ClipList } from "../components/ClipList";
import { useRobotStore } from "../store/robotStore";
import { sendRestPose, sendNeutralStance, setClipSmoothing, stopMotion } from "../api/api-messages";
import { formatPitch, formatRoll, formatOrientationState } from "../api/orientation";

// Clip-playback smoothing presets (T:11 EMA alpha): snappy follows the raw
// frames, smooth lags and rounds the motion.
const SMOOTHING_PRESETS: { label: string; alpha: number }[] = [
    { label: "Snappy", alpha: 0.3 },
    { label: "Normal", alpha: 0.75 },
    { label: "Smooth", alpha: 0.9 },
];

export function Actions() {
    const [smoothing, setSmoothing] = useState(0.75);

    // Orientation tile is driven from the T:10 broadcast (~10 Hz) that the
    // connection hook already consumes — no polling fetch is added here, the
    // store re-renders us when the firmware pushes a new frame. Fields may be
    // null on pre-IMU firmware; the formatters render "—" in that case.
    const pitchDeg = useRobotStore((s) => s.pitchDeg);
    const rollDeg = useRobotStore((s) => s.rollDeg);
    const upsideDown = useRobotStore((s) => s.upsideDown);

    // Pager swaps pages by unmount, so the cleanup fires on blur. Send IDLE
    // so any in-flight firmware clip (T:7) stops when leaving the page, and
    // tear down the JS clip streamer if it happened to be running.
    useEffect(() => () => stopMotion(), []);

    const onSmoothing = (alpha: number) => {
        setSmoothing(alpha);
        setClipSmoothing(alpha);
    };

    // Pose buttons (Task #11). Both REST (flat / all-90 calibration) and NEUTRAL
    // (standing) are firmware-side states; the firmware owns the pose values and
    // the ease. invert-aware ease lives on the firmware side (easeToNeutral
    // routes through applyInvert + the IMU latch).
    const onRestPose = () => sendRestPose();
    const onNeutralStance = () => sendNeutralStance();

    return (
        <View style={styles.mainContainer}>
            <View style={styles.orientationTile}>
                <AppText text="Orientation" size={13} color="#aaa" />
                <View style={styles.orientationRow}>
                    <AppText text={formatPitch(pitchDeg)} size={15} color="#ffffff" />
                    <AppText text={formatRoll(rollDeg)} size={15} color="#ffffff" />
                </View>
                <AppText
                    text={formatOrientationState(upsideDown)}
                    size={14}
                    color={upsideDown === true ? "#ff9966" : "#a0d8a0"}
                />
            </View>
            <View style={styles.actionRow}>
                <IndividualSelectionButton
                    selected={false}
                    title="Rest pose (all servos 90°)"
                    onClick={onRestPose}
                />
            </View>
            <View style={styles.actionRow}>
                <IndividualSelectionButton
                    selected={false}
                    title="Neutral stance"
                    onClick={onNeutralStance}
                />
            </View>
            <View style={styles.smoothingRow}>
                <AppText text="Clip smoothing" size={13} color="#aaa" />
                <View style={styles.smoothingButtons}>
                    {SMOOTHING_PRESETS.map((p) => (
                        <IndividualSelectionButton
                            key={p.label}
                            selected={smoothing === p.alpha}
                            title={p.label}
                            onClick={() => onSmoothing(p.alpha)}
                        />
                    ))}
                </View>
            </View>

            <ClipList />
        </View>
    );
}

const styles = StyleSheet.create({
    mainContainer: {
        flex: 1,
        paddingVertical: 20,
        paddingHorizontal: 20,
        gap: 16,
        alignItems: 'flex-start',
    },
    orientationTile: {
        alignSelf: 'stretch',
        backgroundColor: '#1a1a2e',
        borderRadius: 10,
        paddingVertical: 10,
        paddingHorizontal: 14,
        gap: 6,
    },
    orientationRow: {
        flexDirection: 'row',
        gap: 16,
    },
    actionRow: {
        gap: 12,
        alignItems: 'flex-start',
    },
    smoothingRow: {
        gap: 8,
        alignItems: 'flex-start',
    },
    smoothingButtons: {
        flexDirection: 'row',
        gap: 8,
    },
});
