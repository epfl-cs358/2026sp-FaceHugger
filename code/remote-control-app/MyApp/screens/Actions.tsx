import { useEffect, useState } from "react";
import { StyleSheet, View } from "react-native";
import { IndividualSelectionButton } from "../components/IndividualSelectionButton/IndividualSelectionButton";
import { AppText } from "../components/text/AppText";
import { ClipList } from "../components/ClipList";
import { useRobotStore } from "../store/robotStore";
import { sendRestPose, sendNeutralStance, sendSetAutoInvert, setClipSmoothing, stopMotion } from "../api/api-messages";

// Clip-playback smoothing presets (T:11 EMA alpha): snappy follows the raw
// frames, smooth lags and rounds the motion.
const SMOOTHING_PRESETS: { label: string; alpha: number }[] = [
    { label: "Snappy", alpha: 0.3 },
    { label: "Normal", alpha: 0.75 },
    { label: "Smooth", alpha: 0.9 },
];

export function Actions() {
    const [smoothing, setSmoothing] = useState(0.75);
    const autoFlipEnabled = useRobotStore((s) => s.autoFlipEnabled);
    const setAutoFlipEnabled = useRobotStore((s) => s.setAutoFlipEnabled);

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

    // Auto-flip (IMU) toggle. The store update is optimistic so the UI feels
    // responsive; the next T:10 frame will reconcile (firmware mirrors the
    // setter back via auto_invert_enabled).
    const onToggleAutoFlip = () => {
        const next = !autoFlipEnabled;
        setAutoFlipEnabled(next);
        sendSetAutoInvert(next);
    };

    return (
        <View style={styles.mainContainer}>
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
            <View style={styles.actionRow}>
                <IndividualSelectionButton
                    selected={autoFlipEnabled}
                    title={autoFlipEnabled ? "Auto-flip (IMU): ON" : "Auto-flip (IMU): OFF"}
                    onClick={onToggleAutoFlip}
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
