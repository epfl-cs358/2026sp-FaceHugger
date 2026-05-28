import { useState } from "react";
import { Modal, Pressable, StyleSheet, TouchableOpacity, View } from "react-native";
import { IndividualSelectionButton } from "../components/IndividualSelectionButton/IndividualSelectionButton";
import { AppText } from "../components/text/AppText";
import { ClipList } from "../components/ClipList";
import { sendCommand } from "../services/socket";
import { useRobotStore } from "../store/robotStore";
import { orangeColor } from "../colors/colors";
import { InvertRobotPacket, restAllServosPackets, neutralStancePackets, setClipSmoothing } from "../api/api-messages";

// Clip-playback smoothing presets (T:11 EMA alpha): snappy follows the raw
// frames, smooth lags and rounds the motion.
const SMOOTHING_PRESETS: { label: string; alpha: number }[] = [
    { label: "Snappy", alpha: 0.3 },
    { label: "Normal", alpha: 0.75 },
    { label: "Smooth", alpha: 0.9 },
];

export function Actions() {
    const [pendingInvert, setPendingInvert] = useState(false);
    const [smoothing, setSmoothing] = useState(0.75);
    const inverted = useRobotStore((s) => s.inverted);
    const setInverted = useRobotStore((s) => s.setInverted);

    const onSmoothing = (alpha: number) => {
        setSmoothing(alpha);
        setClipSmoothing(alpha);
    };

    // Flip the robot. T:6 mirrors firmware-driven motion (gaits, flashed clips);
    // the app-side flag mirrors app-streamed clips, which send raw T:4 angles
    // the firmware flag can't touch. Toggle both so invert is consistent
    // whichever way a clip is playing.
    const onConfirmInvert = () => {
        sendCommand(JSON.stringify(InvertRobotPacket));
        setInverted(!inverted);
        setPendingInvert(false);
    };

    // Reset every servo to 90° with one CMD_CALIBRATE (T:4) packet per joint.
    // Mirrored when inverted (no-op at 90°, but kept consistent with the rest).
    const onRestPose = () => {
        restAllServosPackets(90, inverted).forEach(pkt => sendCommand(JSON.stringify(pkt)));
    };

    // Drive every servo to the neutral standing pose (one T:4 packet per joint),
    // mirrored when inverted so the stance matches a flipped robot.
    const onNeutralStance = () => {
        neutralStancePackets(inverted).forEach(pkt => sendCommand(JSON.stringify(pkt)));
    };

    return (
        <View style={styles.mainContainer}>
            <View style={styles.actionRow}>
                <IndividualSelectionButton
                    selected={inverted}
                    title={inverted ? "Inverted (tap to flip back)" : "Invert robot"}
                    onClick={() => setPendingInvert(true)}
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

            {/* Confirm as a bottom sheet so it doesn't reflow the action list. */}
            <Modal
                visible={pendingInvert}
                transparent
                animationType="slide"
                onRequestClose={() => setPendingInvert(false)}
            >
                <Pressable style={styles.backdrop} onPress={() => setPendingInvert(false)}>
                    <Pressable style={styles.sheet} onPress={() => { /* swallow taps inside the sheet */ }}>
                        <AppText
                            text={inverted ? "Flip the robot back upright?" : "Invert the robot?"}
                            size={16}
                            color="#ffffff"
                        />
                        <View style={styles.sheetButtons}>
                            <TouchableOpacity style={styles.cancelBtn} onPress={() => setPendingInvert(false)}>
                                <AppText text="Cancel" size={15} color="#ffffff" />
                            </TouchableOpacity>
                            <TouchableOpacity style={styles.confirmBtn} onPress={onConfirmInvert}>
                                <AppText text={inverted ? "Flip back" : "Invert"} size={15} color="#121212" />
                            </TouchableOpacity>
                        </View>
                    </Pressable>
                </Pressable>
            </Modal>
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
    backdrop: {
        flex: 1,
        backgroundColor: 'rgba(0,0,0,0.5)',
        justifyContent: 'flex-end',
    },
    sheet: {
        backgroundColor: '#1a1a2e',
        paddingHorizontal: 20,
        paddingTop: 20,
        paddingBottom: 32,
        borderTopLeftRadius: 16,
        borderTopRightRadius: 16,
        gap: 16,
    },
    sheetButtons: {
        flexDirection: 'row',
        justifyContent: 'flex-end',
        gap: 12,
    },
    cancelBtn: {
        paddingVertical: 10,
        paddingHorizontal: 20,
        borderRadius: 10,
        backgroundColor: '#2a2a2a',
    },
    confirmBtn: {
        paddingVertical: 10,
        paddingHorizontal: 20,
        borderRadius: 10,
        backgroundColor: orangeColor,
    },
});
