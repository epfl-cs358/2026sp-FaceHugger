import { useState } from "react";
import { StyleSheet, View } from "react-native";
import { IndividualSelectionButton } from "../components/IndividualSelectionButton/IndividualSelectionButton";
import { AppText } from "../components/text/AppText";
import { ClipList } from "../components/ClipList";
import { sendCommand } from "../services/socket";
import { useRobotStore } from "../store/robotStore";
import { InvertRobotPacket, restAllServosPackets, neutralStancePackets } from "../api/api-messages";

export function Actions() {
    const [pendingInvert, setPendingInvert] = useState(false);
    const inverted = useRobotStore((s) => s.inverted);
    const setInverted = useRobotStore((s) => s.setInverted);

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
            <View style={styles.invertRow}>
                <IndividualSelectionButton
                    selected={pendingInvert || inverted}
                    title={inverted ? "Inverted (tap to flip back)" : "Invert robot"}
                    onClick={() => setPendingInvert(prev => !prev)}
                />
                {pendingInvert && (
                    <View style={styles.confirmRow}>
                        <AppText text="Are you sure you want to invert the robot?" size={14} color={'#ffffff'} />
                        <IndividualSelectionButton
                            selected={true}
                            title="Confirm invert"
                            onClick={onConfirmInvert}
                        />
                    </View>
                )}
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
    invertRow: {
        flexDirection: 'row',
        gap: 12,
        alignItems: 'center',
        flexWrap: 'wrap',
    },
    confirmRow: {
        flexDirection: 'row',
        gap: 10,
        alignItems: 'center',
    },
});
