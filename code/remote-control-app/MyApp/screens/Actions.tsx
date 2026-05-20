import { useState } from "react";
import { StyleSheet, View } from "react-native";
import { IndividualSelectionButton } from "../components/IndividualSelectionButton/IndividualSelectionButton";
import { AppText } from "../components/text/AppText";
import { sendCommand } from "../services/socket";
import { InvertRobotPacket } from "../api/api-messages";

export function Actions() {
    const [pendingInvert, setPendingInvert] = useState(false);

    const onConfirmInvert = () => {
        sendCommand(JSON.stringify(InvertRobotPacket));
        setPendingInvert(false);
    };

    return (
        <View style={styles.mainContainer}>
            <View style={styles.actionRow}>
                <IndividualSelectionButton
                    selected={pendingInvert}
                    title="Invert robot"
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
    confirmRow: {
        gap: 10,
        alignItems: 'flex-start',
    },
});
