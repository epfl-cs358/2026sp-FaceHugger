import { useState } from "react";
import { Pressable, ScrollView, StyleSheet, TextInput, View } from "react-native";
import { AppText } from "../components/text/AppText";
import { orangeColor } from "../colors/colors";
import { useRobotStore } from "../store/robotStore";
import { ROBOT_IP, ROBOT_PORT, SIM_IP, SIM_PORT } from "../config/config";
import { useSocketStatus } from "../hooks/useSocketStatus";

// Connection settings: pick which WebSocket the app drives. Editing the fields
// (or tapping a preset) updates the store, which makes useRobotConnection tear
// down the current socket and reconnect to the new target.
export function Settings() {
    const connIP = useRobotStore((s) => s.connIP);
    const connPort = useRobotStore((s) => s.connPort);
    const setConnection = useRobotStore((s) => s.setConnection);
    const connected = useSocketStatus();

    const [ip, setIp] = useState(connIP);
    const [port, setPort] = useState(String(connPort));

    const applyPreset = (nextIp: string, nextPort: number) => {
        setIp(nextIp);
        setPort(String(nextPort));
        setConnection(nextIp, nextPort);
    };

    const onConnect = () => {
        const trimmed = ip.trim();
        const p = Number(port);
        if (!trimmed || !Number.isInteger(p) || p <= 0 || p > 65535) return;
        setConnection(trimmed, p);
    };

    const isActive = (i: string, p: number) => connIP === i && connPort === p;

    return (
        <ScrollView contentContainerStyle={styles.container}>
            <AppText text="Connection" size={22} color="#ffffff" />

            <View style={styles.statusRow}>
                <View style={[styles.dot, { backgroundColor: connected ? "#4caf50" : "#ff7043" }]} />
                <AppText
                    text={connected ? `Connected to ws://${connIP}:${connPort}` : `Not connected (target ws://${connIP}:${connPort})`}
                    size={13}
                    color={connected ? "#4caf50" : "#ff8a65"}
                />
            </View>

            <AppText text="Quick presets" size={13} color="#aaaaaa" />
            <View style={styles.presetRow}>
                <Pressable
                    style={[styles.preset, isActive(ROBOT_IP, ROBOT_PORT) && styles.presetActive]}
                    onPress={() => applyPreset(ROBOT_IP, ROBOT_PORT)}
                >
                    <AppText text="Robot" size={16} color="#ffffff" />
                    <AppText text={`${ROBOT_IP}:${ROBOT_PORT}`} size={12} color="#aaaaaa" />
                </Pressable>
                <Pressable
                    style={[styles.preset, isActive(SIM_IP, SIM_PORT) && styles.presetActive]}
                    onPress={() => applyPreset(SIM_IP, SIM_PORT)}
                >
                    <AppText text="Simulator" size={16} color="#ffffff" />
                    <AppText text={`${SIM_IP}:${SIM_PORT}`} size={12} color="#aaaaaa" />
                </Pressable>
            </View>

            <AppText text="IP address" size={13} color="#aaaaaa" />
            <TextInput
                style={styles.input}
                value={ip}
                onChangeText={setIp}
                autoCapitalize="none"
                autoCorrect={false}
                keyboardType="numbers-and-punctuation"
                placeholder="192.168.4.1"
                placeholderTextColor="#555555"
            />

            <AppText text="Port" size={13} color="#aaaaaa" />
            <TextInput
                style={styles.input}
                value={port}
                onChangeText={(t) => setPort(t.replace(/[^0-9]/g, "").slice(0, 5))}
                keyboardType="number-pad"
                placeholder="81"
                placeholderTextColor="#555555"
            />

            <Pressable style={styles.connectBtn} onPress={onConnect}>
                <AppText text="Connect" size={16} color="#000000" />
            </Pressable>
        </ScrollView>
    );
}

const styles = StyleSheet.create({
    container: {
        padding: 24,
        gap: 12,
    },
    statusRow: {
        flexDirection: "row",
        alignItems: "center",
        gap: 8,
        marginBottom: 8,
    },
    dot: {
        width: 10,
        height: 10,
        borderRadius: 5,
    },
    presetRow: {
        flexDirection: "row",
        gap: 12,
        marginBottom: 8,
    },
    preset: {
        flex: 1,
        gap: 2,
        paddingVertical: 12,
        paddingHorizontal: 16,
        borderRadius: 12,
        backgroundColor: "#2a2a2a",
        borderWidth: 2,
        borderColor: "transparent",
    },
    presetActive: {
        borderColor: orangeColor,
    },
    input: {
        color: "#ffffff",
        backgroundColor: "#1e1e1e",
        borderWidth: 2,
        borderColor: "#2a2a2a",
        borderRadius: 10,
        paddingHorizontal: 14,
        paddingVertical: 10,
        fontSize: 16,
    },
    connectBtn: {
        marginTop: 12,
        alignItems: "center",
        backgroundColor: orangeColor,
        borderRadius: 12,
        paddingVertical: 14,
    },
});
