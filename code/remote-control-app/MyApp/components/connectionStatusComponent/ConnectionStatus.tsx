import { useEffect, useRef, useState } from "react";
import { StyleSheet, Text, TouchableOpacity, View } from "react-native";
import { useSocketStatus } from "../../hooks/useSocketStatus";
import { isConnected } from "../../services/socket";
import { useRobotStore } from "../../store/robotStore";

export function ConnectionStatus(){
    const connected = useSocketStatus();
    const requestReconnect = useRobotStore((s) => s.requestReconnect);
    const [retrying, setRetrying] = useState(false);
    const [failed, setFailed] = useState(false);
    const timers = useRef<ReturnType<typeof setTimeout>[]>([]);

    useEffect(() => () => { timers.current.forEach(clearTimeout); }, []);

    const onRetry = () => {
        if (connected || retrying) return;
        setFailed(false);
        setRetrying(true);
        requestReconnect();
        // Give the socket ~2.5s to open; if it didn't, show a brief failure hint.
        timers.current.push(setTimeout(() => {
            setRetrying(false);
            if (!isConnected()) {
                setFailed(true);
                timers.current.push(setTimeout(() => setFailed(false), 2500));
            }
        }, 2500));
    };

    const bg = connected ? '#18e002' : retrying ? '#e0a402' : '#e02b02';
    const label = connected
        ? "Connected"
        : retrying
        ? "Retrying…"
        : failed
        ? "Couldn't connect — tap to retry"
        : "Not Connected — tap to retry";

    return(
        <View style={styles.mainContainer}>
            <TouchableOpacity
                style={{...styles.connectionInfo, backgroundColor: bg}}
                onPress={onRetry}
                disabled={connected || retrying}
                activeOpacity={0.7}
            >
                <Text style={styles.connectionInfoText}>{label}</Text>
            </TouchableOpacity>
        </View>
    );
}

const styles = StyleSheet.create({
    mainContainer:{
        width: '100%',
        height:'100%',
        justifyContent: 'center',
        alignItems: 'center',
        paddingHorizontal: 16,
    },
    connectionInfo: {
        justifyContent: 'center',
        alignItems:'center',
        display: 'flex',
        paddingHorizontal: 24,
        paddingVertical: 10,
        margin: 8,
        borderRadius: 10
    },
    connectionInfoText: {
        fontSize: 18,
        color: 'white'
    }
});
