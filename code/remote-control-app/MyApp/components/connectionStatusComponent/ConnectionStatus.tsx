import { StyleSheet, Text, View } from "react-native";
import { useSocketStatus } from "../../hooks/useSocketStatus";

export function ConnectionStatus(){
    const connectionStatus = useSocketStatus();
    return(
        <View style={styles.mainContainer}>
            <View style={{...styles.connectionInfo, backgroundColor: connectionStatus ? '#18e002' : '#e02b02'}}>
                <Text style={styles.connectionInfoText}>
                    {connectionStatus ? "Connected" : "Not Connected"}
                </Text>
            </View>
        </View>
    );
}

const styles = StyleSheet.create({
    mainContainer:{
        width: '100%',
        height:'100%',
        justifyContent: 'center',
        alignItems: 'center'
    },
    connectionInfo: {
        justifyContent: 'center',
        alignItems:'center',
        display: 'flex',
        width: 200,
        height: 50,
        margin: 5,
        padding: 5,
        borderRadius: 10
    },
    connectionInfoText: {
        fontSize: 18,
        color: 'white'
    }
});