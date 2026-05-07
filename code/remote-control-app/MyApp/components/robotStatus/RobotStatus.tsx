import { StyleSheet, Text, View } from "react-native"
import { FSMStatus, GaitMode } from "../../api/api-types"
import { AppText } from "../text/AppText";
import { useRobotStore } from "../../store/robotStore";
import { useRobotConnection } from "../../hooks/useRobotConnection";
import { useSocketStatus } from "../../hooks/useSocketStatus";
import { darkerOrangeColor } from "../../colors/colors";

export function RobotStatus(){
    const robotStore = useRobotStore();
    const isConnected = useSocketStatus();
    
    if(isConnected){
        return(<View style={styles.mainContainer}>
        <View style={styles.gateAndSpeedContainer}>
            <View style={styles.blueButtonStyling}>
                <AppText text={`Gate mode: ${GaitMode[robotStore.gaitMode]}`} size={15}/>
            </View>
            <View style={styles.blueButtonStyling}>
                <AppText text={`Speed (m/min): ${robotStore.speed.toFixed(1)}`} size={15}/>
            </View>
        </View>
        <View style={styles.gyroscopeDataContainer}>
            <View style={styles.orangeButtonStyling}>
                <AppText text={`Gyroscope: `} size={15}/>
            </View>
            <View style={styles.orangeButtonStyling}>
                <AppText text={`X: ${robotStore.gyroscope[0].toFixed(1)}`} size={15}/>
            </View>
            <View style={styles.orangeButtonStyling}>
                <AppText text={`Y: ${robotStore.gyroscope[1].toFixed(1)}`} size={15}/>
            </View>
            <View style={styles.orangeButtonStyling}>
                <AppText text={`Z: ${robotStore.gyroscope[2].toFixed(1)}`} size={15}/>
            </View>
        </View>
        <View style={styles.errorMessageContainer}>
            {robotStore.errorMessage != null && <AppText text={`Error message: ${robotStore.errorMessage}`} size={12} color="red"/>}
        </View>
    </View>);
    }else{
        return <View></View>
    }

}

const styles = StyleSheet.create({
    mainContainer: {
        width:'100%',
        display: 'flex',
        flexDirection:'column',
        gap: 5
    },
    gateAndSpeedContainer: {
        width: '100%',
        display: 'flex',
        flexDirection: 'row',
        justifyContent: 'center',
        alignItems: 'center',
        gap: 50
    },
    gyroscopeDataContainer: {
        width: '100%',
        display: 'flex',
        flexDirection: 'row',
        justifyContent: 'center',
        alignItems: 'center',
        gap: 50

    },
    errorMessageContainer: {
        justifyContent: 'center',
        alignItems: 'center',
        width: '100%',
        display: 'flex'
    },
    orangeButtonStyling: {
        backgroundColor: darkerOrangeColor,
        borderRadius: 10,
        shadowColor: '#fff',
        shadowOffset: { width: 0, height: 0 },
        shadowOpacity: 0.5,
        shadowRadius: 6,
        elevation: 5,
        padding: 5,
        justifyContent: 'center',
        alignItems: 'center'
    },
    blueButtonStyling: {
        backgroundColor: 'blue',
        borderRadius: 10,
        shadowColor: '#fff',
        shadowOffset: { width: 0, height: 0 },
        shadowOpacity: 0.5,
        shadowRadius: 6,
        elevation: 5,
        padding: 5,
        justifyContent: 'center',
        alignItems: 'center'
    }
});