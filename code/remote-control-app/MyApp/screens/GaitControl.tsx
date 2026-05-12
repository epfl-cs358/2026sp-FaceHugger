import { StyleSheet, View } from "react-native";
import Joystick, { JoystickAction, JoystickProps } from "../components/joystick/Joystick";
import { useRobotConnection } from "../hooks/useRobotConnection";
import { BLMovementPacket, BRMovementPacket, BWMovementPacket, FLMovementPacket, FRMovementPacket, FWMovementPacket, LeftMovementPacket, RightMovementPacket } from "../api/api-messages";
import { ConnectionStatus } from "../components/connectionStatusComponent/ConnectionStatus";
import { GaitMode } from "../api/api-types";
import { DropDownMenu, DropDownMenuElement, DropDownMenuProps } from "../components/dropdown/DropdownMenu";
import { useRobotStore } from "../store/robotStore";
import { webSocketIP } from "../config/config";

export default function GaitControl(){
    const { sendCommand } = useRobotConnection(webSocketIP);
    const setChosenGaitMode = useRobotStore((s) => s.setChosenGaitMode);

    const actionRight = {
        minAngle: -15,
        maxAngle: 15,
        action: () => sendCommand(JSON.stringify(RightMovementPacket))
    } as JoystickAction;

    const actionLeft = {
        minAngle: 165,
        maxAngle: -165,
        action: () => sendCommand(JSON.stringify(LeftMovementPacket))
    } as JoystickAction;

    const actionForwardRight = {
        minAngle: 15,
        maxAngle: 75,
        action: () => sendCommand(JSON.stringify(FRMovementPacket))
    } as JoystickAction;

    const actionForward = {
        minAngle: 75,
        maxAngle: 105,
        action: () => sendCommand(JSON.stringify(FWMovementPacket))
    } as JoystickAction;

    const actionForwardLeft = {
        minAngle: 105, 
        maxAngle: 165,
        action: () => sendCommand(JSON.stringify(FLMovementPacket))
    } as JoystickAction;

    const actionBackwardLeft = {
        minAngle: -165,
        maxAngle: -105,
        action: () => sendCommand(JSON.stringify(BLMovementPacket))
    } as JoystickAction;

    const actionBackward = {
        minAngle: -105,
        maxAngle: -75,
        action: () => sendCommand(JSON.stringify(BWMovementPacket))
    }

    const actionBackwardRight = {
        minAngle: -75,
        maxAngle: -15,
        action: () => sendCommand(JSON.stringify(BRMovementPacket))
    }

    const gestureProps = {
            actions: [actionLeft, actionRight, actionForwardLeft, actionForward, actionForwardRight, actionBackwardLeft, actionBackward, actionBackwardRight]
    } as JoystickProps;

    //Gait mode drop down elements

    const dropDownElements = Object.keys(GaitMode)
    .filter(key => isNaN(Number(key)))
    .map(key => ({
        key: key,
        elementTitle: key,
        onClick: () => setChosenGaitMode(GaitMode[key as keyof typeof GaitMode])
    } as DropDownMenuElement));

    const dropdownMenuProps = {
        defaultElement: dropDownElements[0],
        elements: dropDownElements
    } as DropDownMenuProps;

    return(
        <View style={styles.mainContainer}>
            <View style={styles.joystickContainer}>
                <Joystick {...gestureProps}/>
            </View>
            <View style={styles.gaitModeSelectionContainer}>
                <DropDownMenu {...dropdownMenuProps}/>
            </View>
        </View>
    );
}

const styles = StyleSheet.create({
    mainContainer: {
        width: '100%',
        height: '100%',
        display: 'flex'
    },
    gaitModeSelectionContainer: {
        flex: 1,
        alignItems: 'flex-end',
        justifyContent:'flex-end'
    },
    joystickContainer: {
        flex: 8,
        width: '100%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center'
    }
});