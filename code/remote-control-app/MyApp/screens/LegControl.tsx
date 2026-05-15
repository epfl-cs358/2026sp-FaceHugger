import { useState } from "react";
import { ScrollView, StyleSheet, View } from "react-native";

import {
    IndividualSelectionButton,
    IndividualSelectionProps
} from "../components/IndividualSelectionButton/IndividualSelectionButton";

import { AppTextInput } from "../components/textInput/AppTextInput";

import { Legs, ServoCalibration, Servos } from "../api/api-types";
import { useRobotConnection } from "../hooks/useRobotConnection";
import { webSocketIP } from "../config/config";

export default function LegControl() {

    const [selectedLeg, setSelectedLeg] = useState(Legs.FRONT_RIGHT_LEG);
    const [selectedServo, setSelectedServo] = useState(Servos.HIP_SERVO);
    const [selectedAngle, setSelectedAngle] = useState(90);

    const { sendCommand } = useRobotConnection(webSocketIP);

    const legs: IndividualSelectionProps[] = [
        {
            selected: selectedLeg === Legs.FRONT_RIGHT_LEG,
            title: "Front right leg",
            onClick: () => setSelectedLeg(Legs.FRONT_RIGHT_LEG)
        },
        {
            selected: selectedLeg === Legs.FRONT_LEFT_LEG,
            title: "Front left leg",
            onClick: () => setSelectedLeg(Legs.FRONT_LEFT_LEG)
        },
        {
            selected: selectedLeg === Legs.BACK_RIGHT_LEG,
            title: "Back right leg",
            onClick: () => setSelectedLeg(Legs.BACK_RIGHT_LEG)
        },
        {
            selected: selectedLeg === Legs.BACK_LEFT_LEG,
            title: "Back left leg",
            onClick: () => setSelectedLeg(Legs.BACK_LEFT_LEG)
        }
    ];

    const servos: IndividualSelectionProps[] = [
        {
            selected: selectedServo === Servos.HIP_SERVO,
            title: "Hip servo",
            onClick: () => setSelectedServo(Servos.HIP_SERVO)
        },
        {
            selected: selectedServo === Servos.THIGH_SERVO,
            title: "Thigh servo",
            onClick: () => setSelectedServo(Servos.THIGH_SERVO)
        },
        {
            selected: selectedServo === Servos.KNEE_SERVO,
            title: "Knee servo",
            onClick: () => setSelectedServo(Servos.KNEE_SERVO)
        }
    ];

    const sendServoAngle = () => {
        sendCommand(
            JSON.stringify({
                id: selectedLeg,
                servo_id: selectedServo,
                a: selectedAngle
            } as ServoCalibration)
        );
    };

    return (
        <View style={styles.mainContainer}>

            {/* Legs selection */}
            <ScrollView
                horizontal
                showsHorizontalScrollIndicator={false}
                style={styles.horizontalScroll}
                contentContainerStyle={styles.horizontalContent}
            >
                {legs.map((element) => (
                    <IndividualSelectionButton
                        key={element.title}
                        {...element}
                    />
                ))}
            </ScrollView>

            {/* Servo selection */}
            <View style={styles.servoButtonsContainer}>
                {servos.map((element) => (
                    <IndividualSelectionButton
                        key={element.title}
                        {...element}
                    />
                ))}
            </View>

            {/* Angle input */}
            <View
                style={styles.sendAngleView}
            >
                <AppTextInput
                    label="Write angle"
                    value={selectedAngle}
                    onChange={setSelectedAngle}
                />

                <IndividualSelectionButton
                    selected={true}
                    title="Send angle"
                    onClick={sendServoAngle}
                />
            </View>

            {/* 3D model / visualization */}
            <View style={styles.modelView} />

        </View>
    );
}

const styles = StyleSheet.create({
    mainContainer: {
        flex: 1,
        paddingVertical: 10,
        gap: 15,
        justifyContent:'center',
        alignItems:'center'
    },

    horizontalScroll: {
        maxHeight: 70,
    },

    horizontalContent: {
        flexDirection: "row",
        alignItems: "center",
        gap: 10,
        paddingHorizontal: 10,
    },

    servoButtonsContainer: {
        flexDirection: "row",
        justifyContent: "center",
        alignItems: "center",
        gap: 10,
        paddingHorizontal: 10,
    },

    angleContainer: {
        flexDirection: "row",
        alignItems: "center",
        gap: 10,
        paddingHorizontal: 10,
    },

    modelView: {
        flex: 1,
    },

    sendAngleView: {
        width: '90%',
        gap: 10,
        justifyContent: 'center',
        alignItems: 'center',
        flex: 1,
    }
});