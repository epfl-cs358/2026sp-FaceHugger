import { GestureDetector, Gesture } from "react-native-gesture-handler";
import { StyleSheet } from "react-native";
import { orangeColor } from "../../colors/colors";
import Animated, {useSharedValue, useAnimatedStyle, runOnJS, withSpring} from 'react-native-reanimated'
import { useEffect } from "react";

export interface JoystickAction{
    minAngle: number
    maxAngle: number //angles should be in the -180 180 range
    action: () => void
}

export interface JoystickProps{
    actions: JoystickAction[]
}

const backgroundWidth = 200;
const backgroundHeight = 200;
const maxRadius = (backgroundWidth/2);

export default function Joystick({actions}: JoystickProps){
    const translationX = useSharedValue(0);
    const translationY = useSharedValue(0);

    useEffect(() => {
        const interval = setInterval(() => {
        const distance = Math.sqrt(translationX.value ** 2 + translationY.value ** 2);
        const angle = Math.atan2(-translationY.value, translationX.value) * 180 / Math.PI;
        if (distance >= 0.8 * maxRadius){
            const matched = actions.find((a) => isAngleInRange(a.minAngle, a.maxAngle, angle));
            if (matched) matched.action();
        }
    }, 100); // fires every 100ms while held

        return () => clearInterval(interval);
    }, [actions]);

    const pan = Gesture.Pan()
        .onUpdate((e) => {
            const distance = Math.sqrt(e.translationX ** 2 + e.translationY ** 2);
            const angle = Math.atan2(e.translationY, e.translationX);

            const clampedDistance = Math.min(distance, maxRadius);
            translationX.value = clampedDistance*Math.cos(angle);
            translationY.value = clampedDistance*Math.sin(angle);
        }).onEnd(() => {
            translationX.value = withSpring(0);
            translationY.value = withSpring(0);
        }).onFinalize(() => {
            translationX.value = withSpring(0);
            translationY.value = withSpring(0);
        });

    const animatedStyle = useAnimatedStyle(() => ({
        transform: [
            {translateX: translationX.value},
            {translateY: translationY.value} 
        ]
    }));

    return (
        <GestureDetector gesture={pan}>
            <Animated.View style={joystickStyles.joystickBackgroundCircle}>
                <Animated.View style={[joystickStyles.joystickMovingPartCircle, animatedStyle]}/>
            </Animated.View>
        </GestureDetector>
    );

}

function isAngleInRange(min: number, max: number, angle: number){
    'worklet';
    if(min <= max){ //for example min = -30 max = 30
        return min <= angle && angle <= max;
    }else{ //we are in the strange case where min > max for example min = 150 max = -150
        return angle >= min || angle <= max;
    }
}

const joystickStyles = StyleSheet.create({
    joystickBackgroundCircle: {
        justifyContent: 'center',
        alignItems: 'center',
        width: backgroundWidth,
        height: backgroundHeight,
        backgroundColor: `${orangeColor}80`,
        borderRadius: 100
    },
    joystickMovingPartCircle: {
        width: backgroundWidth / 2,
        height: backgroundHeight / 2,
        backgroundColor: orangeColor,
        borderRadius: 50
    }
});