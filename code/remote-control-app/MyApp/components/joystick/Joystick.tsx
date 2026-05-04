import { GestureHandlerRootView, GestureDetector, Gesture } from "react-native-gesture-handler";
import { View, StyleSheet } from "react-native";
import { orangeColor } from "../../colors/colors";
import Animated, {useSharedValue, useAnimatedStyle} from 'react-native-reanimated'

export interface JoystickAction{
    minAngle: number
    maxAngle: number //angles should be in the -180 180 range
    action: () => void
}

export interface JoystickProps{
    actions: JoystickAction[]
}

export default function Joystick({actions}: JoystickProps){
    const translationX = useSharedValue(0);
    const translationY = useSharedValue(0);

    const pan = Gesture.Pan()
        .onUpdate((e) => {
            translationX.value = e.translationX;
            translationY.value = e.translationY;
        }).onEnd(() => {
            translationX.value = 0;
            translationY.value = 0;
        });

    const animatedStyle = useAnimatedStyle(() => ({
        transform: [
            {translateX: translationX.value},
            {translateY: translationY.value} 
        ]
    }));

    return (
        <View style={joystickStyles.joystickBackgroundCircle}>
            <GestureDetector gesture={pan}>
                <Animated.View style={[joystickStyles.joystickMovingPartCircle, animatedStyle]}>
                
                </Animated.View>
            </GestureDetector>
        </View>
    );

}

function isAngleInRange(min: number, max: number, angle: number){
    if(min <= max){ //for example min = -30 max = 30
        return min <= angle && angle <= max;
    }else{ //we are in the strange case where min > max for example min = 150 max = -150
        return angle >= min || angle <= max;
    }
}

const backgroundWidth = 200;
const backgroundHeight = 200;

const joystickStyles = StyleSheet.create({
    joystickBackgroundCircle: {
        justifyContent: 'center',
        alignItems: 'center',
        width: backgroundWidth,
        height: backgroundHeight,
        opacity: 0.5,
        backgroundColor: orangeColor,
        borderRadius: 100
    },
    joystickMovingPartCircle: {
        width: backgroundWidth / 2,
        height: backgroundHeight / 2,
        backgroundColor: orangeColor,
        borderRadius: 50
    }
});