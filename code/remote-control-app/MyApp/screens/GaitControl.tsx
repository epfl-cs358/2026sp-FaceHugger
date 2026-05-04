import { View } from "react-native";
import Joystick, { JoystickAction, JoystickProps } from "../components/joystick/Joystick";

const actionRight = {
    minAngle: -15,
    maxAngle: 15,
    action: () => console.log("Right action")
} as JoystickAction;

const actionLeft = {
    minAngle: 165,
    maxAngle: -165,
    action: () => console.log("Left action")
} as JoystickAction;

const actionForwardRight = {
    minAngle: 15,
    maxAngle: 75,
    action: () => console.log("Forward right")
} as JoystickAction;

const actionForward = {
    minAngle: 75,
    maxAngle: 105,
    action: () => console.log("Forward")
} as JoystickAction;

const actionForwardLeft = {
    minAngle: 105, 
    maxAngle: 165,
    action: () => console.log("Forward left")
} as JoystickAction;

const actionBackwardLeft = {
    minAngle: -165,
    maxAngle: -105,
    action: () => console.log("Backward left")
} as JoystickAction;

const actionBackward = {
    minAngle: -105,
    maxAngle: -75,
    action: () => console.log("Backward")
}

const actionBackwardRight = {
    minAngle: -75,
    maxAngle: -15,
    action: () => console.log("Backward right")
}

const gestureProps = {
        actions: [actionRight, actionLeft, actionForwardRight, actionForward, actionForwardLeft, actionBackwardLeft, actionBackward, actionBackwardRight]
} as JoystickProps;
export default function GaitControl(){
    return(
        <View>
            <Joystick {...gestureProps}/>
        </View>
    );
}