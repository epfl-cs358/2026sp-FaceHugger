import { View } from "react-native";
import Joystick, { JoystickProps } from "../components/joystick/Joystick";

export default function GaitControl(){
    const gestureProps = {
        actions: []
    } as JoystickProps;
    return(
        <View>
            <Joystick {...gestureProps}/>
        </View>
    );
}