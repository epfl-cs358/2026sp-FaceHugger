import { Text } from "react-native";

export interface AppTextProps {
    text: string,
    size?: number
    color?: string
}

export function AppText({text, size = 10, color = '#ffffff'}: AppTextProps){
    return(
        <Text style={{fontSize: size, color: color}}>
            {text}
        </Text>
    );
}