import { TouchableOpacity } from "react-native";
import { AppText } from "../text/AppText";
import { orangeColor } from "../../colors/colors";

export type IndividualSelectionProps = {
    selected: boolean,
    title: string,
    onClick: () => void
};

export function IndividualSelectionButton({selected, title, onClick}: IndividualSelectionProps){
    return(<TouchableOpacity onPress={onClick} style={{justifyContent:'center', alignItems:'center', borderRadius: 12, paddingVertical: 8, paddingHorizontal: 16, backgroundColor: selected ? orangeColor : '#2a2a2a'}}>
        <AppText text={title} size={15} color={'#ffffff'}/>
    </TouchableOpacity>);
}