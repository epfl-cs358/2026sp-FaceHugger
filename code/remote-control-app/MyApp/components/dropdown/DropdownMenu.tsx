import { useEffect, useState } from "react"
import { Pressable, View, Text, StyleSheet } from "react-native";
import {Ionicons} from "@expo/vector-icons";
import { AppText } from "../text/AppText";
import { darkerOrangeColor, orangeColor } from "../../colors/colors";

export interface DropDownMenuElement{
    key: string,
    elementTitle: string,
    onClick: () => void
}

export interface DropDownMenuProps {
    defaultElement: DropDownMenuElement,
    elements: Array<DropDownMenuElement>
}

const dropdownMenuTextSize = 24;

export function DropDownMenu({defaultElement, elements}: DropDownMenuProps){
    const [dropdownOpen, setDropdownOpen] = useState(false);
    const [selectedElement, setSelectedElement] = useState(defaultElement.elementTitle);
    useEffect(() => {
        defaultElement.onClick();
    }, []);

    return (<View style={styles.mainContainer}>
        {dropdownOpen && <View style={styles.actualDropDownContainer}>
            {elements.filter((e) => e.elementTitle != selectedElement).map((e) => <DropDownMenuElement key={e.elementTitle} elementTitle={e.elementTitle} onClick={() => {
                setDropdownOpen(false);
                setSelectedElement(e.elementTitle);
                e.onClick();
                }
            }/>)}
        </View>} 

        <Pressable onPress={() => setDropdownOpen(!dropdownOpen)} style={styles.dropdownSelectedButtonElement}>
            <View style={styles.dropdownSelectedElementText}>
                <AppText text={selectedElement} size={dropdownMenuTextSize}/>
            </View>
            <View style={styles.dropDownSelectedElementIcon}>
                <Ionicons name="chevron-down" size={dropdownMenuTextSize} color="white" />
            </View>
        </Pressable>
    </View>);
}

function DropDownMenuElement({elementTitle, onClick}: DropDownMenuElement){
    return(
        <Pressable onPress={onClick} style={dropDownMenuElementStyles.buttonStyle}>
            <AppText text={elementTitle} size={dropdownMenuTextSize}/>
        </Pressable>
    );
}

const dropDownMenuButtonHeight = 50;
const dropDownMenuButtonWidth = 120;

const styles = StyleSheet.create({
    mainContainer: {
        justifyContent:'center',
        alignItems:'center',
        position: 'relative'
    },
    dropdownSelectedButtonElement: {
        marginRight: 10,
        backgroundColor: darkerOrangeColor,
        borderRadius: 12,
        flexDirection: 'row',
        justifyContent:'center',
        alignItems:'center',
        height: dropDownMenuButtonHeight,
        width: dropDownMenuButtonWidth
    },
    dropdownSelectedElementText: {
        paddingLeft: 8,
        flex: 8
    },
    dropDownSelectedElementIcon: {
        flex: 2
    },
    actualDropDownContainer: {
        position: 'absolute',
        bottom: dropDownMenuButtonHeight, // open upward; trigger sits at bottom of screen
        right: 10,
        justifyContent: 'center',
        alignItems: 'center',
        zIndex: 999,
        backgroundColor: '#121212', // so it covers content beneath it
        borderRadius: 8,
        elevation: 4,        // Android shadow
        shadowColor: '#000', // iOS shadow
        shadowOffset: { width: 0, height: 2 },
        shadowOpacity: 0.2,
        shadowRadius: 4,
    }
});

const dropDownMenuElementStyles = StyleSheet.create({
    buttonStyle: {
        height: dropDownMenuButtonHeight,
        width: dropDownMenuButtonWidth,
        justifyContent: 'center',
        alignItems: 'center'
    }
});