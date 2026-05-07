import { useEffect, useState } from "react"
import { Pressable, View, Text, StyleSheet } from "react-native";
import {Ionicons} from "@expo/vector-icons";
import { AppText } from "../text/AppText";

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
        defaultElement.onClick(); //Click the element just in case any side effects need to happen
    });

    return (<View style={styles.mainContainer}>
        <Pressable onPress={() => setDropdownOpen(true)} style={styles.dropdownSelectedButtonElement}>
            <View style={styles.dropdownSelectedElementText}>
                <AppText text={selectedElement} size={dropdownMenuTextSize}/>
            </View>
            <View style={styles.dropDownSelectedElementIcon}>
                <Ionicons name="chevron-down" size={dropdownMenuTextSize} color="black" />
            </View>
        </Pressable>

        {dropdownOpen} 
        && <View style={styles.actualDropDownContainer}>
            {elements.filter((e) => e.elementTitle != selectedElement).map((e) => <DropDownMenuElement key={e.elementTitle} elementTitle={e.elementTitle} onClick={() => {
                e.onClick();
                setDropdownOpen(false);
                }
            }/>)}
        </View>
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
const dropDownMenuButtonWidth = 100;

const styles = StyleSheet.create({
    mainContainer: {
        justifyContent:'center',
        alignItems:'center'
    },
    dropdownSelectedButtonElement: {
        height: dropDownMenuButtonHeight,
        width: dropDownMenuButtonWidth
    },
    dropdownSelectedElementText: {
        flex: 8
    },
    dropDownSelectedElementIcon: {
        flex: 2
    },
    actualDropDownContainer: {
        justifyContent: 'center',
        alignItems: 'center'
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