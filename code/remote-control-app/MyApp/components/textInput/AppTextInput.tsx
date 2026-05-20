import { useEffect, useState } from "react";
import { StyleSheet, TextInput, View } from "react-native";
import { AppText } from "../text/AppText";
import { orangeColor } from "../../colors/colors";

export type AppTextInputProps = {
    label?: string,
    value: number | null,
    onChange: (value: number) => void,
};

export function AppTextInput({ label, value, onChange }: AppTextInputProps) {
    const [focused, setFocused] = useState(false);
    const [error, setError] = useState(false);
    const [text, setText] = useState(value !== null ? String(value) : '');

    // Mirror external prop updates only when the user isn't actively editing.
    useEffect(() => {
        if (!focused) setText(value !== null ? String(value) : '');
    }, [value, focused]);

    function handleChange(raw: string) {
        const cleaned = raw.replace(/[^0-9]/g, '').slice(0, 3);
        setText(cleaned);

        if (cleaned === '') {
            setError(false);
            return;
        }
        const parsed = Number(cleaned);
        if (parsed < 0 || parsed > 180) {
            setError(true);
            return;
        }
        setError(false);
        onChange(parsed);
    }

    function handleBlur() {
        setFocused(false);
        // Snap back to the last committed value if the user left the field empty or invalid.
        if (text === '' || error) {
            setText(value !== null ? String(value) : '');
            setError(false);
        }
    }

    const borderColor = error ? '#ff4d4d' : focused ? orangeColor : '#2a2a2a';

    return (
        <View style={styles.container}>
            <TextInput
                style={[styles.input, { borderColor }]}
                keyboardType="numeric"
                value={text}
                onChangeText={handleChange}
                onFocus={() => setFocused(true)}
                onBlur={handleBlur}
                placeholderTextColor="#555555"
                placeholder="0–180"
                selectionColor={orangeColor}
                maxLength={3}
            />
            {error && <AppText text="Enter a whole number between 0 and 180" size={11} color="#ff4d4d" />}
        </View>
    );
}

const styles = StyleSheet.create({
    container: {
        gap: 4,
    },
    input: {
        backgroundColor: '#1e1e1e',
        color: '#ffffff',
        borderWidth: 2,
        borderRadius: 10,
        paddingVertical: 8,
        paddingHorizontal: 14,
        fontSize: 16,
        minWidth: 100,
    },
});
