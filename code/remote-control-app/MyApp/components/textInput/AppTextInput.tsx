import { useState } from "react";
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

    function handleChange(raw: string) {
        const trimmed = raw.trim();
        if (trimmed === '') {
            setError(false);
            return;
        }
        const parsed = Number(trimmed);
        if (!Number.isInteger(parsed) || parsed < 0 || parsed > 180) {
            setError(true);
            return;
        }
        setError(false);
        onChange(parsed);
    }

    const borderColor = error ? '#ff4d4d' : focused ? orangeColor : '#2a2a2a';

    return (
        <View style={styles.container}>
            <TextInput
                style={[styles.input, { borderColor }]}
                keyboardType="numeric"
                value={value !== null ? String(value) : ''}
                onChangeText={handleChange}
                onFocus={() => setFocused(true)}
                onBlur={() => setFocused(false)}
                placeholderTextColor="#555555"
                placeholder="0–180"
                selectionColor={orangeColor}
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
