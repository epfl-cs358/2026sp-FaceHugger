import React, { useEffect, useRef } from 'react';
import { ScrollView, StyleSheet, TouchableOpacity, View } from 'react-native';
import { useRobotStore } from '../store/robotStore';
import { playClip } from '../api/api-messages';
import { AppText } from './text/AppText';

export function ClipList() {
  const clips = useRobotStore((s) => s.clips);
  const clipPlaying = useRobotStore((s) => s.clipPlaying);
  const setClipPlaying = useRobotStore((s) => s.setClipPlaying);
  const timerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => () => { if (timerRef.current) clearTimeout(timerRef.current); }, []);

  const onPressClip = (id: number, ms: number) => {
    if (useRobotStore.getState().clipPlaying) return;   // synchronous read
    setClipPlaying(true);
    playClip(id);
    timerRef.current = setTimeout(() => setClipPlaying(false), ms + 500);
  };

  if (clips.length === 0) return null;

  return (
    <View style={styles.container}>
      <AppText text="Clips" size={16} color="#ffffff" />
      <ScrollView style={styles.scroll} contentContainerStyle={styles.scrollContent}>
        {clips.map((clip) => (
          <TouchableOpacity
            key={clip.id}
            style={[styles.item, clipPlaying && styles.disabled]}
            onPress={() => onPressClip(clip.id, clip.ms)}
            disabled={clipPlaying}
          >
            <AppText text={clip.name} size={14} color={clipPlaying ? '#888' : '#fff'} />
            <AppText text={`${(clip.ms / 1000).toFixed(1)}s`} size={12} color="#aaa" />
          </TouchableOpacity>
        ))}
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { gap: 8 },
  scroll:    { maxHeight: 220 },
  scrollContent: { gap: 8 },
  item: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingVertical: 10,
    paddingHorizontal: 14,
    backgroundColor: '#1a1a2e',
    borderRadius: 8,
  },
  disabled: { opacity: 0.4 },
});
