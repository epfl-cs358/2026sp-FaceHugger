import React, { useEffect, useMemo, useRef, useState } from 'react';
import { ScrollView, StyleSheet, TouchableOpacity, View } from 'react-native';
import { useRobotStore } from '../store/robotStore';
import { playClip } from '../api/api-messages';
import { STREAM_CLIPS, StreamClip } from '../api/streamClips';
import { streamClip, stopStream } from '../services/clipStreamer';
import { ClipInfo } from '../api/api-types';
import { orangeColor } from '../colors/colors';
import { AppText } from './text/AppText';

type Mode = 'all' | 'flashed' | 'app';

// One name may exist as a flashed (on-robot, T:7) clip, an app (streamed, T:4)
// clip, or both — we keep both rather than dedup, so an authored clip is
// playable before it's flashed.
type Row = { name: string; ms: number; flashed?: ClipInfo; app?: StreamClip };

const buildRows = (flashed: ClipInfo[], app: StreamClip[]): Row[] => {
  const byName = new Map<string, Row>();
  const order: string[] = [];
  for (const c of flashed) {
    byName.set(c.name, { name: c.name, ms: c.ms, flashed: c });
    order.push(c.name);
  }
  for (const c of app) {
    const existing = byName.get(c.name);
    if (existing) {
      existing.app = c;
    } else {
      byName.set(c.name, { name: c.name, ms: c.duration_ms, app: c });
      order.push(c.name);
    }
  }
  return order.map((n) => byName.get(n)!);
};

export function ClipList() {
  const flashed = useRobotStore((s) => s.clips);
  const clipPlaying = useRobotStore((s) => s.clipPlaying);
  const setClipPlaying = useRobotStore((s) => s.setClipPlaying);
  const [mode, setMode] = useState<Mode>('all');
  const timerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(
    () => () => {
      if (timerRef.current) clearTimeout(timerRef.current);
      stopStream();
    },
    [],
  );

  const rows = useMemo(() => buildRows(flashed, STREAM_CLIPS), [flashed]);
  const visible = useMemo(() => {
    if (mode === 'flashed') return rows.filter((r) => r.flashed);
    if (mode === 'app') return rows.filter((r) => r.app);
    return rows;
  }, [rows, mode]);

  // Play on the robot (firmware clip, one T:7 — the ESP32 owns the timing).
  const onRobot = (clip: ClipInfo) => {
    if (useRobotStore.getState().clipPlaying) return;
    setClipPlaying(true);
    playClip(clip.id);
    timerRef.current = setTimeout(() => setClipPlaying(false), clip.ms + 500);
  };

  // Stream from the app (T:4 frames over the live socket — no flash needed).
  const onApp = (clip: StreamClip) => {
    if (useRobotStore.getState().clipPlaying) return;
    setClipPlaying(true);
    streamClip(clip, () => setClipPlaying(false));
    // Safety release in case onDone never fires (socket dropped mid-stream).
    timerRef.current = setTimeout(
      () => setClipPlaying(false),
      clip.duration_ms + clip.frame_ms + 1000,
    );
  };

  if (rows.length === 0) return null;

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <AppText text="Clips" size={16} color="#ffffff" />
        <View style={styles.segment}>
          {(['all', 'flashed', 'app'] as Mode[]).map((m) => (
            <TouchableOpacity
              key={m}
              style={[styles.segItem, mode === m && styles.segItemActive]}
              onPress={() => setMode(m)}
            >
              <AppText
                text={m === 'all' ? 'All' : m === 'flashed' ? 'Flashed' : 'App'}
                size={12}
                color={mode === m ? '#121212' : '#ccc'}
              />
            </TouchableOpacity>
          ))}
        </View>
      </View>

      {visible.length === 0 ? (
        <AppText
          text={
            mode === 'flashed'
              ? 'No flashed clips — connect to the robot to load them.'
              : 'No app clips bundled.'
          }
          size={12}
          color="#888"
        />
      ) : (
        <ScrollView style={styles.scroll} contentContainerStyle={styles.scrollContent}>
          {visible.map((row) => (
            <View key={row.name} style={styles.item}>
              <View style={styles.itemInfo}>
                <AppText text={row.name} size={14} color={clipPlaying ? '#888' : '#fff'} />
                <AppText text={`${(row.ms / 1000).toFixed(1)}s`} size={12} color="#aaa" />
              </View>
              <View style={styles.itemButtons}>
                {row.flashed && (
                  <TouchableOpacity
                    style={[styles.playBtn, clipPlaying && styles.disabled]}
                    onPress={() => onRobot(row.flashed!)}
                    disabled={clipPlaying}
                  >
                    <AppText text="Robot" size={13} color="#fff" />
                  </TouchableOpacity>
                )}
                {row.app && (
                  <TouchableOpacity
                    style={[styles.playBtn, styles.appBtn, clipPlaying && styles.disabled]}
                    onPress={() => onApp(row.app!)}
                    disabled={clipPlaying}
                  >
                    <AppText text="App" size={13} color="#121212" />
                  </TouchableOpacity>
                )}
              </View>
            </View>
          ))}
        </ScrollView>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: { gap: 8, alignSelf: 'stretch' },
  header: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' },
  segment: { flexDirection: 'row', backgroundColor: '#1a1a2e', borderRadius: 8, overflow: 'hidden' },
  segItem: { paddingVertical: 6, paddingHorizontal: 12 },
  segItemActive: { backgroundColor: orangeColor },
  scroll: { maxHeight: 260 },
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
  itemInfo: { gap: 2 },
  itemButtons: { flexDirection: 'row', gap: 8 },
  playBtn: {
    paddingVertical: 6,
    paddingHorizontal: 14,
    borderRadius: 8,
    backgroundColor: '#2a2a2a',
  },
  appBtn: { backgroundColor: orangeColor },
  disabled: { opacity: 0.4 },
});
