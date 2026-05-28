import React, { useEffect, useMemo, useRef, useState } from 'react';
import { ScrollView, StyleSheet, TouchableOpacity, View } from 'react-native';
import { useRobotStore } from '../store/robotStore';
import { playClip, stopClipPlayback } from '../api/api-messages';
import { STREAM_CLIPS, StreamClip } from '../api/streamClips';
import { streamClip, stopStream } from '../services/clipStreamer';
import { ClipInfo } from '../api/api-types';
import { orangeColor } from '../colors/colors';
import { AppText } from './text/AppText';

type Mode = 'all' | 'flashed' | 'app';
type Playing = { name: string; source: 'robot' | 'app'; loop: boolean } | null;

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
  const [loop, setLoop] = useState(false);
  const [playing, setPlaying] = useState<Playing>(null);
  const timerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const clearTimer = () => {
    if (timerRef.current) {
      clearTimeout(timerRef.current);
      timerRef.current = null;
    }
  };
  const finish = () => {
    setClipPlaying(false);
    setPlaying(null);
  };

  useEffect(
    () => () => {
      clearTimer();
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

  // Play on the robot (firmware clip, T:7 — the ESP32 owns the timing).
  const onRobot = (row: Row, clip: ClipInfo) => {
    if (useRobotStore.getState().clipPlaying) return;
    setClipPlaying(true);
    setPlaying({ name: row.name, source: 'robot', loop });
    playClip(clip.id, loop);
    // A looping clip runs until stopped; a one-shot clears itself after its run.
    if (!loop) timerRef.current = setTimeout(finish, clip.ms + 500);
  };

  // Stream from the app (T:4 frames over the live socket — no flash needed).
  const onApp = (row: Row, clip: StreamClip) => {
    if (useRobotStore.getState().clipPlaying) return;
    setClipPlaying(true);
    setPlaying({ name: row.name, source: 'app', loop });
    streamClip(clip, finish, loop);
    // Safety release for one-shots in case onDone never fires (socket dropped).
    if (!loop)
      timerRef.current = setTimeout(finish, clip.duration_ms + clip.frame_ms + 1000);
  };

  const onStop = () => {
    clearTimer();
    if (playing?.source === 'robot') stopClipPlayback();
    else stopStream();
    finish();
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

      <View style={styles.controlRow}>
        <TouchableOpacity
          style={[styles.loopToggle, loop && styles.loopToggleOn]}
          onPress={() => setLoop((v) => !v)}
        >
          <AppText text={loop ? 'Loop: on' : 'Loop: off'} size={12} color={loop ? '#121212' : '#ccc'} />
        </TouchableOpacity>
        {playing && (
          <TouchableOpacity style={styles.stopBtn} onPress={onStop}>
            <AppText text="Stop" size={13} color="#fff" />
          </TouchableOpacity>
        )}
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
          {visible.map((row) => {
            const isPlaying = playing?.name === row.name;
            return (
              <View key={row.name} style={[styles.item, isPlaying && styles.itemPlaying]}>
                <View style={styles.itemInfo}>
                  <AppText text={row.name} size={14} color={clipPlaying && !isPlaying ? '#888' : '#fff'} />
                  <AppText
                    text={
                      isPlaying
                        ? playing!.loop
                          ? '↻ looping'
                          : '▶ playing'
                        : `${(row.ms / 1000).toFixed(1)}s`
                    }
                    size={12}
                    color={isPlaying ? orangeColor : '#aaa'}
                  />
                </View>
                <View style={styles.itemButtons}>
                  {row.flashed && (
                    <TouchableOpacity
                      style={[styles.playBtn, clipPlaying && styles.disabled]}
                      onPress={() => onRobot(row, row.flashed!)}
                      disabled={clipPlaying}
                    >
                      <AppText text="Robot" size={13} color="#fff" />
                    </TouchableOpacity>
                  )}
                  {row.app && (
                    <TouchableOpacity
                      style={[styles.playBtn, styles.appBtn, clipPlaying && styles.disabled]}
                      onPress={() => onApp(row, row.app!)}
                      disabled={clipPlaying}
                    >
                      <AppText text="App" size={13} color="#121212" />
                    </TouchableOpacity>
                  )}
                </View>
              </View>
            );
          })}
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
  controlRow: { flexDirection: 'row', alignItems: 'center', gap: 12 },
  loopToggle: { paddingVertical: 6, paddingHorizontal: 12, borderRadius: 8, backgroundColor: '#1a1a2e' },
  loopToggleOn: { backgroundColor: orangeColor },
  stopBtn: { paddingVertical: 6, paddingHorizontal: 16, borderRadius: 8, backgroundColor: '#e02b02' },
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
  itemPlaying: { borderWidth: 1.5, borderColor: orangeColor },
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
