import { StatusBar } from 'expo-status-bar';
import { StyleSheet, Text, View } from 'react-native';
import Pager, { PageInfo, PagerProps } from './components/pager/Pager';
import Ionicons from '@expo/vector-icons/Ionicons';
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons';
import GaitControl from './screens/GaitControl';
import LegControl from './screens/LegControl';
import { orangeColor } from './colors/colors';
import { GestureHandlerRootView } from 'react-native-gesture-handler';

export default function App() {
  const remoteControlPage = {
    pageName: 'Remote control',
    pageIcon: <Ionicons name="game-controller-outline" size={40} color="white" />,
    pageIconPressed: <Ionicons name="game-controller" size={40} color={orangeColor} />,
    pageComponent: <GaitControl/>
  } as PageInfo;

  const individualControlPage = {
    pageName: 'Individual control',
    pageIcon: <MaterialCommunityIcons name="engine-outline" size={40} color="white" />,
    pageIconPressed: <MaterialCommunityIcons name="engine" size={40} color={orangeColor} />,
    pageComponent: <LegControl/>
  } as PageInfo;

  const pagerProps = {
    defaultPage: remoteControlPage,
    pages: [remoteControlPage, individualControlPage]
  } as PagerProps;

  const pages = [remoteControlPage, individualControlPage]

  return (
    <GestureHandlerRootView>
      <View style={styles.container}>
          <Pager {...pagerProps}/>
      </View>
    </GestureHandlerRootView>
  );
}

const styles = StyleSheet.create({
  container: {
    width:'100%',
    height:'100%',
    flex: 1,
    backgroundColor: '#121212',
    justifyContent: 'center',
  },
});
