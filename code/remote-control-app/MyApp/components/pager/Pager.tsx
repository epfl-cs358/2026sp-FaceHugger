import { ReactNode, useState } from "react"
import { Button, Pressable, ScrollView, StyleSheet, Text, View, Image } from "react-native"
import { AppText } from "../text/AppText";

export interface PageInfo {
    pageName: string,
    pageIcon: ReactNode, //IonIcon MaterialIcon etc
    pageIconPressed: ReactNode,
    pageComponent: ReactNode //The actual component of the page to render
}

export interface PagerProps {
    defaultPage: PageInfo,
    pages: Array<PageInfo> 
}

export default function Pager({defaultPage, pages}: PagerProps){
    const [page, setPage] = useState(defaultPage);
    return(<View style={styles.mainContainer}>
        <View style={styles.pageImageContainer}>
            <Image style={styles.pageImage} source={require('./../../assets/facehugger.png')}/>
        </View>
        <View style={styles.pageContainer}>
            {page.pageComponent}
        </View>
        <View style={styles.pageButtonsContainer}>
            {pages.map((pageInfo, index) => {
                return(
                    <Pressable style={styles.pageButton} key={pageInfo.pageName} onPress={() => setPage(pageInfo)}>
                        {page.pageName === pageInfo.pageName ? pageInfo.pageIconPressed : pageInfo.pageIcon}
                        <AppText text={pageInfo.pageName} size={20}/>
                    </Pressable>
                );
            })}
        </View>
    </View>);
}

const styles = StyleSheet.create({
    mainContainer: {
        flex:1,
        justifyContent: 'center'
    },
    pageButtonsContainer: {
        flex: 2,
        alignItems:'center',
        justifyContent:'center',
        flexDirection: 'row',
        gap:20,
    },
    pageImageContainer: {
      flexDirection: 'row',
      flex: 1,
      backgroundColor: '#1b2023',
      justifyContent:'center',
      alignItems: 'center',
      overflow: 'hidden'
    },
    pageImage: {
        aspectRatio: 1,
        resizeMode: 'contain',
        height: 90,
        width: 90
    },
    pageContainer: {
        justifyContent: 'center',
        alignItems:'center',
        flex: 8
    },
    pageButton: {
        justifyContent: 'center',
        alignItems: 'center'

    }
});