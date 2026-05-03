import { ReactNode, useState } from "react"
import { Button, Pressable, StyleSheet, Text, View } from "react-native"

interface PageInfo {
    pageName: string,
    pageIcon: ReactNode, //IonIcon MaterialIcon etc
    pageComponent: ReactNode //The actual component of the page to render
}

interface PagerProps {
    defaultPageName: string,
    pages: Array<PageInfo> 
}

export default function Pager({defaultPageName, pages}: PagerProps){
    const [page, setPage] = useState(defaultPageName)
    return(<View style={styles.mainContainer}>
        <View style={styles.pageButtonsContainer}>
            {pages.map((pageInfo, index) => {
                return(
                    <Pressable key={pageInfo.pageName} onPress={() => setPage(pageInfo.pageName)}>
                        {pageInfo.pageIcon}
                        <Text>{pageInfo.pageName}</Text>
                    </Pressable>
                );
            })}
        </View>
        <View style={styles.pageContainer}>
            {pages.find((pageInfo) => pageInfo.pageName === page)?.pageComponent}
        </View>
    </View>);
}

const styles = StyleSheet.create({
    mainContainer: {

    },
    pageButtonsContainer: {

    },
    pageContainer: {

    }
});