import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/user_bloc_provider.dart';
import 'package:parkingapp/dialogs/example_dialog.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';

// example for a page (mainpage)

class MainPage extends StatefulWidget {
  final VoidCallback login;
  final String apikey;

  const MainPage({Key key, this.login, this.apikey}) : super(key: key);
  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey,
      body: Center(
        child: getMainPage(),
      ),
    );
  }

  Widget getMainPage() {
    return MaterialApp(
      color: Colors.yellow,
      home: SafeArea(
        child: DefaultTabController(
          length: 3,
          child: new Scaffold(
            body: Stack(children: <Widget>[
              TabBarView(
                children: [
                  new Container(
                    color: Colors.orange,
                  ),
                  new Container(
                    color: Colors.orange,
                  ),
                  new Container(
                    child: Center(
                      child: FlatButton(
                        color: Colors.red,
                        child: Text("Log out"),
                        onPressed: () {
                          logout();
                        },
                      ),
                    ),
                    color: Colors.lightGreen,
                  ),
                ],
              ),
              Container(
                padding: EdgeInsets.only(left: 50),
                height: 150,
                decoration: BoxDecoration(
                  borderRadius: BorderRadius.only(
                      bottomLeft: Radius.circular(50),
                      bottomRight: Radius.circular(50)),
                  color: Colors.white,
                ),
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: <Widget>[Text("Intray"), Container()],
                ),
              ),
              Container(
                height: 80,
                width: 80,
                margin: EdgeInsets.only(
                    top: 110,
                    left: MediaQuery.of(context).size.width * 0.5 - 40),
                child: FloatingActionButton(
                    child: Icon(Icons.add, size: 70),
                    backgroundColor: Colors.red,
                    onPressed: () {
                      showDialog(
                          context: context,
                          builder: (BuildContext context) {
                            return ExampleDialog(apikey: widget.apikey);
                          });
                    }),
              )
            ]),
            appBar: AppBar(
              elevation: 0,
              title: new TabBar(
                tabs: [
                  Tab(
                    icon: new Icon(Icons.home),
                  ),
                  Tab(
                    icon: new Icon(Icons.rss_feed),
                  ),
                  Tab(
                    icon: new Icon(Icons.perm_identity),
                  ),
                ],
                labelColor: Colors.grey,
                unselectedLabelColor: Colors.blue,
                indicatorSize: TabBarIndicatorSize.label,
                indicatorPadding: EdgeInsets.all(5.0),
                indicatorColor: Colors.transparent,
              ),
              backgroundColor: Colors.white,
            ),
            backgroundColor: Colors.white,
          ),
        ),
      ),
    );
  }

  void saveApiKey(User user) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('API_Token', user.apikey);
  }

  logout() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('API_Token', "");
    setState(() {
      build(context);
    });
  }
}
