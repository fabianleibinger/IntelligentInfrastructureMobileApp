import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/dialogs/parkoutdialog.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';

class MainPage extends StatefulWidget {
  final String apikey;

  const MainPage({Key key, this.apikey}) : super(key: key);
  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Vehicle', style: whiteHeader),
        backgroundColor: green,
      ),
      drawer: Drawer(
        child: ListView(
          padding: EdgeInsets.zero,
          children: <Widget>[
            DrawerHeader(
              decoration: BoxDecoration(
                color: green,
              ),
              child: Text('Parkhaus', style: blackHeader),
            ),
            ListTile(
              leading: Icon(Icons.message),
              title: Text('Messages'),
            ),
            ListTile(
              leading: Icon(Icons.account_circle),
              title: Text('Profile'),
            ),
            ListTile(
              leading: Icon(Icons.settings),
              title: Text('Settings'),
            ),
          ],
        ),
      ),
    );
  }
}
