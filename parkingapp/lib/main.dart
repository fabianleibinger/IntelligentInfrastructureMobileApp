import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/firststartpage/firststartpage.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:parkingapp/bloc/blocs/userbloc.dart';

// Main: From here you call all u'r widgets.

void main() {
  runApp(Main());
}

class Main extends StatelessWidget {
  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        debugShowCheckedModeBanner: false,
        title: 'Parking App',
        theme: themeData,
        home: MyHomePage());
  }
}

class MyHomePage extends StatefulWidget {
  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  @override
  Widget build(BuildContext context) {
    return MainPage();
  }

  void login() {
    setState(() {
      build(context);
    });
  }

  @override
  void initState() {
    super.initState();
  }
}
