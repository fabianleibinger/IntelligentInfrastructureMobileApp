import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/loginpage/loginpage.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/dialogs/example_dialog.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/bloc/blocs/user_bloc_provider.dart';

// Main: From here you call all u'r widgets.

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        debugShowCheckedModeBanner: false,
        title: 'Parking App',
        theme: ThemeData(
          primarySwatch: Colors.blue,
          visualDensity: VisualDensity.adaptivePlatformDensity,
          dialogBackgroundColor: Colors.transparent,
        ),
        home: MyHomePage());
  }
}

class MyHomePage extends StatefulWidget {
  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  String apikey;
  @override
  Widget build(BuildContext context) {
    return FutureBuilder(
      future: signinUser(),
      builder: (BuildContext context, AsyncSnapshot snapshot) {
        if (snapshot.hasData) {
          apikey = snapshot.data;
          print("API KEY: " + apikey);
        } else {
          print("no data user (main)");
        }
        return apikey.length > 0
            ? MainPage(apikey: apikey)
            : LoginPage(
                login: login,
              );
      },
    );
  }

  void login() {
    setState(() {
      build(context);
    });
  }

  @override
  void initState() {
    super.initState();
    ;
  }

  Future signinUser() async {
    String username = "";
    apikey = await getApiKey();
    if (apikey.length > 0) {
      userBloc.signinUser("", apikey);
    } else {
      print("No api key");
    }
    return apikey;
  }

  Future getApiKey() async {
    final prefs = await SharedPreferences.getInstance();
    //await prefs.clear();
    return prefs.getString('API_Token');
  }
}
