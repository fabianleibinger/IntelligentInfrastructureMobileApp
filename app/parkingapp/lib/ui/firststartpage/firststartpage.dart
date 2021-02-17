import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';

class FirstStartPage extends StatefulWidget {
  final VoidCallback login;

  const FirstStartPage({Key key, this.login}) : super(key: key);
  @override
  _FirstStartPageState createState() => _FirstStartPageState();
}

class _FirstStartPageState extends State<FirstStartPage> {
  TextEditingController usernameController = new TextEditingController();
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey,
      body: Center(
        child: Container(
          margin: EdgeInsets.only(top: 100, left: 20, right: 20, bottom: 100),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: <Widget>[
              Text(
                "Welcome!",
              ),
              Container(
                height: 200,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.spaceAround,
                  children: <Widget>[],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
